"""One-shot RGB-D bottle detection; retrigger with /vision/detect."""

import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import torch
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_msgs.msg import Bool
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # Register PointStamped transformations.
from ultralytics import YOLO

from vision_guided_simulation.geometry import backproject, median_depth, downward_target
from vision_guided_simulation.images import image_array, image_message


class CameraVision(Node):
    def __init__(self):
        super().__init__('camera_vision')
        defaults = {'model_path': 'yolov8n.pt', 'confidence': .4,
                    'hand_clearance': .18, 'bottle_radius': .046,
                    'output_dir': str(Path.home() / 'vision_guided_ws/work/gazebo_results')}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        torch.set_num_threads(2)
        self.model = YOLO(self.get_parameter('model_path').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.rgb = self.depth = self.info = None
        self.frames = {'rgb': {}, 'depth': {}}
        self.lock = threading.Lock()
        self.armed = True
        self.worker = None
        self.pending = None
        self.last_wait_log = time.monotonic()
        self.scene_ready = False
        self.controllers_ready = False
        self.controllers_future = None
        self.motion_busy = False
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.output = Path(self.get_parameter('output_dir').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.target_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.object_pub = self.create_publisher(PointStamped, '/vision/object_position', 10)
        self.image_pub = self.create_publisher(Image, '/vision/annotated_image', 1)
        self.report_pub = self.create_publisher(String, '/vision/detection_report', 10)
        for topic, field in [('/camera/image', 'rgb'), ('/camera/depth_image', 'depth')]:
            self.create_subscription(Image, topic, lambda msg, f=field: self.store(f, msg),
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(CameraInfo, '/camera/camera_info', lambda msg: self.store('info', msg), qos_profile_sensor_data)
        self.create_service(Trigger, '/vision/detect', self.trigger)
        self.create_subscription(Bool, '/demo/scene_ready', self.on_scene_ready, 1)
        self.create_subscription(String, '/manipulation_status', self.on_motion_status, 10)
        self.create_timer(.2, self.tick)
        self.get_logger().info('Waiting for synchronized RGB, depth, CameraInfo and MoveIt subscriber.')

    def on_scene_ready(self, msg):
        self.scene_ready = msg.data

    def on_motion_status(self, msg):
        self.motion_busy = json.loads(msg.data)['state'] == 'executing'

    def on_controllers(self, future):
        self.controllers_future = None
        try:
            states = {item.name: item.state for item in future.result().controller}
            self.controllers_ready = states.get('panda_arm_controller') == 'active' and states.get('joint_state_broadcaster') == 'active'
        except Exception:
            self.controllers_ready = False

    def store(self, field, msg):
        with self.lock:
            setattr(self, field, msg)
            if field in self.frames:
                stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                self.frames[field][stamp] = msg
                while len(self.frames[field]) > 4:
                    del self.frames[field][min(self.frames[field])]

    def trigger(self, request, response):
        if self.motion_busy or self.pending is not None or (self.worker is not None and self.worker.is_alive()):
            response.success = False
            response.message = 'Detection or manipulation is already running.'
        else:
            self.armed = True
            response.success = True
            response.message = 'Next camera frame will be processed.'
        return response

    def tick(self):
        if self.armed and time.monotonic() - self.last_wait_log > 15:
            self.last_wait_log = time.monotonic()
            self.get_logger().info(
                f'Ready: controllers={self.controllers_ready}, scene={self.scene_ready}, '
                f'rgb={self.rgb is not None}, depth={self.depth is not None}, info={self.info is not None}')
        if not self.controllers_ready:
            if self.controllers.service_is_ready() and self.controllers_future is None:
                self.controllers_future = self.controllers.call_async(ListControllers.Request())
                self.controllers_future.add_done_callback(self.on_controllers)
            return
        if not self.scene_ready:
            return
        if self.pending is not None:
            self.publish_pending()
            return
        if self.motion_busy or not self.armed or (self.worker is not None and self.worker.is_alive()):
            return
        if self.target_pub.get_subscription_count() == 0:
            return
        with self.lock:
            common = self.frames['rgb'].keys() & self.frames['depth'].keys()
            if not common:
                return
            stamp_key = max(common)
            rgb, depth = self.frames['rgb'][stamp_key], self.frames['depth'][stamp_key]
            info = self.info
        if any(msg is None for msg in (rgb, depth, info)):
            return
        stamp = lambda msg: msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if abs(stamp(rgb) - stamp(depth)) > .05:
            return
        if (rgb.width, rgb.height) != (depth.width, depth.height) or (rgb.width, rgb.height) != (info.width, info.height):
            self.get_logger().error('RGB and depth are not aligned; no target published.')
            self.armed = False
            return
        self.armed = False
        self.worker = threading.Thread(target=self.detect, args=(rgb, depth, info), daemon=True)
        self.worker.start()

    def detect(self, rgb, depth, info):
        try:
            image = image_array(rgb)
            depth_image = image_array(depth)
            if depth.encoding not in ('32FC1', '16UC1'):
                raise ValueError(f'Unsupported depth encoding: {depth.encoding}')
            result = self.model.predict(image, device='cpu', imgsz=640,
                                        conf=float(self.get_parameter('confidence').value), verbose=False)[0]
            boxes = [b for b in result.boxes if result.names[int(b.cls.item())] == 'bottle']
            annotated = result.plot()
            cv2.imwrite(str(self.output / 'camera_detected.jpg'), annotated)
            cv2.imwrite(str(self.output / 'camera_rgb.jpg'), image)
            np.save(self.output / 'camera_depth.npy', depth_image)
            msg = image_message(annotated, rgb.header)
            self.image_pub.publish(msg)
            if not boxes:
                raise ValueError('YOLO found no bottle; inspect camera_detected.jpg. No motion requested.')
            (self.output / 'last_detection_failure.json').unlink(missing_ok=True)
            box = max(boxes, key=lambda b: float(b.conf.item()))
            x0, y0, x1, y1 = box.xyxy[0].tolist()
            u, v = (x0+x1)/2, (y0+y1)/2
            z = median_depth(depth_image, u, v)
            xyz = backproject(u, v, z, info.k)
            point = PointStamped()
            point.header = rgb.header
            point.header.frame_id = 'camera_optical_frame'
            point.point.x, point.point.y, point.point.z = xyz
            self.pending = (point, {'pixel': [u, v], 'depth_m': z,
                                  'confidence': float(box.conf.item()), 'camera_xyz_m': xyz,
                                  'camera_k': list(info.k), 'stamp_sec': rgb.header.stamp.sec})
        except Exception as error:
            (self.output / 'last_detection_failure.json').write_text(
                json.dumps({'error': str(error), 'time': time.time()}, indent=2))
            self.get_logger().error(str(error))

    def publish_pending(self):
        point, report = self.pending
        if not self.buffer.can_transform('panda_link0', point.header.frame_id, Time.from_msg(point.header.stamp)):
            return
        try:
            converted = self.buffer.transform(point, 'panda_link0')
            camera = self.buffer.lookup_transform('panda_link0', 'camera_optical_frame', Time())
            # Depth measures the visible surface. A known-radius cylindrical prior
            # estimates the bottle axis; no ground-truth pose enters this node.
            dx = camera.transform.translation.x - converted.point.x
            dy = camera.transform.translation.y - converted.point.y
            distance = (dx*dx + dy*dy)**.5
            radius = float(self.get_parameter('bottle_radius').value)
            if distance > radius:
                converted.point.x -= radius * dx / distance
                converted.point.y -= radius * dy / distance
            self.object_pub.publish(converted)
            pose = downward_target(converted.point, converted.header,
                                   float(self.get_parameter('hand_clearance').value))
            pose.header.stamp = self.get_clock().now().to_msg()
            report['object_xyz_m'] = [float(v) for v in (converted.point.x, converted.point.y, converted.point.z)]
            report['hand_target_xyz_m'] = [float(v) for v in (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)]
            report['radius_prior_m'] = radius
            (self.output / 'detection.json').write_text(json.dumps(report, indent=2))
            self.report_pub.publish(String(data=json.dumps(report)))
            self.target_pub.publish(pose)
            self.pending = None
            self.get_logger().info(f'Bottle detected, target: {report["hand_target_xyz_m"]}')
        except Exception as error:
            self.pending = None
            self.get_logger().error(f'Coordinate conversion failed: {error}')


def main():
    rclpy.init()
    node = CameraVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
