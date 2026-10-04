"""Independent scene reference, collision geometry and endpoint verification."""

import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from vision_guided_simulation.geometry import quaternion_error
from vision_guided_simulation.scene import load_scene


class SceneMonitor(Node):
    def __init__(self):
        super().__init__('scene_monitor')
        self.declare_parameter('scene_path', '')
        self.declare_parameter('output_dir', str(Path.home() / 'vision_guided_ws/work/gazebo_results'))
        self.scene = load_scene(self.get_parameter('scene_path').value)
        self.output = Path(self.get_parameter('output_dir').value)
        self.output.mkdir(parents=True, exist_ok=True)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.markers = self.create_publisher(MarkerArray, '/demo/markers', 10)
        self.ready_pub = self.create_publisher(Bool, '/demo/scene_ready', 1)
        self.client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.scene_applied = False
        self.scene_future = None
        self.target = None
        self.report = None
        self.completed_at = None
        self.create_subscription(PoseStamped, '/detected_object_pose', self.on_target, 10)
        self.create_subscription(String, '/vision/detection_report', self.on_detection, 10)
        self.create_subscription(String, '/manipulation_status', self.on_status, 10)
        self.create_timer(.5, self.tick)

    def on_target(self, msg):
        self.target = msg
        self.completed_at = None
        (self.output / 'verification.json').unlink(missing_ok=True)
        (self.output / 'motion_failure.json').unlink(missing_ok=True)

    def on_detection(self, msg):
        self.report = json.loads(msg.data)
        b = self.scene['bottle']
        p = self.report['object_xyz_m']
        self.report['reference_xy_m'] = [b['x'], b['y']]
        self.report['localization_xy_error_m'] = math.hypot(p[0]-b['x'], p[1]-b['y'])
        self.get_logger().info(f'Bottle-axis XY error: {self.report["localization_xy_error_m"]*1000:.1f} mm')

    def on_status(self, msg):
        status = json.loads(msg.data)
        if status['state'] == 'complete':
            self.completed_at = time.monotonic()
        elif status['state'] == 'failed':
            (self.output / 'motion_failure.json').write_text(json.dumps(status, indent=2))

    def apply_table(self):
        if self.scene_applied or not self.client.service_is_ready() or not self.get_scene.service_is_ready() or self.scene_future:
            return
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        self.scene_future = self.get_scene.call_async(request)
        self.scene_future.add_done_callback(self.send_table)

    def send_table(self, future):
        t = self.scene['table']
        obj = CollisionObject()
        obj.header.frame_id = 'panda_link0'
        obj.id = 'worktable'
        obj.operation = CollisionObject.ADD
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[t['size_x'], t['size_y'], t['size_z']])
        pose = Pose()
        pose.position.x, pose.position.z = t['center_x'], t['center_z']
        pose.orientation.w = 1.
        obj.primitives = [box]
        obj.primitive_poses = [pose]
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obj]
        # The anchored base rests on the worktable.
        acm = future.result().scene.allowed_collision_matrix
        from moveit_msgs.msg import AllowedCollisionEntry
        if 'worktable' not in acm.entry_names:
            for row in acm.entry_values:
                row.enabled.append(False)
            acm.entry_names.append('worktable')
            acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        table = acm.entry_names.index('worktable')
        base = acm.entry_names.index('panda_link0')
        acm.entry_values[table].enabled[base] = True
        acm.entry_values[base].enabled[table] = True
        request.scene.allowed_collision_matrix = acm
        self.scene_future = self.client.call_async(request)
        self.scene_future.add_done_callback(self.on_scene)

    def on_scene(self, future):
        self.scene_applied = future.result().success
        self.scene_future = None

    def tick(self):
        self.apply_table()
        self.ready_pub.publish(Bool(data=self.scene_applied))
        b = self.scene['bottle']
        reference = Marker()
        reference.header.frame_id = 'panda_link0'
        reference.ns, reference.id = 'reference', 0
        reference.type = Marker.CYLINDER
        reference.pose.position.x, reference.pose.position.y = b['x'], b['y']
        reference.pose.position.z = b['bottom_z'] + b['height']/2
        reference.pose.orientation.w = 1.
        reference.scale.x = reference.scale.y = 2*b['radius']
        reference.scale.z = b['height']
        reference.color.r, reference.color.g, reference.color.a = .1, .7, .6
        markers = [reference]
        if self.target:
            for i, (name, dz, color) in enumerate([
                ('reach', 0., (1., .3, .1)), ('approach', .1, (1., .8, .1)),
                ('lift', .15, (.2, .5, 1.))]):
                marker = Marker()
                marker.header = self.target.header
                marker.ns, marker.id, marker.type = name, i+1, Marker.SPHERE
                from copy import deepcopy
                marker.pose = deepcopy(self.target.pose)
                marker.pose.position.z += dz
                marker.scale.x = marker.scale.y = marker.scale.z = .025
                marker.color.r, marker.color.g, marker.color.b = color
                marker.color.a = 1.
                markers.append(marker)
        self.markers.publish(MarkerArray(markers=markers))
        if self.completed_at is not None and time.monotonic()-self.completed_at > 1.:
            self.verify_endpoint()

    def verify_endpoint(self):
        if self.target is None or self.report is None:
            return
        try:
            tf = self.buffer.lookup_transform('panda_link0', 'panda_hand', Time())
        except Exception:
            return
        actual = tf.transform.translation
        goal = self.target.pose.position
        expected = [goal.x, goal.y, goal.z+.15]
        position = [actual.x, actual.y, actual.z]
        error = math.dist(position, expected)
        q = tf.transform.rotation
        angle = quaternion_error([q.x, q.y, q.z, q.w], [1., 0., 0., 0.])
        self.report.update({'final_expected_xyz_m': expected, 'final_actual_xyz_m': position,
                            'endpoint_error_m': error, 'orientation_error_rad': angle,
                            'passed': error <= .015 and angle <= .18 and self.report['localization_xy_error_m'] <= .05})
        (self.output / 'verification.json').write_text(json.dumps(self.report, indent=2))
        self.get_logger().info(f'Endpoint error {error*1000:.1f} mm; orientation {math.degrees(angle):.1f} deg; passed={self.report["passed"]}')
        self.completed_at = None


def main():
    rclpy.init()
    node = SceneMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
