from pathlib import Path
import math
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class ObjectLocalization(Node):
    def __init__(self):
        super().__init__('object_localization')
        self.declare_parameter('image_path', str(
            Path.home() /
            'vision_guided_ws/src/vision_guided_manipulation/assets/Plastic_bottle.jpg'
        ))
        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)
        self.declare_parameter('depth', 0.5)

        image = cv2.imread(self.get_parameter('image_path').value)
        if image is None:
            raise RuntimeError('無法讀取圖片。')
        height, width = image.shape[:2]
        self.cx = width / 2
        self.cy = height / 2

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PoseStamped, '/detected_object_pose', 10
        )
        self.subscription = self.create_subscription(
            PointStamped, '/detected_object_pixel', self.on_pixel, 10
        )
        self.pending = None
        self.timer = self.create_timer(0.2, self.try_transform)
        self.get_logger().info(
            '等待像素中心。使用示範內參與固定深度，不是真實量測。'
        )

    def on_pixel(self, msg):
        if msg.header.frame_id != 'image_pixels':
            self.get_logger().error('像素資料框架不符，忽略。')
            return

        fx = float(self.get_parameter('fx').value)
        fy = float(self.get_parameter('fy').value)
        depth = float(self.get_parameter('depth').value)
        values = [fx, fy, depth, msg.point.x, msg.point.y]
        if not all(math.isfinite(v) for v in values) or min(fx, fy, depth) <= 0:
            self.get_logger().error('內參、深度或像素資料無效。')
            return

        point = PointStamped()
        point.header.frame_id = 'camera_optical_frame'
        # 此 MVP 使用静態 TF 與儲存圖片，查詢最新轉換。
        point.point.x = (msg.point.x - self.cx) * depth / fx
        point.point.y = (msg.point.y - self.cy) * depth / fy
        point.point.z = depth
        self.pending = point
        self.get_logger().info(
            f'相機位置：({point.point.x:.4f}, '
            f'{point.point.y:.4f}, {point.point.z:.4f}) m'
        )

    def try_transform(self):
        if self.pending is None:
            return
        if not self.buffer.can_transform(
            'panda_link0', 'camera_optical_frame', Time()
        ):
            return

        converted = self.buffer.transform(self.pending, 'panda_link0')
        pose = PoseStamped()
        pose.header.frame_id = 'panda_link0'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = converted.point

        # 指定手掌朝向；不是由圖片估計的瓶子朝向。
        pose.pose.orientation.x = 1.0
        self.publisher.publish(pose)
        self.pending = None
        self.get_logger().info(
            f'已發布底座目標：({pose.pose.position.x:.4f}, '
            f'{pose.pose.position.y:.4f}, '
            f'{pose.pose.position.z:.4f}) m'
        )


def main():
    rclpy.init()
    node = None
    try:
        node = ObjectLocalization()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
