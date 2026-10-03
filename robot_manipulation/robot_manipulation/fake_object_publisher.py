import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class FakeObjectPublisher(Node):
    def __init__(self):
        super().__init__('fake_object_publisher')
        self.declare_parameter('x', 0.4)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.3)

        self.publisher = self.create_publisher(
            PoseStamped, '/detected_object_pose', 10
        )
        self.sent = False
        self.timer = self.create_timer(0.5, self.publish_once)
        self.get_logger().info('等待位置訂閱者...')

    def publish_once(self):
        if self.sent or self.publisher.get_subscription_count() == 0:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'panda_link0'
        pose.pose.position.x = float(self.get_parameter('x').value)
        pose.pose.position.y = float(self.get_parameter('y').value)
        pose.pose.position.z = float(self.get_parameter('z').value)
        pose.pose.orientation.x = 1.0

        self.publisher.publish(pose)
        self.sent = True
        self.timer.cancel()
        self.get_logger().info(
            f'已發布一次目標：'
            f'({pose.pose.position.x}, '
            f'{pose.pose.position.y}, '
            f'{pose.pose.position.z})。可按 Ctrl+C 結束。'
        )


def main():
    rclpy.init()
    node = FakeObjectPublisher()
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
