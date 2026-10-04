from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO


class VisionDetector(Node):
    def __init__(self):
        super().__init__('vision_detector')
        default_image = str(
            Path.home() /
            'vision_guided_ws/src/vision_guided_manipulation/assets/Plastic_bottle.jpg'
        )
        self.declare_parameter('image_path', default_image)
        self.declare_parameter('model_path', str(
            Path.home() / 'vision_guided_ws/work/vision_smoke_test/yolov8n.pt'
        ))
        self.publisher = self.create_publisher(
            PointStamped, '/detected_object_pixel', 10
        )
        image = self.get_parameter('image_path').value
        model = YOLO(self.get_parameter('model_path').value)
        result = model.predict(source=image, device='cpu', conf=0.4)[0]

        bottles = [
            box for box in result.boxes
            if result.names[int(box.cls.item())] == 'bottle'
        ]
        if not bottles:
            raise RuntimeError('未找到瓶子，不發布目標。')

        box = max(bottles, key=lambda item: float(item.conf.item()))
        xmin, ymin, xmax, ymax = box.xyxy[0].tolist()
        self.u = (xmin + xmax) / 2
        self.v = (ymin + ymax) / 2
        result.save(filename=str(Path(image).with_name('bottle_detected.jpg')))

        self.timer = self.create_timer(0.5, self.publish_once)
        self.get_logger().info(
            f'瓶子中心 ({self.u:.1f}, {self.v:.1f}) pixels；等待訂閱者。'
        )

    def publish_once(self):
        if self.publisher.get_subscription_count() == 0:
            return
        msg = PointStamped()
        msg.header.frame_id = 'image_pixels'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = self.u
        msg.point.y = self.v
        self.publisher.publish(msg)
        self.timer.cancel()
        self.get_logger().info('已發布一次像素中心，可按 Ctrl+C 結束。')


def main():
    rclpy.init()
    node = None
    try:
        node = VisionDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
