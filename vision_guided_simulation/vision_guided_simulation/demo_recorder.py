"""Save real Gazebo overview frames with the current manipulation state."""

import json
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vision_guided_simulation.images import image_array


class DemoRecorder(Node):
    def __init__(self):
        super().__init__('demo_recorder')
        self.output = Path.home() / 'vision_guided_ws/work/gazebo_results/frames'
        self.output.mkdir(parents=True, exist_ok=True)
        self.state = {'state': 'waiting'}
        self.records = []
        self.finished = False
        self.create_subscription(String, '/manipulation_status', self.on_status, 10)
        self.create_subscription(Image, '/overview/image', self.on_image,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

    def on_status(self, msg):
        self.state = json.loads(msg.data)
        if self.state['state'] == 'executing' and self.finished:
            self.records = []
            self.finished = False

    def on_image(self, msg):
        if self.finished:
            return
        # Keep the latest starting view, then all frames during the sequence.
        if self.state['state'] == 'waiting' and self.records:
            self.records = []
        filename = f'{len(self.records):04d}.jpg'
        image = image_array(msg)
        cv2.imwrite(str(self.output / filename), image)
        self.records.append({'file': filename, 'simulation_time_s':
                             msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             **self.state})
        (self.output / 'frames.json').write_text(json.dumps(self.records, indent=2))
        if self.state['state'] in ('complete', 'failed'):
            self.finished = True
            self.get_logger().info(f'Saved {len(self.records)} overview frames.')


def main():
    rclpy.init()
    node = DemoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
