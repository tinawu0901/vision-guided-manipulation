#!/usr/bin/env python3
"""Save one camera frame without requesting robot motion."""

import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vision_guided_simulation.images import image_array


rclpy.init()
node = Node('camera_snapshot')
received = []
subscription = node.create_subscription(Image, '/camera/image', received.append, qos_profile_sensor_data)
deadline = time.monotonic() + 90
while not received and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.5)
if received:
    output = Path.home() / 'vision_guided_ws/work/gazebo_results/camera_snapshot.jpg'
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image_array(received[-1]))
    print(output)
else:
    print('No camera image received within 90 seconds.')
node.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if received else 1)
