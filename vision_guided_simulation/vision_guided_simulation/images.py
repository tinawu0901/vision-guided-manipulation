"""ROS Image conversion with explicit row stride and byte order."""

import numpy as np
from sensor_msgs.msg import Image


def image_array(msg):
    formats = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'rgba8': ('u1', 4),
               'bgra8': ('u1', 4), '32FC1': ('f4', 1), '16UC1': ('u2', 1)}
    if msg.encoding not in formats:
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')
    dtype, channels = formats[msg.encoding]
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + dtype)
    shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
    strides = (msg.step, dtype.itemsize*channels, dtype.itemsize) if channels > 1 else (msg.step, dtype.itemsize)
    if len(msg.data) < msg.step*msg.height:
        raise ValueError('Image buffer is truncated')
    array = np.ndarray(shape=shape, dtype=dtype, buffer=bytes(msg.data), strides=strides).copy()
    if msg.encoding == 'rgb8':
        return array[..., ::-1].copy()
    if msg.encoding == 'rgba8':
        return array[..., [2, 1, 0]].copy()
    if msg.encoding == 'bgra8':
        return array[..., :3].copy()
    if msg.encoding == '16UC1':
        return array.astype(np.float32)/1000
    return array


def image_message(bgr, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width*3
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg
