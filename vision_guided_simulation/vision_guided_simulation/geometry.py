"""Small, testable camera and pose calculations."""

import math
import numpy as np


def downward_target(point, header, clearance):
    from geometry_msgs.msg import PoseStamped, Quaternion
    from copy import deepcopy
    target = PoseStamped()
    target.header = deepcopy(header)
    target.pose.position = deepcopy(point)
    target.pose.position.z += clearance
    target.pose.orientation = Quaternion(x=1., y=0., z=0., w=0.)
    return target


def backproject(u, v, depth, k):
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    values = (u, v, depth, fx, fy, cx, cy)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('Non-finite camera input')
    if min(depth, fx, fy) <= 0:
        raise ValueError('Depth and focal lengths must be positive')
    return ((u - cx) * depth / fx, (v - cy) * depth / fy, depth)


def median_depth(image, u, v, radius=3):
    x, y = round(u), round(v)
    if not (0 <= x < image.shape[1] and 0 <= y < image.shape[0]):
        raise ValueError('Detection outside depth image')
    patch = image[max(0, y-radius):y+radius+1, max(0, x-radius):x+radius+1]
    valid = patch[np.isfinite(patch) & (patch > .05) & (patch < 3)]
    if not valid.size:
        raise ValueError('No valid depth near detection')
    return float(np.median(valid))


def quaternion_error(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if min(np.linalg.norm(a), np.linalg.norm(b)) == 0:
        raise ValueError('Zero quaternion')
    dot = abs(float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))))
    return 2 * math.acos(min(1., dot))
