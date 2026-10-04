import math
import numpy as np
import pytest

from vision_guided_simulation.geometry import backproject, median_depth, quaternion_error
from vision_guided_simulation.geometry import downward_target


def test_hand_target_points_down_without_mutating_measurement():
    from geometry_msgs.msg import Point
    from std_msgs.msg import Header
    point = Point(x=.45, y=.08, z=.15)
    target = downward_target(point, Header(frame_id='panda_link0'), .18)
    assert target.pose.position.z == pytest.approx(.33)
    assert point.z == .15
    q = target.pose.orientation
    assert q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w == pytest.approx(1.)
    # Rotating the tool's local Z by this quaternion must point down.
    assert 1 - 2*(q.x*q.x + q.y*q.y) == pytest.approx(-1.)


def test_camera_principal_point_and_signs():
    k = [500., 0., 320., 0., 500., 240., 0., 0., 1.]
    assert backproject(320, 240, .5, k) == (0., 0., .5)
    assert backproject(220, 340, .5, k) == (-.1, .1, .5)


def test_invalid_camera_parameters():
    for depth in (0., -.1, float('nan')):
        with pytest.raises(ValueError):
            backproject(320, 240, depth, [500., 0., 320., 0., 500., 240., 0., 0., 1.])


def test_depth_rejects_invalid_pixels():
    image = np.array([[float('nan'), .7, .7], [float('inf'), .7, .7], [0., .7, .7]])
    assert median_depth(image, 1, 1) == .7
    with pytest.raises(ValueError):
        median_depth(np.zeros((3, 3)), 1, 1)


def test_quaternion_sign_equivalence_and_large_error():
    assert quaternion_error([1, 0, 0, 0], [-1, 0, 0, 0]) == 0
    assert quaternion_error([1, 0, 0, 0], [0, 0, 0, 1]) == math.pi
