import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from vision_guided_simulation.images import image_array, image_message


def test_rgb_with_row_padding():
    msg = Image(height=2, width=1, encoding='rgb8', step=4,
                data=bytes([10, 20, 30, 0, 40, 50, 60, 0]))
    assert image_array(msg).tolist() == [[[30, 20, 10]], [[60, 50, 40]]]


def test_big_endian_millimetre_depth():
    msg = Image(height=1, width=2, encoding='16UC1', step=4, is_bigendian=1,
                data=np.array([500, 1200], dtype='>u2').tobytes())
    assert np.allclose(image_array(msg), [[.5, 1.2]])


def test_bgr_roundtrip():
    image = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
    assert np.array_equal(image_array(image_message(image, Header())), image)
