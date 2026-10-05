import numpy as np

from adr_perception.projection_overlay import draw_projection


def test_projection_marks_and_preserves_detector_image():
    image = np.zeros((160, 200, 3), np.uint8)
    image[100, 100] = (0, 0, 255)
    uv = np.array([[50, 60], [140, 60], [140, 140], [50, 140]])
    out = draw_projection(image, [(4, uv)], '1 visible gates', 'GT /adr/odom')
    assert np.array_equal(out[100, 100], image[100, 100])
    assert not image[100, 50].any()
    assert out[100, 50, 0] > 0 and out[100, 50, 1] > 0
    assert not out[100, 50, 2]


def test_missing_gt_has_no_stale_polygon():
    out = draw_projection(np.zeros((160, 200, 3), np.uint8), [],
                          'no GT pose at image time', 'GT /adr/odom')
    assert not out[60:].any()
