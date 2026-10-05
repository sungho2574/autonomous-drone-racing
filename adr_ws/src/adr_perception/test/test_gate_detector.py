"""ROS 없이 검출 로직만 검사 (합성 이미지). rclpy/cv_bridge/adr_msgs 는 스텁으로 대체."""
import sys
import types

import cv2
import numpy as np
import pytest


def _stub_ros():
    for name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'cv_bridge', 'adr_msgs', 'adr_msgs.msg',
                 'sensor_msgs', 'sensor_msgs.msg'):
        sys.modules.setdefault(name, types.ModuleType(name))

    class Dummy:
        def __init__(self, *a, **k):
            pass
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].QoSProfile = Dummy
    sys.modules['rclpy.qos'].ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=0)
    sys.modules['rclpy.qos'].HistoryPolicy = types.SimpleNamespace(KEEP_LAST=0)
    sys.modules['cv_bridge'].CvBridge = Dummy
    sys.modules['adr_msgs.msg'].GateDetection = Dummy
    sys.modules['adr_msgs.msg'].GateDetectionArray = Dummy
    sys.modules['sensor_msgs.msg'].Image = Dummy


_stub_ros()
from adr_perception.gate_detector import GateDetector, is_parallelogram  # noqa: E402

BG = (140, 150, 140)
ORANGE = (0, 115, 255)   # BGR of gz (1.0, 0.45, 0.0)


def _detector():
    d = GateDetector.__new__(GateDetector)
    d.lower = np.array([3, 100, 60], np.uint8)
    d.upper = np.array([25, 255, 255], np.uint8)
    d.min_area, d.min_hole_area = 150.0, 30.0
    d.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    d.max_gates = 4
    d.border = 3
    d.min_corner_area = 400.0
    d.max_parallel_angle_deg = 8.0
    d.max_opposite_length_error = 0.20
    return d


def _gate(img, cx, cy, outer, inner):
    cv2.rectangle(img, (cx - outer // 2, cy - outer // 2), (cx + outer // 2, cy + outer // 2), ORANGE, -1)
    cv2.rectangle(img, (cx - inner // 2, cy - inner // 2), (cx + inner // 2, cy + inner // 2), BG, -1)


def test_two_gates_center_and_order():
    img = np.full((480, 640, 3), BG, np.uint8)
    _gate(img, 320, 240, 210, 150)
    _gate(img, 520, 200, 70, 50)
    dets, _ = _detector().detect(img)
    assert len(dets) == 2
    d = dets[0]                                            # 가장 큰 게이트가 0번
    assert abs(d.center[0] - 320) < 1 and abs(d.center[1] - 240) < 1 and d.has_hole
    assert d.bbox == (215, 135, 211, 211)
    assert d.conf > 0.8
    assert abs(dets[1].center[0] - 520) < 1


def test_no_gate():
    img = np.full((480, 640, 3), BG, np.uint8)
    dets, _ = _detector().detect(img)
    assert dets == []


@pytest.mark.parametrize('centers', [
    [(220, 240), (340, 240)],
    [(220, 180), (220, 300)],
    [(220, 300), (340, 300), (220, 180)],
])
def test_touching_frames_have_separate_openings(centers):
    img = np.full((480, 640, 3), BG, np.uint8)
    for cx, cy in centers:
        _gate(img, cx, cy, 120, 80)
    dets, mask = _detector().detect(img)
    # Prove this exercises one connected orange cluster, not isolated gates.
    assert cv2.connectedComponents(mask)[0] == 2
    assert len(dets) == len(centers)
    for cx, cy in centers:
        matches = [d for d in dets if np.linalg.norm(np.array(d.center) - [cx, cy]) < 1]
        assert len(matches) == 1
        d = matches[0]
        assert d.has_hole and len(d.corners) == 4
        expected = np.array([[cx-40, cy-40], [cx+40, cy-40],
                             [cx+40, cy+40], [cx-40, cy+40]])
        assert np.abs(d.corners - expected).max() <= 1.5
        assert d.bbox[2] < 90 and d.bbox[3] < 90


def test_cluster_rejects_bad_hole_without_losing_good_hole():
    img = np.full((480, 640, 3), BG, np.uint8)
    cv2.rectangle(img, (100, 100), (400, 300), ORANGE, -1)
    cv2.rectangle(img, (130, 130), (230, 270), BG, -1)
    cv2.fillPoly(img, [np.array([[270, 130], [320, 130], [370, 270], [250, 270]])], BG)
    cv2.rectangle(img, (110, 110), (112, 112), BG, -1)  # tiny spurious hole
    dets, _ = _detector().detect(img)
    assert len(dets) == 2
    assert sum(len(d.corners) == 4 for d in dets) == 1


def test_opening_area_order_and_limit():
    img = np.full((480, 640, 3), BG, np.uint8)
    cv2.rectangle(img, (50, 100), (550, 300), ORANGE, -1)
    for x, size in [(100, 40), (230, 80), (400, 120)]:
        cv2.rectangle(img, (x, 140), (x+size, 140+size), BG, -1)
    detector = _detector()
    detector.max_gates = 2
    dets, _ = detector.detect(img)
    assert len(dets) == 2
    assert dets[0].area > dets[1].area
    assert dets[0].center[0] == pytest.approx(460)
    assert dets[1].center[0] == pytest.approx(270)


def test_partial_gate_without_hole():
    """프레임 한쪽만 보이면 has_hole=False, 중심은 bbox 중심, 꼭짓점 없음."""
    img = np.full((480, 640, 3), BG, np.uint8)
    cv2.rectangle(img, (100, 100), (130, 400), ORANGE, -1)
    dets, _ = _detector().detect(img)
    assert len(dets) == 1 and not dets[0].has_hole
    assert abs(dets[0].center[0] - 115) < 1
    assert len(dets[0].corners) == 0


def test_corners_axis_aligned():
    """정면 게이트 → 개구부 4 꼭짓점을 TL→TR→BR→BL 순서로, 1 px 안쪽으로 찾아야 한다."""
    img = np.full((480, 640, 3), BG, np.uint8)
    _gate(img, 320, 240, 210, 150)
    d = _detector().detect(img)[0][0]
    assert len(d.corners) == 4
    expect = np.array([[245, 165], [395, 165], [395, 315], [245, 315]], float)   # TL,TR,BR,BL
    assert np.abs(d.corners - expect).max() <= 1.5, f'{d.corners}'


def test_corners_reject_perspective_trapezoid():
    """사다리꼴은 2D 검출을 유지하지만 PnP용 꼭짓점은 내보내지 않는다."""
    img = np.full((480, 640, 3), BG, np.uint8)
    outer = np.array([[180, 120], [460, 150], [440, 400], [200, 360]], np.int32)
    inner = np.array([[220, 160], [420, 185], [405, 360], [235, 325]], np.int32)
    cv2.fillPoly(img, [outer], ORANGE)
    cv2.fillPoly(img, [inner], BG)
    d = _detector().detect(img)[0][0]
    assert d.has_hole
    assert len(d.corners) == 0


def test_corners_rejected_at_image_border():
    """게이트를 통과하는 중이면 개구부가 화면 밖으로 잘린다 → 꼭짓점을 내보내면 안 된다."""
    img = np.full((480, 640, 3), BG, np.uint8)
    _gate(img, 320, 240, 900, 800)         # 개구부가 화면보다 큼
    dets, _ = _detector().detect(img)
    assert all(len(d.corners) == 0 for d in dets), '잘린 개구부에서 꼭짓점이 나왔다'


def test_small_hole_no_corners():
    """너무 작은 개구부는 꼭짓점 정확도가 안 나오므로 PnP 에 쓰지 않는다."""
    img = np.full((480, 640, 3), BG, np.uint8)
    _gate(img, 320, 240, 40, 15)           # 개구부 15x15 = 225 px^2 < min_corner_area
    dets, _ = _detector().detect(img)
    assert len(dets) == 1 and dets[0].has_hole and len(dets[0].corners) == 0


@pytest.mark.parametrize('angle', [0, 25, 90, 145])
@pytest.mark.parametrize('scale', [0.5, 1.0, 3.0])
def test_parallelogram_rotation_and_scale(angle, scale):
    q = np.array([[0, 0], [120, 0], [155, 90], [35, 90]], float)
    a = np.radians(angle)
    R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    assert is_parallelogram(scale * q @ R.T + [200, 150])


@pytest.mark.parametrize('q', [
    [[0, 0], [100, 0], [140, 100], [-40, 100]],  # only horizontal pair parallel
    [[0, 0], [100, -40], [100, 140], [0, 100]],  # only vertical pair parallel
    [[0, 0], [100, 0], [130, 1000], [-30, 1000]],  # angles pass, lengths fail
    [[0, 0], [0, 0], [100, 100], [0, 100]],
    [[0, 0], [100, 100], [100, 0], [0, 100]],
    [[0, 0], [100, 0], [20, 20], [0, 100]],
    [[0, 0], [100, 0], [100, float('nan')], [0, 100]],
])
def test_reject_invalid_quads(q):
    assert not is_parallelogram(q)


def test_parallel_tolerance_is_configurable():
    q = np.array([[0, 0], [100, 0], [110, 100], [-10, 100]], float)
    assert not is_parallelogram(q)
    assert is_parallelogram(q, max_angle_deg=12, max_length_error=0.20)
    assert not is_parallelogram(q, max_angle_deg=12, max_length_error=0.10)


def test_noisy_parallelogram_passes():
    q = np.array([[0, 1], [121, 0], [153, 91], [35, 89]], float)
    assert is_parallelogram(q)


def test_sheared_gate_corners_pass_end_to_end():
    img = np.full((480, 640, 3), BG, np.uint8)
    outer = np.array([[160, 100], [400, 100], [460, 370], [220, 370]], np.int32)
    inner = np.array([[205, 145], [365, 145], [405, 325], [245, 325]], np.int32)
    cv2.fillPoly(img, [outer], ORANGE)
    cv2.fillPoly(img, [inner], BG)
    d = _detector().detect(img)[0][0]
    assert len(d.corners) == 4
    assert np.abs(d.corners - inner).max() <= 3
