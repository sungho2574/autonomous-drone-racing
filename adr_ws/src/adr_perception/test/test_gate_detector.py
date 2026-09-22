"""ROS 없이 검출 로직만 검사 (합성 이미지). rclpy/cv_bridge/adr_msgs 는 스텁으로 대체."""
import sys
import types

import cv2
import numpy as np


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
from adr_perception.gate_detector import GateDetector  # noqa: E402

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


def test_corners_perspective_quad():
    """원근으로 기울어진 사다리꼴 개구부도 꼭짓점을 잡고 순서가 유지돼야 한다."""
    img = np.full((480, 640, 3), BG, np.uint8)
    outer = np.array([[180, 120], [460, 150], [440, 400], [200, 360]], np.int32)
    inner = np.array([[220, 160], [420, 185], [405, 360], [235, 325]], np.int32)
    cv2.fillPoly(img, [outer], ORANGE)
    cv2.fillPoly(img, [inner], BG)
    d = _detector().detect(img)[0][0]
    assert len(d.corners) == 4
    assert np.abs(d.corners - inner.astype(float)).max() <= 3.0, f'{d.corners}'


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
