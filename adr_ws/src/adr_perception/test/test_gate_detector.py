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
    (x, y, w, h), (u, v), area, hole, conf = dets[0]       # 가장 큰 게이트가 0번
    assert abs(u - 320) < 1 and abs(v - 240) < 1 and hole
    assert (x, y, w, h) == (215, 135, 211, 211)
    assert conf > 0.8
    assert abs(dets[1][1][0] - 520) < 1


def test_no_gate():
    img = np.full((480, 640, 3), BG, np.uint8)
    dets, _ = _detector().detect(img)
    assert dets == []


def test_partial_gate_without_hole():
    """프레임 한쪽만 보이면 has_hole=False, 중심은 bbox 중심."""
    img = np.full((480, 640, 3), BG, np.uint8)
    cv2.rectangle(img, (100, 100), (130, 400), ORANGE, -1)
    dets, _ = _detector().detect(img)
    assert len(dets) == 1 and not dets[0][3]
    assert abs(dets[0][1][0] - 115) < 1
