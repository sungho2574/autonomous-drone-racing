"""색상(주황) 기반 게이트 검출기 — 개구부 4 꼭짓점까지 뽑는다.

/adr/camera/image_raw → HSV 임계 → 모폴로지 → 컨투어(RETR_CCOMP: 바깥 프레임 + 안쪽 구멍)
  → 게이트마다 개구부의 사각형 꼭짓점 4개 / 중심 / 면적 → /adr/gate_detections (2D 픽셀)

꼭짓점은 구멍 컨투어를 convexHull → approxPolyDP 로 4점이 될 때까지 epsilon 을 키우며 찾는다.
이미지 기준 **시계방향 TL→TR→BR→BL** 로 정렬해 내보내므로 gate_pnp 가 그대로 solvePnP 에 쓴다.
이미지 경계에 닿은 개구부(게이트를 통과하는 중)는 잘린 꼭짓점이라 has_corners=False 로 버린다.

디버그 오버레이는 /adr/perception/debug_image 로 내되, **구독자가 있을 때만** 그린다:
    ros2 run rqt_image_view rqt_image_view /adr/perception/debug_image

step 3 에서는 이 자리를 Gatenet(코너 검출)으로 바꾼다. 출력 계약(코너 4점)은 그대로 두면 된다.
HSV 범위는 sim 게이트 색(1.0, 0.45, 0.0 → H≈13, S·V 높음) 기준. 실기체 조명에 맞춰 파라미터로 조정.
"""
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from adr_msgs.msg import GateDetection, GateDetectionArray
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from adr_perception.pnp import order_quad

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

# approxPolyDP 의 epsilon 을 둘레의 이 비율만큼 키워 가며 4각형이 나오는 지점을 찾는다
_EPS_FRACTIONS = (0.01, 0.015, 0.02, 0.03, 0.04, 0.06, 0.08)


def bgr8_to_imgmsg(bgr: np.ndarray, header) -> Image:
    """CvBridge.cv2_to_imgmsg(encoding='bgr8') 대체.

    cv_bridge 의 인코딩 경로는 파이썬 cv2 의 CV_8UC3 상수와 OpenCV 4 로 빌드된 C++ 확장의 값을
    같이 쓴다. OpenCV 5 를 pip 로 깔면 상수가 16→64 로 바뀌어 KeyError 로 죽는다(읽기 경로는 멀쩡).
    bgr8 은 그냥 행 우선 바이트라 직접 채우는 편이 버전에 안 휘둘린다.
    """
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[0], bgr.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = bgr.shape[1] * 3
    msg.data = np.ascontiguousarray(bgr, dtype=np.uint8).tobytes()
    return msg


@dataclass
class Detection:
    """검출된 게이트 1개 (이미지 좌표)."""
    bbox: tuple              # (x, y, w, h) 주황 프레임 바운딩박스. 디버그 이미지엔 안 그리지만
                             # 메시지로는 계속 내보낸다 (다른 소비자/디버깅용)
    center: tuple            # (u, v) 개구부 중심. 구멍이 없으면 bbox 중심
    area: float              # 주황 프레임 컨투어 면적 [px^2]
    has_hole: bool
    conf: float
    corners: np.ndarray      # (4,2) float32, TL→TR→BR→BL. 신뢰 못 하면 빈 배열


def quad_from_contour(contour: np.ndarray) -> np.ndarray:
    """컨투어 → 사각형 꼭짓점 (4,2). 못 찾으면 빈 배열."""
    hull = cv2.convexHull(contour)
    peri = cv2.arcLength(hull, True)
    if peri <= 0:
        return np.empty((0, 2), np.float32)
    for f in _EPS_FRACTIONS:
        ap = cv2.approxPolyDP(hull, f * peri, True)
        if len(ap) == 4:
            # cv2 의 contour 함수는 float32/int32 만 받는다 (order_quad 는 float64 를 준다)
            quad = order_quad(ap).astype(np.float32)
            # 볼록하고 면적이 남아 있어야 한다 (자기교차·퇴화 방지)
            if cv2.isContourConvex(quad) and cv2.contourArea(quad) > 0:
                return quad
    return np.empty((0, 2), np.float32)


class GateDetector(Node):
    def __init__(self):
        super().__init__('gate_detector')
        self.declare_parameter('image_topic', '/adr/camera/image_raw')
        self.declare_parameter('hsv_lower', [3, 100, 60])      # OpenCV H 0~179
        self.declare_parameter('hsv_upper', [25, 255, 255])
        self.declare_parameter('min_area', 150.0)               # 프레임 픽셀 면적 하한 [px^2]
        self.declare_parameter('min_hole_area', 30.0)           # 개구부 하한 [px^2]
        self.declare_parameter('morph_kernel', 3)
        self.declare_parameter('max_gates', 4)
        self.declare_parameter('border_margin', 3)              # 꼭짓점이 이 안쪽으로 들어오면 잘린 것으로 본다 [px]
        self.declare_parameter('min_corner_area', 400.0)        # PnP 에 쓸 개구부 최소 면적 [px^2]
        self.declare_parameter('debug_topic', '/adr/perception/debug_image')

        self.lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        self.min_area = float(self.get_parameter('min_area').value)
        self.min_hole_area = float(self.get_parameter('min_hole_area').value)
        k = int(self.get_parameter('morph_kernel').value)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)) if k > 1 else None
        self.max_gates = int(self.get_parameter('max_gates').value)
        self.border = int(self.get_parameter('border_margin').value)
        self.min_corner_area = float(self.get_parameter('min_corner_area').value)

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(GateDetectionArray, '/adr/gate_detections', 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_topic').value, SENSOR_QOS)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image, SENSOR_QOS)
        self.n = 0
        self.get_logger().info(f'gate_detector: HSV {self.lower.tolist()}~{self.upper.tolist()}, '
                               f'개구부 4 꼭짓점 검출, debug → '
                               f'{self.get_parameter("debug_topic").value} (구독자 있을 때만)')

    # ------------------------------------------------------------------
    def detect(self, bgr: np.ndarray):
        """BGR 이미지 → (Detection 리스트 면적 내림차순, 마스크)."""
        h_img, w_img = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        if self.kernel is not None:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        if hierarchy is None:
            return out, mask
        hierarchy = hierarchy[0]          # [next, prev, first_child, parent]
        for i, c in enumerate(contours):
            if hierarchy[i][3] != -1:     # 바깥(프레임) 컨투어만
                continue
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            # 가장 큰 구멍(개구부) 찾기
            hole, hole_area = None, 0.0
            j = hierarchy[i][2]
            while j != -1:
                a = cv2.contourArea(contours[j])
                if a > hole_area:
                    hole, hole_area = contours[j], a
                j = hierarchy[j][0]

            corners = np.empty((0, 2), np.float32)
            if hole is not None and hole_area >= self.min_hole_area:
                m = cv2.moments(hole)
                cu, cv_ = m['m10'] / m['m00'], m['m01'] / m['m00']
                has_hole = True
                if hole_area >= self.min_corner_area:
                    q = quad_from_contour(hole)
                    # 이미지 경계에 닿으면 꼭짓점이 잘린 것 — PnP 에 쓰면 포즈가 크게 틀어진다
                    if len(q) == 4 and (q[:, 0] > self.border).all() and (q[:, 1] > self.border).all() \
                            and (q[:, 0] < w_img - 1 - self.border).all() \
                            and (q[:, 1] < h_img - 1 - self.border).all():
                        corners = q
            else:
                cu, cv_ = x + w / 2, y + h / 2
                has_hole = False
            # 신뢰도: 프레임(링) 면적 비율이 사각 링에 가깝고 종횡비가 1 에 가까울수록 높게
            ring_ratio = (area - hole_area) / max(w * h, 1.0)
            aspect = min(w, h) / max(w, h)
            conf = float(np.clip(0.5 * aspect + 0.5 * min(ring_ratio / 0.5, 1.0), 0.0, 1.0))
            out.append(Detection((x, y, w, h), (cu, cv_), area, has_hole, conf, corners))
        out.sort(key=lambda d: d.area, reverse=True)
        return out[:self.max_gates], mask

    def _on_image(self, msg: Image):
        self.n += 1
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dets, _ = self.detect(bgr)

        arr = GateDetectionArray()
        arr.header = msg.header
        arr.image_width, arr.image_height = msg.width, msg.height
        for k, d in enumerate(dets):
            m = GateDetection()
            m.id = k
            m.u, m.v = float(d.center[0]), float(d.center[1])
            m.x, m.y, m.w, m.h = (float(v) for v in d.bbox)
            m.area, m.has_hole, m.confidence = float(d.area), bool(d.has_hole), d.conf
            m.has_corners = len(d.corners) == 4
            if m.has_corners:
                m.corners = [float(x) for x in d.corners.reshape(-1)]
            arr.detections.append(m)
        self.det_pub.publish(arr)

        if self.debug_pub.get_subscription_count() > 0:
            self.debug_pub.publish(bgr8_to_imgmsg(self._draw(bgr, dets, msg.width, msg.height), msg.header))

        if self.n % 300 == 1:
            n_q = sum(len(d.corners) == 4 for d in dets)
            self.get_logger().info(f'{msg.width}x{msg.height} frames={self.n} gates={len(dets)} 꼭짓점확보={n_q}')

    def _draw(self, bgr, dets, width, height):
        """프레임 bbox + 개구부 4 꼭짓점과 그 사각형.

        0번(가장 큰 = PnP 대상)은 초록, 나머지는 회색. 꼭짓점 순서를 눈으로 확인할 수 있게
        0번 점은 크게 그리고 0~3 번호를 바깥쪽에 붙인다.
        bbox 는 주인공이 아니므로 1 px 어두운 선으로 깔아 둔다 (꼭짓점이 묻히지 않게).
        """
        vis = bgr.copy()
        for k, d in enumerate(dets):
            target = (k == 0)
            x, y, w, h = (int(v) for v in d.bbox)
            cv2.rectangle(vis, (x, y), (x + w, y + h),
                          (0, 140, 0) if target else (110, 110, 110), 1, cv2.LINE_AA)
            if len(d.corners) == 4:
                q = d.corners.astype(np.int32)
                color = (0, 255, 0) if target else (160, 160, 160)
                cv2.polylines(vis, [q], True, color, 2, cv2.LINE_AA)
                ctr = q.mean(axis=0)
                for i, (u, v) in enumerate(q):
                    cv2.circle(vis, (int(u), int(v)), 6 if i == 0 else 4, (0, 0, 255), -1, cv2.LINE_AA)
                    # 번호는 사각형 중심의 반대쪽(바깥)으로 밀어 선·점과 겹치지 않게
                    off = np.array([u, v]) - ctr
                    off = off / max(np.linalg.norm(off), 1e-6) * 16
                    cv2.putText(vis, str(i), (int(u + off[0]) - 4, int(v + off[1]) + 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
                cv2.circle(vis, (int(d.center[0]), int(d.center[1])), 3, color, -1, cv2.LINE_AA)
                if target:
                    cv2.putText(vis, f'PnP target  conf={d.conf:.2f}',
                                (8, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            else:
                # 꼭짓점을 못 딴 검출은 중심만 (경계에 잘렸거나 너무 작음)
                cv2.drawMarker(vis, (int(d.center[0]), int(d.center[1])), (0, 200, 255),
                               cv2.MARKER_TILTED_CROSS, 10, 1)
        cv2.drawMarker(vis, (width // 2, height // 2), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        return vis


def main(args=None):
    rclpy.init(args=args)
    node = GateDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
