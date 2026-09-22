"""색상(주황) 기반 게이트 검출기.

/adr/camera/image_raw → HSV 임계 → 모폴로지 → 컨투어(RETR_CCOMP: 바깥 프레임 + 안쪽 구멍)
  → 게이트마다 bbox / 개구부 중심 / 면적 → /adr/gate_detections (GateDetectionArray, 2D 픽셀)

디버그 오버레이(bbox·중심점·id)는 /adr/perception/debug_image 로 내되, **구독자가 있을 때만** 그린다:
    ros2 run rqt_image_view rqt_image_view /adr/perception/debug_image

step 3 에서는 이 자리를 Gatenet(코너 검출) + PnP 로 바꾸고 3D 결과를 /adr/detected_gates(GateArray) 로 낸다.
HSV 범위는 sim 게이트 색(1.0, 0.45, 0.0 → H≈13, S·V 높음) 기준. 실기체 조명에 맞춰 파라미터로 조정.
"""
import cv2
import numpy as np
import rclpy
from adr_msgs.msg import GateDetection, GateDetectionArray
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


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
        self.declare_parameter('debug_topic', '/adr/perception/debug_image')

        self.lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        self.upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        self.min_area = float(self.get_parameter('min_area').value)
        self.min_hole_area = float(self.get_parameter('min_hole_area').value)
        k = int(self.get_parameter('morph_kernel').value)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)) if k > 1 else None
        self.max_gates = int(self.get_parameter('max_gates').value)

        self.bridge = CvBridge()
        self.det_pub = self.create_publisher(GateDetectionArray, '/adr/gate_detections', 10)
        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_topic').value, SENSOR_QOS)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image, SENSOR_QOS)
        self.n = 0
        self.get_logger().info(f'gate_detector: HSV {self.lower.tolist()}~{self.upper.tolist()}, '
                               f'debug → {self.get_parameter("debug_topic").value} (구독자 있을 때만)')

    # ------------------------------------------------------------------
    def detect(self, bgr: np.ndarray):
        """BGR 이미지 → [(bbox(x,y,w,h), center(u,v), area, has_hole, conf)] 면적 내림차순."""
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
            if hole is not None and hole_area >= self.min_hole_area:
                m = cv2.moments(hole)
                cu, cv_ = m['m10'] / m['m00'], m['m01'] / m['m00']
                has_hole = True
            else:
                cu, cv_ = x + w / 2, y + h / 2
                has_hole = False
            # 신뢰도: 프레임(링) 면적 비율이 사각 링에 가깝고 종횡비가 1 에 가까울수록 높게
            ring_ratio = (area - hole_area) / max(w * h, 1.0)
            aspect = min(w, h) / max(w, h)
            conf = float(np.clip(0.5 * aspect + 0.5 * min(ring_ratio / 0.5, 1.0), 0.0, 1.0))
            out.append(((x, y, w, h), (cu, cv_), area, has_hole, conf))
        out.sort(key=lambda d: d[2], reverse=True)
        return out[:self.max_gates], mask

    def _on_image(self, msg: Image):
        self.n += 1
        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dets, _ = self.detect(bgr)

        arr = GateDetectionArray()
        arr.header = msg.header
        arr.image_width, arr.image_height = msg.width, msg.height
        for k, ((x, y, w, h), (u, v), area, has_hole, conf) in enumerate(dets):
            d = GateDetection()
            d.id = k
            d.u, d.v = float(u), float(v)
            d.x, d.y, d.w, d.h = float(x), float(y), float(w), float(h)
            d.area, d.has_hole, d.confidence = float(area), bool(has_hole), conf
            arr.detections.append(d)
        self.det_pub.publish(arr)

        if self.debug_pub.get_subscription_count() > 0:
            vis = bgr.copy()
            for k, ((x, y, w, h), (u, v), area, has_hole, conf) in enumerate(dets):
                color = (0, 255, 0) if has_hole else (0, 200, 255)
                cv2.rectangle(vis, (x, y), (x + w, y + h), color, 2)
                cv2.circle(vis, (int(u), int(v)), 5, (0, 0, 255), -1)
                cv2.putText(vis, f'G{k} {conf:.2f}', (x, max(y - 6, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            cx, cy = msg.width // 2, msg.height // 2
            cv2.drawMarker(vis, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
            out = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            out.header = msg.header
            self.debug_pub.publish(out)

        if self.n % 300 == 1:
            self.get_logger().info(f'{msg.width}x{msg.height} frames={self.n} gates={len(dets)}')


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
