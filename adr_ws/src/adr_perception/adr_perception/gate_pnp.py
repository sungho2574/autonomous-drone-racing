"""게이트 개구부 4 꼭짓점 + 게이트 맵 → PnP 로 기체 위치 추정 → 궤적으로 시각화.

    /adr/gate_detections (코너) + /adr/camera/camera_info (내부파라미터) + gates.yaml (맵)
      → solvePnP → /adr/pnp/marks (추정 위치마다 파란 X 표시)
                   + /adr/pnp/pose, /adr/pnp/target_gate, /adr/pnp/error

**제어에는 전혀 쓰이지 않는다.** PnP 가 얼마나 쓸 만한지 눈으로 보고 숫자로 재기 위한 계측용이다.
인식이 된 순간에만 그 자리에 X 를 하나 찍는다 — 선으로 잇지 않으므로 추정이 듬성듬성한 것도,
옆으로 흩어지는 것도 그대로 보인다. rviz 의 빨간 flown_path(실제 비행)와 겹쳐 보면 편차가 드러난다.

대상 게이트 고르기
  화면에서 가장 큰(=가장 가까운) 검출 1개만 쓴다. 그 검출이 '몇 번 게이트인지'는
  다음 게이트 인덱스 상태기계로 정한다 — 오도메트리로 게이트 평면 통과를 감지해 진행시킨다.
  (오도메트리는 '어느 게이트인지' 고르는 데만 쓴다. 포즈 추정 자체에는 안 들어간다.)

정밀도 — 단일 평면 타겟이라 거리가 멀수록 급격히 나빠진다 (1 px 노이즈, 중앙값):
  2.5 m: 횡방향 0.05 m / 자세 1.0°    3 m: 0.09 m / 1.6°
  4 m:   0.19 m / 2.7°                6 m: 0.77 m / 7.3°
  깊이 오차는 그보다 한 자릿수 작다 (3 m 에서 0.009 m). 궤적이 옆으로 흔들리는 게 정상이다.
또 카메라가 20° 위를 보고 있어 아주 가까우면 개구부 아래 꼭짓점이 화면을 벗어난다.
  정면 접근 기준 수평 자세로 3 m 이상, 기수를 5° 숙이면 2 m 부터 4점이 잡힌다.
"""
import os

import numpy as np
import rclpy
from adr_msgs.msg import GateDetectionArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, Int32
from visualization_msgs.msg import Marker

from adr_perception.pnp import camera_extrinsic, drone_pose_in_map, quat_from_R
from adr_planning.course import load_course

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)


class GatePnP(Node):
    def __init__(self):
        super().__init__('gate_pnp')
        self.declare_parameter('gates_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_marks', 2000)       # 쌓아 둘 X 표시 개수
        self.declare_parameter('mark_size', 0.12)       # X 한 변 [m]
        self.declare_parameter('camera_info_topic', '/adr/camera/camera_info')
        self.declare_parameter('odom_topic', '/adr/odom')
        # camera_link 의 base_link 기준 장착. adr_sim/config/racer_spec.yaml 의 camera 절과 같아야 한다
        self.declare_parameter('cam_xyz', [0.045, 0.0, 0.022])
        self.declare_parameter('cam_tilt_deg', 20.0)        # 위쪽 틸트(+). racer_spec.yaml 의 camera.tilt_deg 와 같아야 함
        self.declare_parameter('max_reproj_px', 3.0)        # 재투영오차 상한
        # 오도메트리와 이만큼 넘게 벌어지면 '다른 게이트를 본 것'으로 보고 버린다.
        # PnP 오차(수십 cm)보다 훨씬 크게 잡아 성능 측정을 왜곡하지 않게 한다. 0 이면 끔.
        self.declare_parameter('max_odom_gap', 5.0)

        gates_file = self.get_parameter('gates_file').value or os.path.join(
            get_package_share_directory('adr_bringup'), 'config', 'gates.yaml')
        self.course = load_course(gates_file)
        self.frame_id = self.get_parameter('frame_id').value
        self.T_base_opt = camera_extrinsic(self.get_parameter('cam_xyz').value,
                                           float(self.get_parameter('cam_tilt_deg').value))
        self.max_reproj = float(self.get_parameter('max_reproj_px').value)
        self.max_gap = float(self.get_parameter('max_odom_gap').value)

        self.K = None
        self.D = None
        self.odom_p = None
        self.next_i = 0            # course.gates 인덱스 — 지금 향하고 있는 게이트
        self._s_prev = None        # 그 게이트 평면까지의 부호거리 (통과 감지용)
        self.n_try = self.n_ok = 0
        self._err_sum = 0.0

        self.max_marks = int(self.get_parameter('max_marks').value)
        self.mark_size = float(self.get_parameter('mark_size').value)
        self.marks = self._new_marker()
        self.marks_pub = self.create_publisher(Marker, '/adr/pnp/marks', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/adr/pnp/pose', 10)
        self.gate_pub = self.create_publisher(Int32, '/adr/pnp/target_gate', 10)
        self.err_pub = self.create_publisher(Float32, '/adr/pnp/error', 10)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._on_info, SENSOR_QOS)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(GateDetectionArray, '/adr/gate_detections', self._on_det, 10)
        self.get_logger().info(
            f'gate_pnp: 게이트 {len(self.course.gates)}개, 개구부 {self.course.inner_size} m, '
            f'카메라 틸트 {self.get_parameter("cam_tilt_deg").value}° → /adr/pnp/marks '
            f'(제어에는 미반영)')

    def _new_marker(self) -> Marker:
        """X 표시를 모아 담을 LINE_LIST 마커. X 하나 = 선분 2개 = 점 4개."""
        m = Marker()
        m.header.frame_id = self.frame_id
        m.ns, m.id = 'pnp', 0
        m.type, m.action = Marker.LINE_LIST, Marker.ADD
        m.scale.x = 0.012                      # 선 두께 [m]
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.5, 1.0, 1.0   # 파랑
        m.pose.orientation.w = 1.0
        return m

    # ------------------------------------------------------------------ 입력
    def _on_info(self, m: CameraInfo):
        if self.K is None:
            self.K = np.array(m.k, dtype=float).reshape(3, 3)
            self.D = np.array(m.d, dtype=float) if len(m.d) else np.zeros(5)
            self.get_logger().info(f'camera_info: fx={self.K[0, 0]:.1f} fy={self.K[1, 1]:.1f} '
                                   f'cx={self.K[0, 2]:.1f} cy={self.K[1, 2]:.1f} {m.width}x{m.height}')

    def _on_odom(self, m: Odometry):
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        self.odom_p = p
        self._advance_gate(p)

    def _advance_gate(self, p):
        """게이트 평면을 통과 방향으로 지나면 다음 게이트로 넘어간다."""
        g = self.course.gates[self.next_i]
        s = float((p - g.center) @ g.normal)
        lateral = np.linalg.norm((p - g.center) - s * g.normal)
        if self._s_prev is not None and self._s_prev < 0.0 <= s and lateral < self.course.inner_size:
            self.next_i = (self.next_i + 1) % len(self.course.gates)
            self._s_prev = None
            self.get_logger().info(f'게이트 {g.id} 통과 → 다음 대상 '
                                   f'{self.course.gates[self.next_i].id}')
            return
        self._s_prev = s

    # ------------------------------------------------------------------ PnP
    def _on_det(self, msg: GateDetectionArray):
        if self.K is None:
            return
        # 가장 큰(= id 0, 검출기가 면적 내림차순 정렬) 검출 중 꼭짓점이 확보된 것
        det = next((d for d in msg.detections if d.has_corners), None)
        if det is None:
            return
        self.n_try += 1
        g = self.course.gates[self.next_i]
        corners = np.array(det.corners, dtype=float).reshape(4, 2)
        T, err = drone_pose_in_map(corners, self.course.inner_size, self.K, self.D,
                                   g.center, g.yaw, self.T_base_opt)
        if T is None or err > self.max_reproj:
            self._warn_throttled(f'PnP 기각: 재투영오차 {err:.2f} px > {self.max_reproj}')
            return
        p = T[:3, 3]
        if self.max_gap > 0 and self.odom_p is not None:
            gap = float(np.linalg.norm(p - self.odom_p))
            if gap > self.max_gap:
                self._warn_throttled(f'PnP 기각: 오도메트리와 {gap:.1f} m 차이 — '
                                     f'게이트 {g.id} 이 아닌 다른 게이트를 봤을 가능성')
                return

        self.n_ok += 1
        q = quat_from_R(T[:3, :3])
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = map(float, q)
        self.pose_pub.publish(ps)
        self.gate_pub.publish(Int32(data=int(g.id)))

        # 수평면 X 자. LINE_LIST 라 점 2개씩 선분이 되므로 대각선 2개 = 점 4개.
        # 비행이 대체로 수평이라 위/비스듬히서 볼 때 X 로 읽힌다.
        h = self.mark_size / 2.0
        for dx, dy in ((-h, -h), (+h, +h), (-h, +h), (+h, -h)):
            self.marks.points.append(Point(x=float(p[0] + dx), y=float(p[1] + dy), z=float(p[2])))
        if len(self.marks.points) > 4 * self.max_marks:
            del self.marks.points[:len(self.marks.points) - 4 * self.max_marks]
        self.marks.header.stamp = ps.header.stamp
        self.marks_pub.publish(self.marks)

        if self.odom_p is not None:
            e = float(np.linalg.norm(p - self.odom_p))
            self._err_sum += e
            self.err_pub.publish(Float32(data=e))
            if self.n_ok % 100 == 0:
                self.get_logger().info(
                    f'PnP {self.n_ok}/{self.n_try} 성공, 대상 게이트 {g.id}, '
                    f'오도메트리 대비 평균 {self._err_sum / self.n_ok:.2f} m (이번 {e:.2f} m)')

    def _warn_throttled(self, text):
        if self.n_try % 100 == 1:
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = GatePnP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
