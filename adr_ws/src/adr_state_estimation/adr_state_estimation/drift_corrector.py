"""VIO 를 게이트 PnP 로 보정해 rviz 에 그린다 (논문 §2.4 drift-correction KF).

    /adr/vio/odom  (VIO, map — vio_align 이 시작 시 한 번만 정렬한 뒤로는 순수 VIO)
    /adr/pnp/gate  (GatePnP — 게이트 4 꼭짓점 PnP 로 얻은 기체 위치, 회전은 버림)
      → KF(drift state [p_d, v_d]) → /adr/state/corrected, /adr/state/corrected_path

**제어에는 전혀 들어가지 않는다.** 보는 용도다.
rviz 에서 연한 VioPath(쌩 VIO)와 진한 CorrectedPath(보정) 를 겹쳐 보면 효과가 바로 보인다.

시간 정렬
  PnP 측정값의 잔차는 y = p_pnp − p_vio(같은 시각) 이므로, VIO 포즈를 stamp 로 되짚어야 한다.
  이미지 시각 앞뒤 VIO를 보간한다. 뒤쪽 샘플은 제한 시간 동안 기다린다.
  KF 갱신 상태는 측정 시각에 유지하고, 출력만 현재 VIO 시각으로 예측한다.
  이미 처리한 측정보다 오래된 PnP는 기각한다.

거리 필터와 KF 혁신 잔차 기반 이상치 기각을 적용한다.
"""
import json
import time
import numpy as np
import rclpy
from adr_msgs.msg import GatePnP
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Point
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32, String, ColorRGBA

from adr_state_estimation.drift_kf import DriftKF
from adr_state_estimation.timed_drift import TimedDrift, interpolate


def stamp_s(h) -> float:
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class DriftCorrector(Node):
    def __init__(self):
        super().__init__('drift_corrector')
        self.declare_parameter('vio_topic', '/adr/vio/odom')
        self.declare_parameter('pnp_topic', '/adr/pnp/gate')
        self.declare_parameter('reference_topic', '/adr/odom')   # 오차 리포트용(진실값/mocap). 보정에는 미사용
        self.declare_parameter('frame_id', 'map')
        # KF (논문 기본값 σ_p=0.1, σ_v=0.2, λ_r=1.0)
        self.declare_parameter('sigma_p', 0.1)
        self.declare_parameter('sigma_v', 0.2)
        self.declare_parameter('lambda_r', 1.0)
        # PnP 위치 불확실성 [m]. 이 코스 실측(3 m/1 px)에서 깊이 0.01, 횡방향 0.09 수준이라
        # 여유를 둬 0.25 로 잡았다. 논문은 한 검출을 50회 perturb 해 σ 를 뽑으라고 한다.
        self.declare_parameter('sigma_r', [0.25, 0.25, 0.25])
        # 논문 §2.3(a) 거리 필터. 논문 값 1~13 m 는 실외 대형 트랙 기준이라 이 코스(반경 4 m)에 맞게 축소.
        self.declare_parameter('tau_d_min', 1.0)
        self.declare_parameter('tau_d_max', 8.0)
        self.declare_parameter('pnp_wait_s', 0.25)
        self.declare_parameter('max_dt', 0.10)          # 보간 앞뒤 VIO 최대 간격 [s]
        # 측정이 이 시간 넘게 끊기면 등속 외삽을 멈추고 마지막 drift 를 유지한다.
        # 이 코스는 게이트 사이 ~2 초라 그보다 넉넉히 잡는다. (논문에 없는 처리 — drift_kf.predict 주석 참고)
        self.declare_parameter('hold_timeout', 2.5)
        self.declare_parameter('innovation_gate', 7.81)  # 3D NIS threshold; 0 disables
        self.declare_parameter('v_max', 0.5)            # drift 속도 상한 [m/s]
        self.declare_parameter('path_max_points', 6000)

        self.frame_id = self.get_parameter('frame_id').value
        self.kf = DriftKF(sigma_p=float(self.get_parameter('sigma_p').value),
                          sigma_v=float(self.get_parameter('sigma_v').value),
                          sigma_r=self.get_parameter('sigma_r').value,
                          lambda_r=float(self.get_parameter('lambda_r').value),
                          v_max=float(self.get_parameter('v_max').value),
                          innovation_gate=float(self.get_parameter('innovation_gate').value))
        self.tau_min = float(self.get_parameter('tau_d_min').value)
        self.tau_max = float(self.get_parameter('tau_d_max').value)
        self.max_dt = float(self.get_parameter('max_dt').value)
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)
        self.max_points = int(self.get_parameter('path_max_points').value)

        self.timed = TimedDrift(self.kf, self.hold_timeout)
        self.wait_s = float(self.get_parameter("pnp_wait_s").value)
        self._pending = []
        self._n_rej_time = 0
        self.create_timer(0.02, self._drain_pnp)
        self._buf = []              # [(t, p_vio)] 최근 VIO 포즈
        self._ref = None
        self._n_rej_dist = 0
        self._holding = False
        self._sum_vio = self._sum_cor = 0.0
        self._n_err = 0

        self.decision_pub = self.create_publisher(String, '/adr/pnp/decision', 10)
        self.marks_pub = self.create_publisher(Marker, '/adr/pnp/marks', 10)
        self.marks = Marker()
        self.marks.header.frame_id = self.frame_id
        self.marks.ns = 'pnp'
        self.marks.type = Marker.LINE_LIST
        self.marks.scale.x = 0.018
        self.marks.pose.orientation.w = 1.0
        self.marks.color.a = 1.0
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.odom_pub = self.create_publisher(Odometry, '/adr/state/corrected', 10)
        self.path_pub = self.create_publisher(Path, '/adr/state/corrected_path', 10)
        self.drift_pub = self.create_publisher(Vector3Stamped, '/adr/state/drift', 10)
        self.err_pub = self.create_publisher(Float32, '/adr/state/error', 10)
        self.create_subscription(Odometry, self.get_parameter('vio_topic').value, self._on_vio, 20)
        self.create_subscription(GatePnP, self.get_parameter('pnp_topic').value, self._on_pnp, 10)
        self.create_subscription(Odometry, self.get_parameter('reference_topic').value, self._on_ref, 10)
        self.create_timer(5.0, self._report)
        self.get_logger().info(
            f'drift_corrector: VIO {self.get_parameter("vio_topic").value} + '
            f'PnP {self.get_parameter("pnp_topic").value} → /adr/state/corrected (제어 미반영). '
            f'σ_p={self.kf.sigma_p} σ_v={self.kf.sigma_v} λ_r={self.kf.lambda_r} '
            f'거리필터 {self.tau_min}~{self.tau_max} m, hold {self.hold_timeout} s')

    # ------------------------------------------------------------------
    def _on_ref(self, m: Odometry):
        p = m.pose.pose.position
        self._ref = np.array([p.x, p.y, p.z])

    def _on_vio(self, m: Odometry):
        t = stamp_s(m.header)
        p = m.pose.pose.position
        p_vio = np.array([p.x, p.y, p.z])

        if not np.all(np.isfinite(p_vio)) or (self._buf and t <= self._buf[-1][0]):
            return
        self._buf.append((t, p_vio))
        self._buf = self._buf[-300:]
        self._drain_pnp()
        predicted = self.timed.at(t)
        self._holding = (self.timed.last_update is None or
                         t - self.timed.last_update > self.hold_timeout)
        p_cor = predicted.correct(p_vio)

        out = Odometry()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.frame_id
        out.child_frame_id = 'corrected_base_link'
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, p_cor)
        out.pose.pose.orientation = m.pose.pose.orientation    # 자세·속도는 VIO 그대로 (논문 §2.4)
        out.twist = m.twist
        self.odom_pub.publish(out)

        ps = PoseStamped()
        ps.header = out.header
        ps.pose = out.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_points:
            del self.path.poses[:len(self.path.poses) - self.max_points]
        self.path.header.stamp = out.header.stamp
        self.path_pub.publish(self.path)

        d = Vector3Stamped()
        d.header = out.header
        d.vector.x, d.vector.y, d.vector.z = map(float, predicted.p_d)
        self.drift_pub.publish(d)

        if self._ref is not None:
            e_cor = float(np.linalg.norm(p_cor - self._ref))
            self._sum_vio += float(np.linalg.norm(p_vio - self._ref))
            self._sum_cor += e_cor
            self._n_err += 1
            self.err_pub.publish(Float32(data=e_cor))

    def _on_pnp(self, m: GatePnP):
        # 논문 §2.3(a) 거리 필터 — 너무 가까우면 원근 왜곡, 너무 멀면 코너 해상도 부족
        if not (self.tau_min <= m.distance_m <= self.tau_max):
            self._decision(m, False, "distance")
            self._n_rej_dist += 1
            return
        self._pending.append((stamp_s(m.header), time.monotonic(), m))
        self._pending.sort(key=lambda item: item[0])
        if len(self._pending) > 100:
            _, _, dropped = self._pending.pop(0)
            self._decision(dropped, False, "queue_overflow")
            self._n_rej_time += 1
        self._drain_pnp()

    def _drain_pnp(self):
        while self._pending:
            t, received, m = self._pending[0]
            if self.timed.time is not None and t <= self.timed.time:
                self._pending.pop(0)
                self._decision(m, False, "time_alignment")
                self._n_rej_time += 1
                continue
            if not self._buf or t > self._buf[-1][0]:
                if time.monotonic() - received < self.wait_s:
                    break
                self._pending.pop(0)
                self._decision(m, False, "time_alignment")
                self._n_rej_time += 1
                continue
            self._pending.pop(0)
            p_vio = interpolate(self._buf, t, self.max_dt)
            if p_vio is None:
                self._decision(m, False, "no_interpolation")
                self._n_rej_time += 1
                continue
            z = np.array([m.pos_w.x, m.pos_w.y, m.pos_w.z])
            accepted = self.timed.update(t, z, p_vio, m.quality)
            self._decision(m, accepted, "accepted" if accepted else "innovation_or_invalid")

    def _decision(self, m, accepted, reason):
        self.decision_pub.publish(String(data=json.dumps(dict(
            stamp=stamp_s(m.header), gate_id=int(m.gate_id),
            accepted=bool(accepted), reason=reason))))
        if not np.isfinite([m.pos_w.x, m.pos_w.y, m.pos_w.z]).all():
            return
        color = ColorRGBA(r=0.0, g=0.65, b=0.55, a=1.0) if accepted else ColorRGBA(r=0.9, g=0.15, b=0.15, a=1.0)
        for dx, dy in ((-.12, -.12), (.12, .12), (-.12, .12), (.12, -.12)):
            self.marks.points.append(Point(x=m.pos_w.x+dx, y=m.pos_w.y+dy, z=m.pos_w.z))
            self.marks.colors.append(color)
        self.marks.points = self.marks.points[-8000:]
        self.marks.colors = self.marks.colors[-8000:]
        self.marks.header.stamp = m.header.stamp
        self.marks_pub.publish(self.marks)

    def _report(self):
        if self._n_err == 0:
            return
        v, c = self._sum_vio / self._n_err, self._sum_cor / self._n_err
        gain = (1.0 - c / v) * 100.0 if v > 1e-6 else 0.0
        self.get_logger().info(
            f'KF update {self.kf.n_update}회 (거리필터 기각 {self._n_rej_dist}, '
            f'시간정합 기각 {self._n_rej_time}, 대기 {len(self._pending)}, '
            f'이상치/유효성 기각 {self.kf.n_rejected}, NIS {self.kf.last_nis:.2f})'
            f'{" [유지모드]" if self._holding else ""}, '
            f'drift 추정 {np.round(self.kf.p_d, 3)} m | '
            f'기준 대비 평균오차  VIO {v:.3f} → 보정 {c:.3f} m ({gain:+.0f}%)')


def main(args=None):
    rclpy.init(args=args)
    node = DriftCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
