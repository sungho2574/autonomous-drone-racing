"""step1 컨트롤러: min-snap 궤적을 PX4 position control 로 추종.

상태기계
  WAIT_TRAJ → WARMUP → ARMING → TAKEOFF → TRACK → HOLD → (LAND) → DONE

- WAIT_TRAJ : /adr/trajectory (latched) 와 PX4 위치·상태가 들어올 때까지 대기
- WARMUP    : 현재 위치 hold 세트포인트를 warmup_count 회 선발행 (PX4 offboard 진입 조건)
- ARMING    : DO_SET_MODE(offboard) + ARM 명령, 둘 다 확인될 때까지 반복(1 Hz)
- TAKEOFF   : 궤적 시작점(호버점)으로 상승, pos_tol 이내 & 저속이면 TRACK
- TRACK     : t=0 부터 궤적 샘플링(pos/vel/acc/yaw/yawrate feedforward). 끝나면 HOLD
- HOLD      : 궤적 끝점 hold. land_after=true 면 hold_time 후 LAND
- LAND      : NAV_LAND 명령 후 disarm 될 때까지 대기 → DONE
"""
from __future__ import annotations

import numpy as np
from adr_interfaces.msg import PolynomialTrajectory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from adr_control.offboard_base import OffboardBase, spin
from adr_planning.min_snap import Trajectory

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def trajectory_from_msg(msg: PolynomialTrajectory) -> Trajectory:
    return Trajectory.from_plain([
        {'duration': s.duration, 'cx': list(s.cx), 'cy': list(s.cy),
         'cz': list(s.cz), 'cyaw': list(s.cyaw)} for s in msg.segments])


class PositionController(OffboardBase):
    def __init__(self):
        super().__init__('px4_position_controller')
        self.declare_parameter('trajectory_topic', '/adr/trajectory')
        self.declare_parameter('pos_tol', 0.25)          # TAKEOFF 완료 판정 [m]
        self.declare_parameter('vel_tol', 0.3)           # TAKEOFF 완료 판정 [m/s]
        self.declare_parameter('takeoff_timeout', 20.0)  # [s] 초과 시 그냥 TRACK 시작
        self.declare_parameter('feedforward', True)      # 속도/가속 피드포워드 사용
        self.declare_parameter('land_after', True)
        self.declare_parameter('hold_time', 3.0)         # HOLD 유지 시간 [s]
        self.declare_parameter('time_scale', 1.0)        # 궤적 재생 배속 (<1 이면 느리게)

        self.pos_tol = float(self.get_parameter('pos_tol').value)
        self.vel_tol = float(self.get_parameter('vel_tol').value)
        self.takeoff_timeout = float(self.get_parameter('takeoff_timeout').value)
        self.feedforward = bool(self.get_parameter('feedforward').value)
        self.land_after = bool(self.get_parameter('land_after').value)
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.time_scale = float(self.get_parameter('time_scale').value)

        self.traj: Trajectory | None = None
        self.state = 'WAIT_TRAJ'
        self._t_state = 0.0       # 현재 상태 진입 후 경과 시간
        self._t_traj = 0.0        # TRACK 궤적 시간
        self._hold_p = None
        self._hold_yaw = float('nan')
        self._warm = 0

        self.create_subscription(PolynomialTrajectory,
                                 self.get_parameter('trajectory_topic').value,
                                 self._on_traj, LATCHED)
        self._state_pub = self.create_publisher(String, '/adr/controller_state', 10)
        self.get_logger().info('px4_position_controller ready, waiting for trajectory')

    # ------------------------------------------------------------------
    def _on_traj(self, msg: PolynomialTrajectory):
        if self.state not in ('WAIT_TRAJ', 'DONE'):
            self.get_logger().warn('비행 중 궤적 갱신은 무시함')
            return
        self.traj = trajectory_from_msg(msg)
        self.get_logger().info(f'trajectory: {len(self.traj.segments)} segments, {self.traj.duration:.1f}s')

    def _goto(self, state: str):
        self.get_logger().info(f'{self.state} → {state}')
        self.state = state
        self._t_state = 0.0

    def _publish_state(self):
        if self._tick % int(self.rate_hz) == 0:
            self._state_pub.publish(String(data=f'{self.state} t_traj={self._t_traj:.2f}'))

    def _hold(self):
        """현재 hold 목표를 position-only 로 발행."""
        self.publish_offboard_mode(position=True)
        self.publish_trajectory_setpoint(self._hold_p, yaw_enu=self._hold_yaw)

    # ------------------------------------------------------------------
    def step(self, dt: float):
        self._t_state += dt
        self._publish_state()
        s = self.state

        if s == 'WAIT_TRAJ':
            if self.traj is None or not self.position_valid or not self.status_received:
                if self._tick % int(5 * self.rate_hz) == 0:
                    self.get_logger().info(
                        f'waiting: traj={self.traj is not None} pos={self.position_valid} '
                        f'status={self.status_received}')
                return
            # 이륙 전 hold 목표 = 현재 위치(바닥). yaw 는 궤적 시작 yaw
            self._hold_p = self.position_enu.copy()
            self._hold_yaw = self.traj.sample(0.0)[3]
            self._warm = 0
            self._goto('WARMUP')

        elif s == 'WARMUP':
            self._hold()
            self._warm += 1
            if self._warm >= self.warmup_count:
                self._goto('ARMING')

        elif s == 'ARMING':
            self._hold()
            if self._tick % int(self.rate_hz) == 0:      # 1 Hz 로 명령 재전송
                if not self.in_offboard:
                    self.set_offboard_mode()
                if self.auto_arm and not self.armed:
                    self.arm()
            if self.in_offboard and self.armed:
                p0, _, _, yaw0, _ = self.traj.sample(0.0)
                self._hold_p = np.array(p0)
                self._hold_yaw = yaw0
                self._goto('TAKEOFF')

        elif s == 'TAKEOFF':
            self._hold()
            err = np.linalg.norm(self.position_enu - self._hold_p)
            spd = np.linalg.norm(self.velocity_enu)
            if (err < self.pos_tol and spd < self.vel_tol) or self._t_state > self.takeoff_timeout:
                if self._t_state > self.takeoff_timeout:
                    self.get_logger().warn(f'takeoff timeout (err={err:.2f} m), tracking anyway')
                self._t_traj = 0.0
                self._goto('TRACK')

        elif s == 'TRACK':
            self._t_traj += dt * self.time_scale
            p, v, a, yaw, yr = self.traj.sample(self._t_traj)
            if self.feedforward:
                self.publish_offboard_mode(position=True, velocity=True, acceleration=True)
                self.publish_trajectory_setpoint(p, v, a, yaw, yr)
            else:
                self.publish_offboard_mode(position=True)
                self.publish_trajectory_setpoint(p, yaw_enu=yaw)
            if self._t_traj >= self.traj.duration:
                self._hold_p = np.array(p)
                self._hold_yaw = yaw
                self._goto('HOLD')

        elif s == 'HOLD':
            self._hold()
            if self.land_after and self._t_state > self.hold_time:
                self.land()
                self._goto('LAND')

        elif s == 'LAND':
            # NAV_LAND 로 모드가 바뀌므로 세트포인트는 더 보내지 않는다.
            if not self.armed and self._t_state > 2.0:
                self._goto('DONE')
            elif self._t_state > 60.0:
                self.get_logger().warn('landing timeout')
                self._goto('DONE')

        elif s == 'DONE':
            pass


def main():
    spin(PositionController)


if __name__ == '__main__':
    main()
