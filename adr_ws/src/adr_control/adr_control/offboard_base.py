"""PX4 offboard 노드 공통 베이스 (px4_msgs + uXRCE-DDS).

arm / offboard 전환 / 상태 구독 / 세트포인트 발행처럼 컨트롤러 종류와 무관한 부분을 담는다.
- step1: PositionController (TrajectorySetpoint → PX4 position control)
- step2: RL rate controller 는 같은 베이스를 상속해 VehicleRatesSetpoint 를 발행하면 된다.

프레임 규약: 이 베이스의 public API 는 전부 ENU(map)/FLU 이고, PX4 경계에서만 NED/FRD 로 바꾼다.

map(월드) 원점 vs PX4 local 원점 (origin_mode 파라미터)
  - 'world' : PX4 local 프레임 == map. 외부 위치(EV: sim 진실값 / 실기체 mocap)로 EKF2 를 돌릴 때.
  - 'start' : PX4 local 원점 = 기체가 부팅한 자리(GPS 시뮬 모드). 기체가 gates.yaml 의 start 에
              놓여 있다고 보고, 이륙 전 정지 상태에서 offset = start − p_local 을 한 번 재서
              이후 모든 세트포인트/위치를 보정한다. 결과 offset 은 /adr/local_origin 으로 latched 발행
              → px4_odom_to_tf 가 같은 값으로 TF 를 보정한다.
"""
from __future__ import annotations

import os
from math import isnan

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from adr_control import frames as F

# PX4 → ROS 토픽(/fmu/out/*): best effort + volatile. 신뢰성 QoS 로 구독하면 아무것도 안 온다.
PX4_SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
PX4_PUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
LATCHED_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


def px4_topic(base: str, msg_type) -> str:
    """버전 관리되는 메시지는 uXRCE-DDS 토픽에 '_v<MESSAGE_VERSION>' 이 붙는다.

    PX4 1.18 기준 vehicle_status → vehicle_status_v4, vehicle_local_position → vehicle_local_position_v1.
    MESSAGE_VERSION 이 0 이거나 없는 메시지는 접미사 없이 그대로다. px4_msgs 를 올릴 때
    토픽 이름을 손으로 고치지 않도록 상수에서 유도한다.
    """
    v = getattr(msg_type, 'MESSAGE_VERSION', 0)
    return f'{base}_v{v}' if v else base


class OffboardBase(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('warmup_count', 20)     # offboard 전환 전 미리 보낼 세트포인트 수(≥10 권장)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('fmu_ns', '')          # PX4_UXRCE_DDS_NS 를 쓰면 '/<ns>'
        self.declare_parameter('origin_mode', 'world')  # 'world' | 'start' (모듈 docstring 참고)
        self.declare_parameter('gates_file', '')        # origin_mode=start 일 때 start 를 읽을 yaml

        ns = self.get_parameter('fmu_ns').value
        self.origin_mode = self.get_parameter('origin_mode').value
        self.origin = np.zeros(3)     # map = local + origin  (ENU)
        self._origin_pub = self.create_publisher(PointStamped, '/adr/local_origin', LATCHED_QOS)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.warmup_count = int(self.get_parameter('warmup_count').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)

        self._mode_pub = self.create_publisher(OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', PX4_PUB_QOS)
        self._sp_pub = self.create_publisher(TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', PX4_PUB_QOS)
        self._cmd_pub = self.create_publisher(VehicleCommand, f'{ns}/fmu/in/vehicle_command', PX4_PUB_QOS)
        self.create_subscription(VehicleLocalPosition,
                                 px4_topic(f'{ns}/fmu/out/vehicle_local_position', VehicleLocalPosition),
                                 self._on_local_position, PX4_SUB_QOS)
        self.create_subscription(VehicleStatus,
                                 px4_topic(f'{ns}/fmu/out/vehicle_status', VehicleStatus),
                                 self._on_status, PX4_SUB_QOS)

        self._lpos: VehicleLocalPosition | None = None
        self._status: VehicleStatus | None = None
        self._tick = 0
        self._timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

    # ------------------------------------------------------------------ 상태
    def _on_local_position(self, msg: VehicleLocalPosition):
        self._lpos = msg

    def _on_status(self, msg: VehicleStatus):
        self._status = msg

    @property
    def position_valid(self) -> bool:
        return self._lpos is not None and self._lpos.xy_valid and self._lpos.z_valid

    @property
    def position_enu(self) -> np.ndarray:
        """PX4 local 프레임 위치 (ENU)."""
        return F.ned_to_enu([self._lpos.x, self._lpos.y, self._lpos.z])

    @property
    def position_world(self) -> np.ndarray:
        """map(월드) 프레임 위치 = local + origin."""
        return self.position_enu + self.origin

    def fix_origin(self):
        """이륙 전 정지 상태에서 호출. origin_mode 에 따라 map↔local offset 을 정하고 발행한다."""
        if self.origin_mode == 'start':
            gates_file = self.get_parameter('gates_file').value or os.path.join(
                get_package_share_directory('adr_bringup'), 'config', 'gates.yaml')
            from adr_planning.course import load_course
            start = load_course(gates_file).start
            ground = np.array([start[0], start[1], 0.0])       # 이륙 전 = 바닥
            self.origin = ground - self.position_enu
        elif self.origin_mode != 'world':
            raise ValueError(f"origin_mode 는 'world' 또는 'start': {self.origin_mode}")
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x, msg.point.y, msg.point.z = map(float, self.origin)
        self._origin_pub.publish(msg)
        self.get_logger().info(f'origin_mode={self.origin_mode}: map = local + {np.round(self.origin, 3)}')

    @property
    def velocity_enu(self) -> np.ndarray:
        return F.ned_to_enu([self._lpos.vx, self._lpos.vy, self._lpos.vz])

    @property
    def yaw_enu(self) -> float:
        return F.yaw_ned_to_enu(self._lpos.heading)

    @property
    def armed(self) -> bool:
        return self._status is not None and self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    @property
    def in_offboard(self) -> bool:
        return self._status is not None and self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    @property
    def status_received(self) -> bool:
        return self._status is not None

    # ------------------------------------------------------------------ 발행
    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self, position=True, velocity=False, acceleration=False,
                              attitude=False, body_rate=False):
        m = OffboardControlMode()
        m.timestamp = self._now_us()
        m.position, m.velocity, m.acceleration = position, velocity, acceleration
        m.attitude, m.body_rate = attitude, body_rate
        self._mode_pub.publish(m)

    def publish_trajectory_setpoint(self, p_world, v_enu=None, a_enu=None,
                                    yaw_enu: float = float('nan'), yaw_rate_enu: float = float('nan')):
        """map(ENU) 입력 → PX4 local NED TrajectorySetpoint. None/NaN 은 '제어 안 함'."""
        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        nan3 = [float('nan')] * 3
        p_local = np.asarray(p_world, dtype=float) - self.origin if p_world is not None else None
        sp.position = [float(x) for x in F.enu_to_ned(p_local)] if p_local is not None else nan3
        sp.velocity = [float(x) for x in F.enu_to_ned(v_enu)] if v_enu is not None else nan3
        sp.acceleration = [float(x) for x in F.enu_to_ned(a_enu)] if a_enu is not None else nan3
        sp.jerk = nan3
        sp.yaw = float('nan') if isnan(yaw_enu) else float(F.yaw_enu_to_ned(yaw_enu))
        sp.yawspeed = float('nan') if isnan(yaw_rate_enu) else float(F.yaw_rate_enu_to_ned(yaw_rate_enu))
        self._sp_pub.publish(sp)

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0, param3: float = 0.0):
        c = VehicleCommand()
        c.timestamp = self._now_us()
        c.command = command
        c.param1, c.param2, c.param3 = float(param1), float(param2), float(param3)
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self._cmd_pub.publish(c)

    def arm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        # param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED(1), param2 = PX4 custom main mode OFFBOARD(6)
        self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def land(self):
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ------------------------------------------------------------------ 루프
    def _on_timer(self):
        self._tick += 1
        self.step(1.0 / self.rate_hz)

    def step(self, dt: float):
        raise NotImplementedError


def spin(node_cls, *args):
    rclpy.init()
    node = node_cls(*args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
