"""PX4 offboard 노드 공통 베이스 (px4_msgs + uXRCE-DDS).

arm / offboard 전환 / 상태 구독 / 세트포인트 발행처럼 컨트롤러 종류와 무관한 부분을 담는다.
- step1: PositionController (TrajectorySetpoint → PX4 position control)
- step2: RL rate controller 는 같은 베이스를 상속해 VehicleRatesSetpoint 를 발행하면 된다.

프레임 규약: 이 베이스의 public API 는 전부 ENU(map)/FLU 이고, PX4 경계에서만 NED/FRD 로 바꾼다.
"""
from __future__ import annotations

from math import isnan

import numpy as np
import rclpy
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

        ns = self.get_parameter('fmu_ns').value
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
        return F.ned_to_enu([self._lpos.x, self._lpos.y, self._lpos.z])

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

    def publish_trajectory_setpoint(self, p_enu, v_enu=None, a_enu=None,
                                    yaw_enu: float = float('nan'), yaw_rate_enu: float = float('nan')):
        """ENU 입력 → NED TrajectorySetpoint. None/NaN 은 '제어 안 함'."""
        sp = TrajectorySetpoint()
        sp.timestamp = self._now_us()
        nan3 = [float('nan')] * 3
        sp.position = [float(x) for x in F.enu_to_ned(p_enu)] if p_enu is not None else nan3
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
