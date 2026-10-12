"""GT → A2RL ONNX (50 Hz) → PX4 body rates, with position takeoff/landing."""
import json
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleRatesSetpoint, VehicleStatus
from std_msgs.msg import String

from adr_control.a2rl_gt import state_from_gt
from adr_control.a2rl_runtime import PolicyRunner, load_map_from_config, rpy_from_quat
from adr_control.offboard_base import OffboardBase, PX4_PUB_QOS, PX4_SUB_QOS, px4_topic, spin


class A2RLController(OffboardBase):
    def __init__(self):
        super().__init__('a2rl_rl_controller')
        assets = Path(get_package_share_directory('adr_control')) / 'policies/a2rl_monorace'
        defaults = dict(policy_path=str(assets / 'policy.onnx'),
                        gt_topic='/adr/ground_truth/odom', camera_uptilt_deg=45.0,
                        laps=1, land_after=True, time_scale=1.0,
                        gt_timeout=0.10, control_gap_timeout=0.10,
                        takeoff_timeout=30.0, pos_tol=0.25, vel_tol=0.3,
                        max_flight_time=180.0, gate_timeout=20.0, hold_time=3.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        if not self.get_parameter('use_sim_time').value:
            raise ValueError('A2RL GT controller requires use_sim_time:=true (Gazebo only)')
        if self.rate_hz != 50.0 or self.cfg['time_scale'] != 1.0:
            raise ValueError('This policy requires rate_hz=50 and time_scale=1.0')
        if self.cfg['laps'] < 1:
            raise ValueError('laps must be >= 1')
        for name in ('gt_timeout', 'control_gap_timeout', 'takeoff_timeout',
                     'pos_tol', 'vel_tol', 'max_flight_time', 'gate_timeout', 'hold_time'):
            if not np.isfinite(self.cfg[name]) or self.cfg[name] <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not 40 <= self.cfg['camera_uptilt_deg'] <= 50:
            raise ValueError('camera_uptilt_deg must be within trained range 40–50')
        self.mp = load_map_from_config(assets / 'config_asrun.yaml',
                                      map_yaml=self.get_parameter('gates_file').value)
        self.runner = PolicyRunner(self.cfg['policy_path'], self.mp,
                                   uptilt_deg=self.cfg['camera_uptilt_deg'])
        delta = self.mp.gate_pos[0] - self.mp.start_pos
        self.takeoff_yaw = float(np.arctan2(delta[1], delta[0]))
        self.gt = None
        self.gt_stamp = None
        self.last_step = None
        self.track_start = self.last_pass = self.state_start = self.now()
        self.state = 'WAIT_GT'
        self.reason = ''
        self.hold_pos = None
        self.hold_yaw = self.takeoff_yaw
        self.warm = 0
        ns = self.get_parameter('fmu_ns').value
        self.rate_pub = self.create_publisher(
            VehicleRatesSetpoint, px4_topic(f'{ns}/fmu/in/vehicle_rates_setpoint',
                                          VehicleRatesSetpoint), PX4_PUB_QOS)
        self.state_pub = self.create_publisher(String, '/adr/controller_state', 10)
        self.debug_pub = self.create_publisher(String, '/adr/rl/debug', 10)
        self.create_subscription(Odometry, self.cfg['gt_topic'], self.on_gt, PX4_SUB_QOS)
        self.get_logger().info(f'A2RL ready: GT={self.cfg["gt_topic"]}, CPU 50 Hz, '
                               f'policy={self.cfg["policy_path"]}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            state = state_from_gt(msg)
        except ValueError:
            self.gt = None
            return
        if self.gt_stamp is not None and stamp <= self.gt_stamp:
            return
        self.gt, self.gt_stamp = state, stamp

    def fresh_gt(self):
        return (self.gt is not None and self.gt_stamp is not None
                and -0.02 <= self.now() - self.gt_stamp <= self.cfg['gt_timeout'])

    def goto(self, state, reason=''):
        self.get_logger().info(f'{self.state} → {state} {reason}')
        self.state, self.state_start = state, self.now()
        if reason:
            self.reason = reason
        self.publish_state()

    def publish_state(self):
        elapsed = max(0.0, self.now() - self.track_start) if self.state == 'TRACK' else 0.0
        self.state_pub.publish(String(data=f'{self.state} t_traj={elapsed:.2f} '
            f'gate_id={self.mp.gate_ids[self.runner.tracker.target]} '
            f'gates_passed={self.runner.tracker.gates_passed} laps={self.runner.tracker.laps} '
            f'reason={self.reason}'))

    def hold(self):
        self.publish_offboard_mode(position=True)
        self.publish_trajectory_setpoint(self.hold_pos, yaw_enu=self.hold_yaw)

    def abort(self, reason):
        self.get_logger().error(reason)
        self.land()
        self.goto('LAND', reason)

    def step(self, dt):
        now = self.now()
        gap = 0.0 if self.last_step is None else now - self.last_step
        self.last_step = now
        elapsed = now - self.state_start
        if self._tick % 50 == 0:
            self.publish_state()
        if self.state in ('TAKEOFF', 'TRACK', 'HOLD'):
            if not self.armed:
                self.goto('DONE', 'disarmed')
                return
            if not self.in_offboard:
                self.abort('offboard lost')
                return
            if not self.fresh_gt() or gap < 0 or gap > self.cfg['control_gap_timeout']:
                self.abort('GT stale/invalid or control clock gap')
                return
        if self.state == 'WAIT_GT':
            if not (self.fresh_gt() and self.position_valid and self.status_received):
                if self._tick % 250 == 0:
                    self.get_logger().info('Waiting for fresh Gazebo GT and PX4 position/status')
                return
            if np.linalg.norm(self.gt['pos'][:2] - self.mp.start_pos[:2]) > 1.0:
                raise RuntimeError('GT start does not match A2RL map; restart sim with map:=a2rl_monorace')
            self.fix_origin()
            self.hold_pos = self.gt['pos'].copy()
            self.goto('WARMUP')
        elif self.state in ('WARMUP', 'ARMING'):
            if not self.fresh_gt():
                if self.armed:
                    self.abort('GT lost while arming')
                else:
                    self.warm = 0
                    self.goto('WAIT_GT')
                return
            self.hold()
            if self.state == 'WARMUP':
                self.warm += 1
                if self.warm >= self.warmup_count:
                    self.goto('ARMING')
            else:
                if self._tick % 50 == 0:
                    if not self.in_offboard:
                        self.set_offboard_mode()
                    if self.auto_arm and not self.armed:
                        self.arm()
                if self.in_offboard and self.armed:
                    self.hold_pos = self.mp.start_pos.copy()
                    self.goto('TAKEOFF')
        elif self.state == 'TAKEOFF':
            self.hold()
            yaw = float(rpy_from_quat(self.gt['quat'])[2])
            yaw_err = abs((yaw - self.takeoff_yaw + np.pi) % (2 * np.pi) - np.pi)
            if (np.linalg.norm(self.gt['pos'] - self.hold_pos) < self.cfg['pos_tol']
                    and np.linalg.norm(self.gt['vel']) < self.cfg['vel_tol']
                    and yaw_err < 0.15):
                self.runner.reset(self.gt['pos'])
                self.track_start = self.last_pass = now
                self.goto('TRACK')
            elif elapsed > self.cfg['takeoff_timeout']:
                self.abort('takeoff timeout; RL was not started')
        elif self.state == 'TRACK':
            if now - self.track_start > self.cfg['max_flight_time'] or now - self.last_pass > self.cfg['gate_timeout']:
                self.abort('flight/gate progress timeout')
                return
            try:
                out = self.runner.step(self.gt)
                if not np.isfinite(np.r_[out['obs'], out['action_raw'], out['action'],
                                         out['thrust_norm'], out['rates_flu']]).all():
                    raise ValueError('non-finite policy output')
            except Exception as exc:
                self.abort(f'policy inference failed: {exc}')
                return
            events = out['events']
            failure = next((key for key in ('reverse', 'frame_hit', 'out_of_bounds') if events[key]), None)
            if events['passed']:
                self.last_pass = now
            finished = self.runner.tracker.laps >= self.cfg['laps']
            out['gt_stamp'] = self.gt_stamp
            out['command_sent'] = not failure and not finished
            self.debug_pub.publish(String(data=json.dumps(out, default=lambda x:
                x.tolist() if isinstance(x, np.ndarray) else x.item())))
            if failure:
                self.abort(f'gate tracker: {failure}')
                return
            if finished:
                self.hold_pos = self.gt['pos'].copy()
                self.hold_yaw = float(rpy_from_quat(self.gt['quat'])[2])
                self.goto('HOLD', 'requested laps completed')
                self.hold()
                return
            msg = VehicleRatesSetpoint()
            msg.timestamp = self._now_us()
            msg.roll, msg.pitch, msg.yaw = out['roll_rate'], out['pitch_rate'], out['yaw_rate']
            msg.thrust_body = [0.0, 0.0, -out['thrust_norm']]
            self.publish_offboard_mode(position=False, body_rate=True)
            self.rate_pub.publish(msg)
        elif self.state == 'HOLD':
            self.hold()
            if self.cfg['land_after'] and elapsed >= self.cfg['hold_time']:
                self.land()
                self.goto('LAND')
        elif self.state == 'LAND':
            if not self.armed and elapsed > 2.0:
                self.goto('DONE')
            elif self._tick % 50 == 0 and self._status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.land()


def main():
    spin(A2RLController)
