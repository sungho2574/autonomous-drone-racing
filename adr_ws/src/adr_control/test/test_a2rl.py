"""Policy contract, GT frame conversion and controller failure transitions."""
import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleStatus
import rclpy

from adr_control.a2rl_gt import state_from_gt
from adr_control.a2rl_runtime import PolicyRunner, load_map_from_config
from adr_control.a2rl_rl_controller import A2RLController

ROOT = Path(__file__).parents[2]
ASSETS = ROOT / 'adr_control/policies/a2rl_monorace'
MAP = ROOT / 'adr_bringup/config/maps/a2rl_monorace.yaml'
SHIFT = np.array(yaml.safe_load(MAP.read_text())['course']['translation_from_training_enu'])


def test_gt_twist_rotation_and_invalid_quaternion():
    msg = Odometry()
    msg.pose.pose.orientation.z = np.sqrt(0.5)
    msg.pose.pose.orientation.w = np.sqrt(0.5)
    msg.twist.twist.linear.x = 2.0
    msg.twist.twist.angular.y = 3.0
    state = state_from_gt(msg)
    np.testing.assert_allclose(state['vel'], [0, 2, 0], atol=1e-12)
    np.testing.assert_allclose(state['rates'], [0, 3, 0])
    msg.pose.pose.orientation.z = msg.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError):
        state_from_gt(msg)


def test_handoff_reference_inference():
    ref = json.loads((Path(__file__).parent / 'data/a2rl_reference_step.json').read_text())
    mp = load_map_from_config(ASSETS / 'config_asrun.yaml', MAP)
    runner = PolicyRunner(ASSETS / 'policy.onnx', mp)
    # Keep the handoff reference in its original frame; translate position only.
    ref['x'] = np.array(ref['x']) + SHIFT
    runner.reset(ref['x'], ref['gate_idx'])
    runner.uptilt = ref['uptilt']
    out = runner.step(dict(pos=ref['x'], vel=ref['v'], quat=ref['q'], rates=ref['w']))
    for actual, expected in [('obs', 'obs'), ('action', 'action'),
                             ('thrust_n', 'cmd_thrust'), ('rates_flu', 'cmd_rates')]:
        np.testing.assert_allclose(out[actual], ref[expected], atol=1e-5, rtol=0)
    np.testing.assert_allclose([out['roll_rate'], out['pitch_rate'], out['yaw_rate']],
                               out['rates_flu'] * [1, -1, -1])
    assert not mp.virtual_mask.any()
    np.testing.assert_allclose(mp.bounds_hi, np.array([-4, 95, 8]) + SHIFT)
    np.testing.assert_allclose(mp.gate_pos[:, :2].mean(axis=0), [0, 0], atol=1e-12)


def test_shared_panel_holes_and_solid_middle():
    mp = load_map_from_config(ASSETS / 'config_asrun.yaml', MAP)
    tracker = PolicyRunner(ASSETS / 'policy.onnx', mp).tracker
    for z in (1.4, 5.0):
        assert not tracker.frame_hit((np.array([-18, 84, z]) + SHIFT), (np.array([-18, 86, z]) + SHIFT))
    assert tracker.frame_hit((np.array([-18, 84, 3.2]) + SHIFT), (np.array([-18, 86, 3.2]) + SHIFT))
    tracker.reset(4)
    ev = tracker.update((np.array([-18, 84, 5.0]) + SHIFT), (np.array([-18, 86, 5.0]) + SHIFT))
    assert ev['passed'] and ev['gate_id'] == 6
    ev = tracker.update((np.array([-18, 86, 1.4]) + SHIFT), (np.array([-18, 84, 1.4]) + SHIFT))
    assert ev['passed'] and ev['gate_id'] == 7


@pytest.fixture
def controller(monkeypatch):
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', f'gates_file:={MAP}'])
    node = A2RLController()
    node._timer.cancel()
    clock = [10.0]
    monkeypatch.setattr(node, 'now', lambda: clock[0])
    node.state_start = node.track_start = node.last_pass = 10.0
    node._status = VehicleStatus()
    node._status.arming_state = VehicleStatus.ARMING_STATE_ARMED
    node._status.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    node.gt = dict(pos=node.mp.start_pos.copy(), vel=np.zeros(3), rates=np.zeros(3),
                   quat=np.array([0., 0., np.sin(node.takeoff_yaw/2), np.cos(node.takeoff_yaw/2)]))
    node.gt_stamp = 10.0
    node.state = 'TRACK'
    sent, modes, lands = [], [], []
    monkeypatch.setattr(node.rate_pub, 'publish', sent.append)
    monkeypatch.setattr(node, 'publish_offboard_mode', lambda **kw: modes.append(kw))
    monkeypatch.setattr(node, 'land', lambda: lands.append(True))
    yield node, clock, sent, modes, lands
    node.destroy_node()
    rclpy.shutdown()


def test_track_sends_rate_only_and_previous_action(controller):
    node, clock, sent, modes, lands = controller
    node.step(.02)
    assert len(sent) == 1 and not lands
    assert modes == [dict(position=False, body_rate=True)]
    assert -1 <= sent[0].thrust_body[2] <= 0
    prev = node.runner.prev_action.copy()
    clock[0] += .02
    node.gt_stamp = clock[0]
    captured = []
    node.debug_pub.publish = lambda msg: captured.append(json.loads(msg.data))
    node.step(.02)
    np.testing.assert_allclose(captured[0]['obs'][12:16], prev, atol=1e-7)


@pytest.mark.parametrize('failure', ['stale', 'clock_gap', 'nonfinite', 'oob', 'offboard'])
def test_track_failure_stops_rate_commands(controller, failure):
    node, clock, sent, modes, lands = controller
    if failure == 'stale':
        node.gt_stamp = 9.0
    elif failure == 'clock_gap':
        node.last_step = 9.0
    elif failure == 'nonfinite':
        node.gt['vel'][0] = np.nan
    elif failure == 'oob':
        node.gt['pos'][2] = 10
    else:
        node._status.nav_state = VehicleStatus.NAVIGATION_STATE_POSCTL
    node.step(.02)
    assert not sent and lands and node.state == 'LAND'


def test_takeoff_timeout_never_enters_rl(controller):
    node, clock, sent, modes, lands = controller
    node.state = 'TAKEOFF'
    node.hold_pos = node.mp.start_pos.copy()
    node.gt['pos'][2] = 0.1
    node.state_start = -30.0
    node.step(.02)
    assert node.state == 'LAND' and lands and not sent


def test_completed_lap_switches_to_hold(controller):
    node, clock, sent, modes, lands = controller
    node.runner.tracker.laps = node.cfg['laps']
    node.step(.02)
    assert node.state == 'HOLD' and not sent and not lands
    assert modes == [dict(position=True)]
