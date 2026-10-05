"""Exercise TRACK with ROS transport stubbed: yaw and derivatives reach PX4."""
import importlib.util
from pathlib import Path
import sys
import types

import numpy as np
import pytest


@pytest.mark.parametrize('scale', [0.5, 1.0, 1.5])
@pytest.mark.parametrize('feedforward', [False, True])
def test_track_heading_and_playback_derivatives(monkeypatch, scale, feedforward):
    root = Path(__file__).parents[2]
    monkeypatch.syspath_prepend(str(root / 'adr_planning'))
    modules = {
        'adr_msgs': types.ModuleType('adr_msgs'),
        'adr_msgs.msg': types.SimpleNamespace(PolynomialTrajectory=object),
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.qos': types.SimpleNamespace(
            DurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=0),
            ReliabilityPolicy=types.SimpleNamespace(RELIABLE=0),
            QoSProfile=lambda **kw: None),
        'std_msgs': types.ModuleType('std_msgs'),
        'std_msgs.msg': types.SimpleNamespace(String=object),
        'adr_control.offboard_base': types.SimpleNamespace(OffboardBase=object, spin=lambda: None),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location('controller_under_test',
        root / 'adr_control/adr_control/px4_position_controller.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    obj = module.PositionController.__new__(module.PositionController)
    obj._t_state = obj._t_traj = 0.0
    obj.state = 'TRACK'
    obj.time_scale, obj.feedforward = scale, feedforward
    obj._publish_state = lambda: None
    obj.publish_offboard_mode = lambda **kw: None
    published = []
    obj.publish_trajectory_setpoint = lambda *a, **kw: published.append((a, kw))
    obj.traj = types.SimpleNamespace(duration=10, sample=lambda t:
        (np.array([t, 0, 0]), np.array([1., 2, 3]), np.array([4., 5, 6]), .7, .8))
    obj.step(.02)
    assert obj._t_traj == pytest.approx(.02 * scale)
    args, kwargs = published[0]
    if feedforward:
        assert np.allclose(args[1], np.array([1, 2, 3]) * scale)
        assert np.allclose(args[2], np.array([4, 5, 6]) * scale**2)
        assert args[3] == pytest.approx(.7)
        assert args[4] == pytest.approx(.8 * scale)
    else:
        assert kwargs['yaw_enu'] == pytest.approx(.7)
