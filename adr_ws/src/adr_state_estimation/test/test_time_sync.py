import numpy as np
from adr_state_estimation.time_sync import PoseSync


def test_pnp_before_vio_waits_and_interpolates():
    s = PoseSync()
    s.add_pnp(1.05, 'pnp', 10)
    s.add_pose(1, [0, 0, 0])
    assert s.drain(10.2) == []
    s.add_pose(1.1, [1, 2, 3])
    t, p, payload = s.drain(10.3)[0]
    np.testing.assert_allclose(p, [.5, 1, 1.5])
    assert t == 1.05 and payload == 'pnp'
    assert s.drain(10.4) == []


def test_time_history_survives_250hz_and_delayed_pnp():
    s = PoseSync()
    for i in range(2501):
        s.add_pose(i/250, [i/250, 0, 0])
    s.add_pnp(5.002, None, 12)
    assert s.drain(12)[0][1][0] == 5.002


def test_reverse_arrival_order_duplicate_and_expiration():
    s = PoseSync(wait=1)
    s.add_pnp(1.1, 'later', 0)
    s.add_pnp(1.0, 'earlier', 0)
    s.add_pnp(1.0, 'duplicate', 0)
    s.add_pose(1.1, [1, 0, 0])
    s.add_pose(1.0, [0, 0, 0])
    assert [r[2] for r in s.drain(.1)] == ['earlier', 'later']
    s.add_pnp(1.05, 'too_late', .2)
    assert s.counts['duplicate_or_late'] == 2
    s.add_pnp(2, 'no_vio', .2)
    assert s.drain(2) == []
    assert s.counts['wait_timeout'] == 1


def test_no_extrapolation_or_interpolation_across_gap():
    s = PoseSync(wait=1)
    s.add_pose(1, [0, 0, 0]); s.add_pose(2, [10, 0, 0])
    s.add_pnp(1.5, None, 0)
    assert s.drain(.5) == []
    assert s.drain(2) == []
    assert s.counts['vio_gap'] == 1


def test_pending_and_history_bounded():
    s = PoseSync(limit=2, history=1)
    for i in range(3): s.add_pnp(i, None, 0)
    assert len(s.pending) == 2 and s.counts['overflow'] == 1
    for i in range(20): s.add_pose(i, [i, 0, 0])
    assert len(s.poses) <= 3
    s.drain(.1)
    assert s.counts['history_expired'] == 2
