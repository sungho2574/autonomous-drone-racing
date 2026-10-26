import numpy as np
from adr_state_estimation.drift_kf import DriftKF
from adr_state_estimation.timed_drift import TimedDrift, interpolate


def test_interpolation_and_missing_brackets():
    rows = [(1., np.array([10., 0, 0])), (1.04, np.array([10.4, 0, 0]))]
    np.testing.assert_allclose(interpolate(rows, 1.02, .1), [10.2, 0, 0])
    assert interpolate(rows, .99, .1) is None
    assert interpolate(rows, 1.05, .1) is None
    assert interpolate(rows, 1.02, .02) is None
    np.testing.assert_allclose(interpolate(rows, 1., .1), [10, 0, 0])


def test_output_prediction_does_not_change_late_measurement_result():
    immediate = TimedDrift(DriftKF(), 2.5)
    delayed = TimedDrift(DriftKF(), 2.5)
    for i in range(30):
        t = i * .1
        # Outputs can already be ahead of the measurement that arrives later.
        delayed.at(t + .2)
        for f in (immediate, delayed):
            assert f.update(t, [0.1 * t, 0, 0], [0, 0, 0], 1)
    np.testing.assert_allclose(immediate.at(3.2).x, delayed.at(3.2).x)
    np.testing.assert_allclose(immediate.at(3.2).P, delayed.at(3.2).P)
    assert delayed.time == 2.9 or abs(delayed.time - 2.9) < 1e-9


def test_old_measurements_cannot_rewind_state():
    f = TimedDrift(DriftKF(), 2.5)
    f.update(1, [0, 0, 0], [0, 0, 0], 1)
    x, p = f.kf.x.copy(), f.kf.P.copy()
    assert not f.update(.9, [5, 0, 0], [0, 0, 0], 1)
    np.testing.assert_array_equal(f.kf.x, x)
    np.testing.assert_array_equal(f.kf.P, p)


def test_hold_stops_extrapolation_and_rejection_does_not_refresh():
    f = TimedDrift(DriftKF(), 2.5)
    f.update(0, [0, 0, 0], [0, 0, 0], 1)
    f.kf.x[3] = .2
    assert not f.update(1, [100, 0, 0], [0, 0, 0], 1)
    assert f.last_update == 0
    np.testing.assert_allclose(f.at(10).p_d, [.5, 0, 0])
    np.testing.assert_allclose(f.at(20).p_d, [.5, 0, 0])
