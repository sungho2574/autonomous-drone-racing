"""Heading integration: serialization, camera geometry, gate handoffs and limits."""
from pathlib import Path
import numpy as np
import pytest
from adr_planning.course import Gate, course_waypoints, load_course
from adr_planning.min_snap import Trajectory, plan
from adr_planning.perception_heading import (HeadingOptions, angular_peaks, blend_angle,
    desired_headings, gate_schedule, optical_points, perception_aware_heading)
MAPS = Path(__file__).parents[2] / 'adr_bringup/config/maps'


@pytest.fixture(scope='module')
def figure8():
    c = load_course(MAPS / 'figure8.yaml')
    wp, yaw = course_waypoints(c, 2, .8)
    baseline = plan(wp, yaw, v_avg=3, v_max=6, a_max=8)
    opts = HeadingOptions(max_rate_deg=60, max_accel_deg=100)
    out, report = perception_aware_heading(baseline, c, 2, .8, options=opts)
    return c, baseline, out, report, opts


def test_overlay_preserves_position_path_and_derivatives(figure8):
    _, before, after, report, _ = figure8
    scale = report['time_stretch']
    assert scale > 1
    for t in np.linspace(0, before.duration, 151):
        a, b = before.sample(t), after.sample(t * scale)
        for derivative in range(3):
            assert np.allclose(a[derivative] / scale**derivative, b[derivative], atol=1e-7)


def test_continuity_limits_and_serialization(figure8):
    _, _, out, report, opts = figure8
    peaks = np.degrees(angular_peaks(out))
    assert peaks[0] <= opts.max_rate_deg * (1 + 1e-8)
    assert peaks[1] <= opts.max_accel_deg * (1 + 1e-8)
    for left, right in zip(out.segments[:-1], out.segments[1:]):
        for d in range(3):
            assert np.allclose(left.eval(left.duration, d), right.eval(0, d), atol=2e-6)
    assert abs(out.sample(0)[4]) < 1e-8
    assert abs(out.sample(out.duration)[4]) < 1e-8
    roundtrip = Trajectory.from_plain(out.to_plain())
    for t in np.linspace(0, out.duration, 30):
        a, b = out.sample(t), roundtrip.sample(t)
        assert a[3] == pytest.approx(b[3])
        assert a[4] == pytest.approx(b[4])


def test_wrap_around_shortest_arc():
    assert abs(abs(blend_angle(np.radians(179), np.radians(-179), .5)) - np.pi) < 1e-8
    assert blend_angle(0, -np.pi, .5) == pytest.approx(np.pi/2)


@pytest.mark.parametrize('approach', [0, .8])
@pytest.mark.parametrize('end_at_start', [False, True])
def test_gate_schedule_passage_and_lap_wrap(approach, end_at_start):
    c = load_course(MAPS / 'cross.yaml')
    wp, yaw = course_waypoints(c, 2, approach, end_at_start)
    tr = plan(wp, yaw)
    times, gates = gate_schedule(tr, c, 2, approach, end_at_start)
    assert [g.id for g in gates] == [g.id for g in c.gates] * 2
    assert np.all(np.diff(times) > 0)
    for t, gate in zip(times, gates):
        assert np.allclose(tr.sample(t)[0], gate.center, atol=1e-7)


def test_zero_weights_stationary_vertical_gate_and_finish():
    gates = [Gate(1, 20, 0, 1.5, 0), Gate(2, 20, 0, 4, np.pi)]
    samples = np.array([[0, 0, 0, 1.5, 0, 0, 0, 0, 0, 0],
                        [1, 20, 0, 1.5, 0, 0, 1, 0, 0, 0],
                        [3, 20, 0, 4, 0, 0, 0, 0, 0, 0]])
    yaw = desired_headings(samples, np.array([.5, 2.0]), gates, HeadingOptions())
    assert np.isfinite(yaw).all()
    assert yaw[0] == pytest.approx(0)
    assert yaw[-1] == yaw[-2]


def test_camera_projection_conventions():
    opts = HeadingOptions(camera_xyz=(0, 0, 0), camera_tilt_deg=0)
    p = np.zeros(3)
    assert np.allclose(optical_points([5, 0, 0], p, p, 0, opts), [0, 0, 5])
    assert np.allclose(optical_points([0, 5, 0], p, p, np.pi/2, opts), [0, 0, 5])
    assert np.allclose(optical_points([5, -1, 1], p, p, 0, opts), [1, -1, 5])


@pytest.mark.parametrize('name', ['figure8', 'big_track', 'cross', 'inverted_loop'])
def test_all_maps_visibility_and_unlimited_timing(name):
    c = load_course(MAPS / f'{name}.yaml')
    wp, yaw = course_waypoints(c, 1, .8)
    before = plan(wp, yaw, v_avg=3, v_max=6, a_max=8)
    after, r = perception_aware_heading(before, c, 1, .8)
    assert r['planned']['visible_fraction'] > r['baseline']['visible_fraction']
    assert np.isfinite(after.sample_all(.2)).all()
    assert r['time_stretch'] == 1.0
    assert after.duration == pytest.approx(before.duration)
    for t in np.linspace(0, before.duration, 30):
        for derivative in range(3):
            assert np.allclose(before.sample(t)[derivative], after.sample(t)[derivative], atol=1e-7)


@pytest.mark.parametrize('kwargs', [dict(d_min=8), dict(lambda_g=2), dict(sample_dt=0),
    dict(max_rate_deg=-1), dict(max_accel_deg=-1), dict(smoothing_time=0),
    dict(hfov_deg=180), dict(center_tolerance_deg=45), dict(d_max=float('nan'))])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        HeadingOptions(**kwargs)
