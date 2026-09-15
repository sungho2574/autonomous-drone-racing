import os

import numpy as np
import pytest

from adr_planning.course import course_waypoints, load_course
from adr_planning.min_snap import Trajectory, plan

GATES = os.path.join(os.path.dirname(__file__), '..', '..', 'adr_bringup', 'config', 'gates.yaml')


def _square():
    wp = np.array([[0, 0, 1], [2, 0, 1], [2, 2, 1], [0, 2, 1], [0, 0, 1.0]])
    yaw = np.array([0, np.pi / 2, np.pi, -np.pi / 2, 0.0])
    return wp, yaw


def test_passes_waypoints():
    wp, yaw = _square()
    tr = plan(wp, yaw, v_avg=1.0)
    t = 0.0
    for i, seg in enumerate(tr.segments):
        p, *_ = tr.sample(t)
        assert np.allclose(p, wp[i], atol=1e-6)
        t += seg.duration
    p, *_ = tr.sample(tr.duration)
    assert np.allclose(p, wp[-1], atol=1e-6)


def test_rest_to_rest():
    wp, yaw = _square()
    tr = plan(wp, yaw, v_avg=1.0)
    for t in (0.0, tr.duration):
        _, v, a, _, yr = tr.sample(t)
        assert np.allclose(v, 0, atol=1e-6)
        assert np.allclose(a, 0, atol=1e-6)
        assert abs(yr) < 1e-6


def test_continuity_at_junctions():
    wp, yaw = _square()
    tr = plan(wp, yaw, v_avg=1.0)
    t = 0.0
    for seg in tr.segments[:-1]:
        t += seg.duration
        before = tr.sample(t - 1e-7)
        after = tr.sample(t + 1e-7)
        for k in range(3):   # pos, vel, acc
            assert np.allclose(before[k], after[k], atol=1e-4)
        assert abs(before[3] - after[3]) < 1e-4


def test_limits_scale_time():
    wp, yaw = _square()
    fast = plan(wp, yaw, v_avg=5.0)
    limited = plan(wp, yaw, v_avg=5.0, v_max=1.0, a_max=1.0)
    v, a = limited.peak()
    assert limited.duration > fast.duration
    assert v <= 1.0 * 1.01
    assert a <= 1.0 * 1.01


def test_yaw_unwrapped():
    wp = np.array([[0, 0, 1], [1, 0, 1], [2, 0, 1.0]])
    yaw = np.array([3.0, -3.0, 2.8])            # ±π 를 넘는 점프 → unwrap 으로 짧은 경로
    tr = plan(wp, yaw, v_avg=1.0)
    _, _, _, y_mid, _ = tr.sample(tr.segments[0].duration)
    assert abs(y_mid - (2 * np.pi - 3.0)) < 1e-6


def test_plain_roundtrip():
    wp, yaw = _square()
    tr = plan(wp, yaw, v_avg=1.0)
    tr2 = Trajectory.from_plain(tr.to_plain())
    for t in np.linspace(0, tr.duration, 20):
        a, b = tr.sample(t), tr2.sample(t)
        assert np.allclose(a[0], b[0]) and abs(a[3] - b[3]) < 1e-12


def test_course_file_plan():
    course = load_course(GATES)
    wp, yaw = course_waypoints(course, laps=1, approach_dist=0.8)
    assert wp.shape == (1 + 4 * 3 + 1, 3)
    tr = plan(wp, yaw, v_avg=3.0, v_max=6.0, a_max=8.0)
    # 게이트 중심 통과 시 속도가 게이트 법선과 정렬(cos > 0.95)
    t = 0.0
    for i, seg in enumerate(tr.segments):
        if i % 3 == 2 and i < len(tr.segments) - 1:
            _, v, _, _, _ = tr.sample(t)
            n = np.array([np.cos(yaw[i]), np.sin(yaw[i]), 0])
            assert v @ n / np.linalg.norm(v) > 0.95
        t += seg.duration


def test_bad_input():
    with pytest.raises(ValueError):
        plan(np.zeros((1, 3)), np.zeros(1))
