from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from adr_perception.association import PoseBuffer, MatchConfig, project_gates, associate
from adr_perception.pnp import camera_extrinsic, drone_pose_in_map, order_quad
from adr_planning.course import load_course

K = np.array([[320.0, 0, 320], [0, 320, 240], [0, 0, 1]])
D = np.zeros(5)


def gate(gid):
    return SimpleNamespace(id=gid)


def quad(x=100.0, size=80.0):
    return np.array(
        [[x, 100], [x + size, 100], [x + size, 100 + size], [x, 100 + size]]
    )


def test_selects_six_independent_of_route_or_area_order():
    projected = [(gate(4), quad(100)), (gate(6), quad(240))]
    m = associate([quad(240) + 1], projected, MatchConfig())
    assert m["gate"].id == 6
    assert m["reason"] == "accepted"
    assert m["second"]["gate"].id == 4


def test_ambiguous_and_absolute_rejection():
    m = associate(
        [quad(113)], [(gate(4), quad(100)), (gate(6), quad(127))], MatchConfig()
    )
    assert m["reason"] == "absolute_error"
    m = associate(
        [quad(106)], [(gate(4), quad(100)), (gate(6), quad(113))], MatchConfig()
    )
    assert m["reason"] == "ambiguous"
    m = associate(
        [quad(102)], [(gate(4), quad(100)), (gate(6), quad(106))], MatchConfig()
    )
    assert m["reason"] == "ambiguous"  # ratio passes, 5 px margin does not
    m = associate(
        [quad(100)], [(gate(4), quad(100)), (gate(6), quad(100))], MatchConfig()
    )
    assert m["reason"] == "ambiguous"  # zero/zero must never win


def test_single_bad_corner_and_duplicate_detection():
    q = quad()
    q[0] += [20, 0]
    m = associate([q], [(gate(4), quad())], MatchConfig())
    assert m["rmse"] == 10
    assert m["reason"] == "absolute_error"  # RMSE passes; maximum corner fails
    m = associate([quad(), quad() + 1], [(gate(4), quad())], MatchConfig())
    assert m["reason"] == "duplicate_id"
    assert associate([quad()], [], MatchConfig()) is None


def test_cyclic_corner_order():
    q = quad()
    m = associate([np.roll(q, 2, axis=0)], [(gate(6), q)], MatchConfig())
    np.testing.assert_allclose(m["corners"], q)
    assert m["reason"] == "accepted"


def test_pose_time_interpolation_no_extrapolation_or_stale_gap():
    b = PoseBuffer()
    b.add(1, [0, 0, 0], Rotation.from_euler("z", 170, degrees=True).as_quat())
    b.add(1.1, [2, 0, 0], Rotation.from_euler("z", -170, degrees=True).as_quat())
    T = b.sample(1.05)
    np.testing.assert_allclose(T[:3, 3], [1, 0, 0])
    np.testing.assert_allclose(T[:2, :2], -np.eye(2), atol=1e-12)
    assert b.sample(0.9) is None and b.sample(1.2) is None
    assert b.sample(1) is not None
    b.add(2, [3, 0, 0], [0, 0, 0, 1])
    assert b.sample(1.5) is None


def test_big_track_six_observed_while_approaching_four():
    root = Path(__file__).resolve().parents[4]
    course = load_course(
        str(root / "adr_ws/src/adr_bringup/config/maps/big_track.yaml")
    )
    T = np.eye(4)
    T[:3, 3] = [3.3, 5.7, 1.25]
    T[:3, :3] = Rotation.from_euler("z", -30, degrees=True).as_matrix()
    ext = camera_extrinsic([0.045, 0, 0.022], 20)
    projected = project_gates(course.gates, course.inner_size, T, ext, K, D, 640, 480)
    assert {4, 6} <= {g.id for g, _ in projected}
    six = next(uv for g, uv in projected if g.id == 6)
    m = associate([order_quad(six)], projected, MatchConfig())
    assert m["reason"] == "accepted" and m["gate"].id == 6
    estimated, err = drone_pose_in_map(
        m["corners"], course.inner_size, K, D, m["gate"].center, m["gate"].yaw, ext
    )
    np.testing.assert_allclose(estimated[:3, 3], T[:3, 3], atol=1e-5)
    assert err < 1e-5
    # Looking away must not project points from behind the camera.
    T[:3, :3] = Rotation.from_euler("z", 150, degrees=True).as_matrix()
    assert 6 not in {
        g.id
        for g, _ in project_gates(
            course.gates, course.inner_size, T, ext, K, D, 640, 480
        )
    }
