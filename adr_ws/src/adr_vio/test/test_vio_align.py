"""ROS 없이 정렬 수학만 검사. rclpy/tf2_ros 는 스텁으로 대체한다."""
import math
import sys
import types

import numpy as np


def _stub_ros():
    for name in ('rclpy', 'rclpy.node', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
                 'nav_msgs.msg', 'std_msgs', 'std_msgs.msg', 'tf2_ros', 'tf2_ros.static_transform_broadcaster'):
        sys.modules.setdefault(name, types.ModuleType(name))

    class Dummy:
        def __init__(self, *a, **k):
            pass
    sys.modules['rclpy.node'].Node = object
    for mod, names in (('geometry_msgs.msg', ('PoseStamped', 'TransformStamped')),
                       ('nav_msgs.msg', ('Odometry', 'Path')),
                       ('std_msgs.msg', ('Float32',)),
                       ('tf2_ros', ('Buffer', 'TransformBroadcaster', 'TransformListener')),
                       ('tf2_ros.static_transform_broadcaster', ('StaticTransformBroadcaster',))):
        for n in names:
            setattr(sys.modules[mod], n, Dummy)


_stub_ros()
from adr_vio.vio_align import R_to_quat, Rz, quat_to_R, yaw_of  # noqa: E402


def test_quat_roundtrip():
    for yaw, pitch in ((0.3, 0.0), (-2.0, 0.2), (3.0, -0.4)):
        R = Rz(yaw) @ np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],
                                [-math.sin(pitch), 0, math.cos(pitch)]])
        q = R_to_quat(R)
        assert np.allclose(quat_to_R(q), R, atol=1e-9)
        assert abs(np.linalg.norm(q) - 1) < 1e-12


def test_quat_to_R_180deg():
    """trace < 0 분기 (180° 회전) 도 정확해야 한다."""
    for axis in range(3):
        R = -np.eye(3)
        R[axis, axis] = 1.0          # 해당 축 기준 180°
        assert np.allclose(quat_to_R(R_to_quat(R)), R, atol=1e-9)


def test_yaw_of():
    assert abs(yaw_of(Rz(0.7)) - 0.7) < 1e-12


def _align_yaw(R_mb, t_mb, R_gi, t_gi):
    """vio_align._align 의 yaw 모드와 같은 계산."""
    R_mg = Rz(yaw_of(R_mb) - yaw_of(R_gi))
    return R_mg, t_mb - R_mg @ t_gi


def test_alignment_maps_first_pose_onto_reference():
    """정렬 직후 VIO 포즈는 기준 포즈와 겹쳐야 한다 (OpenVINS 는 원점 yaw=0 에서 시작)."""
    R_mb, t_mb = Rz(2.356), np.array([2.83, -2.83, 0.05])    # 기준: start 지점, 135° 방향
    R_gi, t_gi = np.eye(3), np.zeros(3)                       # VIO: 자기 원점
    R_mg, t_mg = _align_yaw(R_mb, t_mb, R_gi, t_gi)
    assert np.allclose(R_mg @ t_gi + t_mg, t_mb)
    assert abs(yaw_of(R_mg @ R_gi) - yaw_of(R_mb)) < 1e-12


def test_alignment_preserves_relative_motion():
    """정렬은 강체변환이라 VIO 가 움직인 거리는 map 에서도 같아야 한다."""
    R_mb, t_mb = Rz(-1.0), np.array([1.0, 2.0, 1.5])
    R_mg, t_mg = _align_yaw(R_mb, t_mb, Rz(0.0), np.zeros(3))
    a, b = np.array([1.0, 0.0, 0.0]), np.array([3.0, 1.0, -0.5])
    d_vio = np.linalg.norm(b - a)
    d_map = np.linalg.norm((R_mg @ b + t_mg) - (R_mg @ a + t_mg))
    assert abs(d_vio - d_map) < 1e-12


def test_yaw_mode_keeps_z_axis_up():
    """yaw 모드는 기울기를 안 건드린다 — VIO 의 중력 정렬 오차가 map 에서 그대로 보여야 한다."""
    R_mb = Rz(0.5) @ np.array([[1, 0, 0], [0, math.cos(0.1), -math.sin(0.1)],
                               [0, math.sin(0.1), math.cos(0.1)]])   # 기준이 롤 0.1 rad 기울어짐
    R_mg, _ = _align_yaw(R_mb, np.zeros(3), np.eye(3), np.zeros(3))
    assert np.allclose(R_mg[:, 2], [0, 0, 1], atol=1e-12)            # map z 축 유지
