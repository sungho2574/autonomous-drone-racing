import numpy as np
import pytest

from adr_control import frames as F


def test_vector_roundtrip():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(F.enu_to_ned(v), [2.0, 1.0, -3.0])
    assert np.allclose(F.ned_to_enu(F.enu_to_ned(v)), v)


@pytest.mark.parametrize('yaw', np.linspace(-np.pi, np.pi, 13))
def test_yaw_roundtrip(yaw):
    assert abs(F.wrap_pi(F.yaw_ned_to_enu(F.yaw_enu_to_ned(yaw)) - yaw)) < 1e-12


def test_yaw_cardinal():
    assert abs(F.yaw_enu_to_ned(0.0) - np.pi / 2) < 1e-12          # East → NED yaw +90°
    assert abs(F.yaw_enu_to_ned(np.pi / 2)) < 1e-12                  # North → 0
    assert abs(F.wrap_pi(F.yaw_enu_to_ned(np.pi) + np.pi / 2)) < 1e-12   # West → -90°


@pytest.mark.parametrize('yaw', [0.0, 0.7, -2.0, 3.0])
def test_quat_yaw_consistency(yaw):
    """ENU yaw 만 있는 자세를 PX4 로 바꾸면 NED yaw 와 일치해야 한다."""
    q_ros = F.quat_from_yaw(yaw)
    q_px4 = F.quat_enu_flu_to_ned_frd(q_ros)
    assert abs(F.wrap_pi(F.yaw_from_quat(q_px4) - F.yaw_enu_to_ned(yaw))) < 1e-9
    back = F.quat_ned_frd_to_enu_flu(q_px4)
    # 부호 모호성(q ≡ -q) 허용
    assert np.allclose(back, q_ros, atol=1e-9) or np.allclose(back, -q_ros, atol=1e-9)


def test_quat_unit():
    q = F.quat_enu_flu_to_ned_frd(F.quat_from_yaw(1.0))
    assert abs(np.linalg.norm(q) - 1) < 1e-12
