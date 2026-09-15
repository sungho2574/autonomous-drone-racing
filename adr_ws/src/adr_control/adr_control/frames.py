"""ENU(ROS, map) ↔ NED(PX4) 좌표 변환 유틸 (ROS 비의존).

- 위치/속도/가속도 벡터: ENU (x_e, y_n, z_u) ↔ NED (x_n, y_e, z_d)  →  (e, n, u) ↔ (n, e, -u)
- yaw: ENU 는 +x(East) 기준 CCW, NED 는 +x(North) 기준 CW(위에서 봤을 때) →  yaw_ned = π/2 − yaw_enu
- 쿼터니언: body FLU→ENU (ROS)  ↔  body FRD→NED (PX4)
    q_ned_frd = q_ENU→NED ⊗ q_enu_flu ⊗ q_FLU→FRD
  PX4 gz_bridge 와 동일한 상수: q_ENU→NED = (0, √½, √½, 0), q_FLU→FRD = (0, 1, 0, 0)  (w, x, y, z)
"""
from __future__ import annotations

from math import pi

import numpy as np

_S = np.sqrt(0.5)
Q_ENU_TO_NED = np.array([0.0, _S, _S, 0.0])   # w, x, y, z
Q_FLU_TO_FRD = np.array([0.0, 1.0, 0.0, 0.0])


def enu_to_ned(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return np.array([v[1], v[0], -v[2]])


def ned_to_enu(v) -> np.ndarray:
    return enu_to_ned(v)   # 대합 변환(involution)


def wrap_pi(a: float) -> float:
    return (a + pi) % (2 * pi) - pi


def yaw_enu_to_ned(yaw: float) -> float:
    return wrap_pi(pi / 2 - yaw)


def yaw_ned_to_enu(yaw: float) -> float:
    return wrap_pi(pi / 2 - yaw)


def yaw_rate_enu_to_ned(r: float) -> float:
    return -r


def quat_mul(a, b) -> np.ndarray:
    """Hamilton 곱, (w, x, y, z)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_enu_flu_to_ned_frd(q) -> np.ndarray:
    """ROS 자세(body FLU → ENU, wxyz) → PX4 자세(body FRD → NED, wxyz)."""
    return quat_mul(quat_mul(Q_ENU_TO_NED, np.asarray(q, dtype=float)), Q_FLU_TO_FRD)


def quat_ned_frd_to_enu_flu(q) -> np.ndarray:
    return quat_mul(quat_mul(quat_conj(Q_ENU_TO_NED), np.asarray(q, dtype=float)),
                    quat_conj(Q_FLU_TO_FRD))


def quat_from_yaw(yaw: float) -> np.ndarray:
    """z축 회전 쿼터니언 (wxyz)."""
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def yaw_from_quat(q) -> float:
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
