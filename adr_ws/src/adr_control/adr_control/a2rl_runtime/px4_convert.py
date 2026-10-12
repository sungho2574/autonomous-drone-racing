"""좌표계 변환(ENU/FLU ↔ NED/FRD) + 추력 [N] → PX4 정규화 스로틀 (numpy 전용).

변환 규칙은 성호 레포 `adr_ws/src/adr_control/adr_control/frames.py` 와 **같다**
(동등성 테스트 (E) 가 난수 입력으로 두 구현을 비교한다). 차이는 쿼터니언 순서뿐:

    frames.py        : (w, x, y, z)  ← PX4 메시지 순서
    여기 / 학습 env  : (x, y, z, w)  ← ROS geometry_msgs / roma 순서

그래서 아래 함수들은 **xyzw 를 받고 xyzw 를 돌려준다**. PX4 메시지에 넣을 때만
`xyzw_to_wxyz()` 로 바꿔 쓰면 된다.

벡터 규칙
    ENU→NED : (E, N, U) → (N, E, D) = (v[1], v[0], -v[2])   — 자기역변환
    FLU→FRD : (전, 좌, 상) → (전, 우, 하) = (x, -y, -z)      — x 축 180° 회전
    각속도  : (p, q, r)_FLU → (p, -q, -r)_FRD                 — 위와 같은 회전
    요각    : yaw_NED = wrap(π/2 - yaw_ENU)

추력 → 스로틀은 두 모델을 제공한다 (아래 함수 주석 참고).
"""
from __future__ import annotations

import numpy as np

# frames.py 와 같은 상수 (wxyz)
_Q_ENU_TO_NED_WXYZ = np.array([0.0, np.sqrt(0.5), np.sqrt(0.5), 0.0])
_Q_FLU_TO_FRD_WXYZ = np.array([0.0, 1.0, 0.0, 0.0])

# racer_spec.yaml (adr_sim/config) 의 값 — SITL 기체
KF_DEFAULT = 1.4e-6          # motor_constant [N/(rad/s)^2]
OMEGA_MIN = 300.0            # px4.ec_min
OMEGA_MAX = 3000.0           # motor.max_rot_velocity
THR_HOVER_SPEC = 0.31        # px4.thr_hover (참고값)
N_ROTOR = 4
G = 9.81


# ------------------------------------------------------------------ 쿼터니언

def xyzw_to_wxyz(q):
    q = np.asarray(q, dtype=np.float64)
    return np.concatenate([q[..., 3:4], q[..., 0:3]], axis=-1)


def wxyz_to_xyzw(q):
    q = np.asarray(q, dtype=np.float64)
    return np.concatenate([q[..., 1:4], q[..., 0:1]], axis=-1)


def _qmul_wxyz(a, b):
    """해밀턴 곱 (wxyz). frames.py::quat_mul 과 같은 식."""
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def _conj_wxyz(q):
    return np.concatenate([q[..., 0:1], -q[..., 1:4]], axis=-1)


def quat_enu_flu_to_ned_frd(q_xyzw):
    """ENU/FLU 자세 쿼터니언 (xyzw) → NED/FRD (xyzw).

    frames.py: q_ned_frd = Q_ENU_TO_NED ⊗ q_enu_flu ⊗ Q_FLU_TO_FRD
    """
    q = xyzw_to_wxyz(q_xyzw)
    out = _qmul_wxyz(_qmul_wxyz(np.broadcast_to(_Q_ENU_TO_NED_WXYZ, q.shape), q),
                     np.broadcast_to(_Q_FLU_TO_FRD_WXYZ, q.shape))
    return wxyz_to_xyzw(out)


def quat_ned_frd_to_enu_flu(q_xyzw):
    """위의 역변환. PX4 `VehicleAttitude.q`(wxyz) 를 받아 쓸 때 필요하다."""
    q = xyzw_to_wxyz(q_xyzw)
    qen = np.broadcast_to(_Q_ENU_TO_NED_WXYZ, q.shape)
    qfd = np.broadcast_to(_Q_FLU_TO_FRD_WXYZ, q.shape)
    out = _qmul_wxyz(_qmul_wxyz(_conj_wxyz(qen), q), _conj_wxyz(qfd))
    return wxyz_to_xyzw(out)


# ------------------------------------------------------------------ 벡터 / 각

def enu_to_ned(v):
    """위치·속도 ENU → NED. 자기역변환이라 NED→ENU 에도 같은 함수를 쓴다."""
    v = np.asarray(v, dtype=np.float64)
    return np.stack([v[..., 1], v[..., 0], -v[..., 2]], axis=-1)


ned_to_enu = enu_to_ned


def body_rates_flu_to_frd(w):
    """(p, q, r)_FLU → (p, -q, -r)_FRD. VehicleRatesSetpoint 에 바로 넣을 순서."""
    w = np.asarray(w, dtype=np.float64)
    return np.stack([w[..., 0], -w[..., 1], -w[..., 2]], axis=-1)


body_rates_frd_to_flu = body_rates_flu_to_frd


def yaw_enu_to_ned(yaw):
    return (np.pi / 2.0 - np.asarray(yaw, dtype=np.float64) + np.pi) % (2 * np.pi) - np.pi


yaw_ned_to_enu = yaw_enu_to_ned


def yaw_rate_enu_to_ned(r):
    """요 레이트 부호만 뒤집힌다 (frames.py 와 동일)."""
    return -np.asarray(r, dtype=np.float64)


# ------------------------------------------------------------- 추력 → 스로틀

def throttle_sitl(thrust_n, kf=KF_DEFAULT, omega_min=OMEGA_MIN,
                  omega_max=OMEGA_MAX, n_rotor=N_ROTOR):
    """SITL(racer_spec) 정확식.

    Gazebo 모터 모델은 로터 하나당 T_i = kf·ω², 그리고 PX4 출력 u 는 ω 에 **선형**이다
    (ω = ω_min + u·(ω_max - ω_min)). 집합추력 T 를 4 로 나눠 ω 를 구하고 되돌린다.

        u = (sqrt(T / (n·kf)) - ω_min) / (ω_max - ω_min)

    racer_spec 값(kf 1.4e-6, ω 300~3000)이면 분모가 2700 이고,
    T = 0.75·9.81 N (호버) → u ≈ 0.31 (= px4.thr_hover) 로 맞는다.
    """
    t = np.maximum(np.asarray(thrust_n, dtype=np.float64), 0.0)
    omega = np.sqrt(t / (n_rotor * kf))
    return np.clip((omega - omega_min) / (omega_max - omega_min), 0.0, 1.0)


def throttle_hover_calibrated(thrust_n, thr_hover, mass_kg, omega_min_frac=0.0):
    """실기체 근사식 — **호버 스로틀 하나만** 재서 보정한다.

    왜 SITL 식을 그대로 못 쓰나: 실기체의 ω_min/ω_max/kf 는 배터리 전압·프롭·ESC 에
    따라 달라지고 측정이 번거롭다. 반면 "정지 비행할 때 로그에 찍힌 스로틀"
    (`thr_hover`, PX4 `MPC_THR_HOVER` 또는 `actuator_motors` 평균) 은 한 번 띄우면 얻는다.

    가정: 추력이 스로틀의 제곱에 비례한다 (T ≈ c·u², PX4 기본 모델 `THR_MDL_FAC=0`
    에서 실제로 쓰는 가정이다. 프로펠러 추력 ∝ ω², 출력 ∝ ω).

        T_hover = m·g  에서  c = m·g / thr_hover²
        u = thr_hover · sqrt(T / (m·g))

    `omega_min_frac` 을 주면 아이들 오프셋(ω_min 에 해당하는 스로틀)을 반영한
    조금 더 정확한 형태를 쓴다 — u = u0 + (thr_hover-u0)·sqrt(T/(m·g)),  u0 = omega_min_frac.
    기본 0 이면 위 단순식과 같다.

    한계: 배터리 처짐(같은 u 에 추력 감소)은 보정하지 않는다. 비행 중 전압이 떨어지면
    같은 a0 에 실제 가속도가 줄어든다 — 학습에서 추력 DR(k_eta ±18%)로 겪게 한 범위 안이다.
    """
    t = np.maximum(np.asarray(thrust_n, dtype=np.float64), 0.0)
    t_hover = float(mass_kg) * G
    u0 = float(omega_min_frac)
    return np.clip(u0 + (float(thr_hover) - u0) * np.sqrt(t / t_hover), 0.0, 1.0)


def throttle(thrust_n, mode="sitl", **kw):
    """`mode='sitl'` 또는 `'hover'`. policy_runner 가 이 함수를 쓴다."""
    if mode == "sitl":
        return throttle_sitl(thrust_n, **kw)
    if mode == "hover":
        return throttle_hover_calibrated(thrust_n, **kw)
    raise ValueError(f"모르는 스로틀 모드: {mode}")
