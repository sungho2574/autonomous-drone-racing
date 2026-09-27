"""VIO 병진 drift 를 게이트 PnP 로 보정하는 칼만 필터 (ROS 비의존, numpy 만).

출처: *Drift-Corrected Monocular VIO and Perception-Aware Planning for Autonomous Drone Racing*
(Azhari et al., KAIST, A2RL×DCL 2025) §2.4 — docs/kaist_drone_racing_ros2_spec.md 참고.

핵심 설계 두 가지를 그대로 따른다.
  1) 전체 상태가 아니라 **drift 상태만** 추정한다: x_d = [p_d, v_d] ∈ R^6.
     VIO 는 고빈도·국소적으로 정확하지만 서서히 흘러간다. 게이트는 저빈도지만 전역으로 정확하다.
     그 '흘러간 양'만 상태로 두면 필터가 6차원으로 끝나고, VIO 의 좋은 성질을 안 망친다.
  2) PnP 의 **회전은 버리고 위치만** 측정값으로 쓴다 (§2.2). 코너 노이즈에 회전이 훨씬 취약하다.
     → 보정 결과도 위치만 바뀌고 자세·속도는 VIO 값을 그대로 쓴다.

모델
    예측: x⁻ = F x⁺,           F = [[I, Δt·I], [0, I]]        (등속 drift)
          P⁻ = F P⁺ Fᵀ + Q,    Q = diag(σ_p·I, σ_v·I)·Δt
    갱신: z = p_pnp (map 기준 기체 위치),  H = [I, 0]
          y = z − p_vio                    ← 잔차는 'VIO 가 얼마나 틀렸나' 그 자체
          K = P⁻Hᵀ(H P⁻ Hᵀ + R)⁻¹
          x⁺ = x⁻ + K(y − H x⁻),  P⁺ = (I − K H) P⁻
          R = λ_r · diag(σ_r)              ← λ_r 은 검출 품질로 스케일

논문 Q 는 Δt 곱이 없지만 여기서는 곱한다. VIO 가 30~250 Hz 로 들어오는데 Δt 를 안 곱하면
프로세스 노이즈가 입력 주기에 따라 달라져 같은 σ 값이 다른 뜻이 되어버린다.
"""
from __future__ import annotations

import numpy as np


class DriftKF:
    """x = [p_d(3), v_d(3)]. 단위는 전부 m, m/s."""

    def __init__(self, sigma_p: float = 0.1, sigma_v: float = 0.2,
                 sigma_r=(0.25, 0.25, 0.25), lambda_r: float = 1.0,
                 p0_pos: float = 1.0, p0_vel: float = 1.0, v_max: float = 0.5):
        self.x = np.zeros(6)
        self.P = np.diag([p0_pos] * 3 + [p0_vel] * 3).astype(float)
        self.sigma_p = float(sigma_p)
        self.sigma_v = float(sigma_v)
        self.sigma_r = np.asarray(sigma_r, dtype=float).reshape(3)
        self.lambda_r = float(lambda_r)
        # drift 속도 상한 [m/s]. 측정이 드문드문 들어오면 v_d 가 과하게 잡힐 수 있어
        # hold 로 들어가기 전에도 외삽이 폭주하지 않게 막아 둔다.
        self.v_max = float(v_max)
        self.n_update = 0

    # ------------------------------------------------------------------
    @property
    def p_d(self) -> np.ndarray:
        """위치 drift 추정값."""
        return self.x[:3]

    @property
    def v_d(self) -> np.ndarray:
        return self.x[3:]

    def predict(self, dt: float, hold: bool = False):
        """hold=True 면 등속 항을 끄고 p_d 를 그대로 둔다 (불확실성 P 는 계속 키운다).

        논문에는 없는 처리다. 논문은 레이싱 중 게이트가 늘 보인다고 보지만, 여기서는
        측정이 끊기는 구간이 생긴다(게이트가 시야 밖, 미션 종료 후 호버 등).
        그때 등속 모델은 v_d 를 무한정 적분해 drift 추정이 발산한다 — 실측으로
        측정이 끊긴 뒤 p_d.z 가 2 → 11 m 까지 자라는 걸 확인했다.
        정보가 없을 때의 최선의 추정은 '마지막 drift 를 유지'다.
        """
        if not (dt > 0.0):
            return
        F = np.eye(6)
        if not hold:
            F[:3, 3:] = dt * np.eye(3)
        Q = np.diag([self.sigma_p] * 3 + [self.sigma_v] * 3) * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, p_pnp, p_vio, quality: float = 1.0):
        """게이트 PnP 위치 1건으로 갱신.

        p_pnp : map 기준 기체 위치 (PnP)
        p_vio : 같은 시각의 VIO 위치 (map 기준, 보정 전)
        quality: 0~1. 낮을수록 R 을 키워 덜 믿는다.
        """
        y = np.asarray(p_pnp, dtype=float).reshape(3) - np.asarray(p_vio, dtype=float).reshape(3)
        H = np.zeros((3, 6))
        H[:, :3] = np.eye(3)
        # 품질이 낮을수록 분산을 키운다. quality=1 → λ_r, quality→0 → λ_r/0.05 (20배)
        scale = self.lambda_r / max(float(quality), 0.05)
        R = np.diag(self.sigma_r ** 2) * scale
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (y - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        n = float(np.linalg.norm(self.x[3:]))
        if n > self.v_max:                       # 속도 상한
            self.x[3:] *= self.v_max / n
        self.n_update += 1
        return y

    def correct(self, p_vio) -> np.ndarray:
        """보정된 위치 p_c = p_v + p_d (논문 §2.4 마지막 식)."""
        return np.asarray(p_vio, dtype=float).reshape(3) + self.p_d
