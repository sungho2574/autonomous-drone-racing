"""정책 액션 4 개 → 집합추력 [N] + 동체 각속도 명령 [rad/s] (numpy 전용).

학습 env `racing_env/racer_env.py::_control` 과 같은 식이고, 추력 식은
`racer_params.py::thrust_from_action` 을 **그대로** 옮긴 것이다.

    a[0] → 추력   T = m_nom · (G + (a_max_g-1)·G·clip(a0,0,1) + G·clip(a0,-1,0))
                  a0 = -1 → 0 g,  a0 = 0 → 1 g(호버),  a0 = +1 → a_max_g g (= 4 g)
    a[1] → 롤 레이트  p = a1 · 10 rad/s   (FLU, +x 전방 축 기준 오른쪽 감기)
    a[2] → 피치 레이트 q = a2 · 10 rad/s  (FLU, +y 좌측 축 기준 = 기수 들림 +)
    a[3] → 요 레이트   r = a3 · 7 rad/s   (FLU, +z 상방 축 기준 반시계)

중요 — **공칭 질량 0.80 kg 고정**이다. 실제 기체 질량으로 바꾸면 안 된다.
학습에서 정책은 "공칭 질량 기준 추력 명령"을 내보내고 실제 추력 오차(질량·k_eta)는
도메인 랜덤화로 겪게 했다. 여기서 실제 질량을 넣으면 학습 때와 다른 스케일이 된다.
(실기체 호버 스로틀 보정은 `px4_convert.py` 쪽에서 한다.)

a 는 반드시 **클립된 값**([-1,1])을 쓴다. 정책 그래프의 두 번째 출력(`action`)이
이미 클립된 값이고, 그 값이 다음 스텝 관측의 `u_last` 로도 들어간다.
"""
from __future__ import annotations

import numpy as np

G = 9.81                      # racer_params.G 와 같은 값
M_NOM = 0.80                  # airframe.mass_nominal (학습 config)
A_MAX_G = 4.0                 # action.a_max_g
MAX_BODY_RATE = (10.0, 10.0, 7.0)   # action.max_body_rate [rad/s]


def thrust_from_action(a0, mass=M_NOM, a_max_g=A_MAX_G):
    """racer_params.thrust_from_action 과 같은 식 (numpy)."""
    a0 = np.asarray(a0, dtype=np.float64)
    up = (a_max_g - 1.0) * G
    acc = G + up * np.clip(a0, 0.0, 1.0) + G * np.clip(a0, -1.0, 0.0)
    return mass * acc


class ActionMapper:
    def __init__(self, m_nom=M_NOM, a_max_g=A_MAX_G, max_body_rate=MAX_BODY_RATE):
        self.m_nom = float(m_nom)
        self.a_max_g = float(a_max_g)
        self.max_body_rate = np.asarray(max_body_rate, dtype=np.float64)
        self.hover_thrust_n = self.m_nom * G       # a0 = 0 일 때의 추력

    def map(self, a):
        """(4,) 클립된 액션 → (추력 [N], 각속도 명령 (3,) [rad/s] FLU)."""
        a = np.clip(np.asarray(a, dtype=np.float64).reshape(-1), -1.0, 1.0)
        thrust = float(thrust_from_action(a[0], self.m_nom, self.a_max_g))
        rates = a[1:4] * self.max_body_rate
        return thrust, rates

    def batch(self, acts):
        """(N,4) → (추력 (N,), 각속도 (N,3))."""
        a = np.clip(np.asarray(acts, dtype=np.float64), -1.0, 1.0)
        return (thrust_from_action(a[:, 0], self.m_nom, self.a_max_g),
                a[:, 1:4] * self.max_body_rate)

    def action0_for_thrust(self, thrust_n):
        """역함수 — 주어진 추력이 나오는 a0. 검증·분석용."""
        r = float(thrust_n) / (self.m_nom * G)
        return (r - 1.0) / (self.a_max_g - 1.0) if r >= 1.0 else r - 1.0
