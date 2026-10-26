"""목표 게이트 추적 + 통과/역통과/프레임충돌/이탈 판정 (numpy 전용).

학습 env `racing_env/track.py` 와 **같은 로직**이다. 공용 패널(한 판에 구멍 2 개,
A2RL t4) 도 포함한다. 차이는 호출 주기 하나뿐이다:

  - 학습: 물리 서브스텝 0.01 s 마다 (prev_pos → curr_pos) 판정, 정책 스텝당 2 회
  - 런타임: 정책 스텝 0.02 s 마다 1 회

판정이 **선분-평면 해석 교차**라 구간이 길어도 터널링은 없다. 다만 0.02 s 안에서
게이트를 두 장 지나가면(현실적으로 거의 없음) 한 장만 센다. 동등성 테스트 (C)
(`racing_env/export_policy.py`) 가 실제 롤아웃에서 불일치 개수를 세어 보고한다.

쓰는 쪽:
    gt = GateTracker(mp)                 # 목표 = 맵 첫 게이트(인덱스 0)
    ev = gt.update(prev_pos, curr_pos)   # 매 제어 주기 1 회
    gt.target                            # 다음 관측에 쓸 목표 게이트 인덱스
"""
from __future__ import annotations

import numpy as np

from .obs_builder import MapGeom


def _crossing(prev_l, curr_l):
    """선분이 게이트 평면(로컬 x=0)을 지났는지 + 교차점 (y,z).

    정방향 조건 `prev_x < 0 <= curr_x` 는 학습 코드와 글자 그대로 같다.
    prev_l / curr_l: (...,3) 게이트 로컬 좌표.
    """
    px, cx = prev_l[..., 0], curr_l[..., 0]
    fwd = (px < 0.0) & (cx >= 0.0)
    rev = (cx < 0.0) & (px >= 0.0)
    cross = fwd | rev
    denom = np.where(cross, cx - px, 1.0)
    t = np.clip(-px / denom, 0.0, 1.0)
    cy = prev_l[..., 1] + t * (curr_l[..., 1] - prev_l[..., 1])
    cz = prev_l[..., 2] + t * (curr_l[..., 2] - prev_l[..., 2])
    return fwd, rev, cy, cz


class GateTracker:
    def __init__(self, mp: MapGeom, target=0):
        self.mp = mp
        self.reset(target)

    def reset(self, target=0):
        self.target = int(target)
        self.gates_passed = 0
        self.laps = 0                 # 마지막 게이트를 지나 1 번으로 되감긴 횟수
        self.pass_log = []            # (게이트 id, 중심오차) 기록

    # ------------------------------------------------------------- 좌표 변환
    def to_local(self, idx, p):
        return self.mp.gate_RT[idx] @ (np.asarray(p, dtype=np.float64)
                                       - self.mp.gate_pos[idx])

    # ------------------------------------------------------------- 개별 판정
    def target_pass(self, prev_pos, curr_pos):
        """목표 게이트 정/역 통과. → (passed, reverse, offset)."""
        i = self.target
        pl, cl = self.to_local(i, prev_pos), self.to_local(i, curr_pos)
        fwd, rev, cy, cz = _crossing(pl, cl)
        inside = max(abs(float(cy)), abs(float(cz))) <= self.mp.pass_half
        passed = bool(fwd) and inside
        reverse = bool(rev) and inside and not passed
        offset = float(np.hypot(cy, cz)) if passed else 0.0
        return passed, reverse, offset

    def out_of_bounds(self, pos):
        pos = np.asarray(pos, dtype=np.float64)
        return bool(((pos < self.mp.bounds_lo) | (pos > self.mp.bounds_hi)).any())

    # ------------------------------------------------------------- 한 스텝
    def update(self, prev_pos, curr_pos):
        """제어 주기 1 회분. 통과하면 목표를 다음 게이트로 넘긴다.

        반환 dict:
            passed       목표 게이트 정방향 통과
            reverse      역방향 통과 (학습에서는 **즉시 종료** 사유)
            offset       통과 시 중심 오차 ‖(y,z)‖ [m]
            out_of_bounds코스 박스 이탈 (학습에서는 추락)
            lap_done     이 스텝에서 마지막 게이트를 통과해 1 번으로 되감겼는지
            target       갱신된 목표 게이트 인덱스
            gate_id      갱신된 목표 게이트의 맵 yaml id
        """
        passed, reverse, offset = self.target_pass(prev_pos, curr_pos)
        oob = self.out_of_bounds(curr_pos)
        lap_done = False
        if passed:
            self.pass_log.append((self.mp.gate_ids[self.target], offset))
            self.gates_passed += 1
            lap_done = self.target == self.mp.num_gates - 1
            if lap_done:
                self.laps += 1
            self.target = (self.target + 1) % self.mp.num_gates
        return dict(passed=passed, reverse=reverse, offset=offset,
                    out_of_bounds=oob, lap_done=lap_done,
                    target=self.target, gate_id=self.mp.gate_ids[self.target])
