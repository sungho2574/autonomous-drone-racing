"""Minimum-snap piecewise polynomial 궤적 생성 (ROS 비의존, numpy 만 사용).

Mellinger & Kumar (ICRA 2011) 방식:
- 각 세그먼트를 7차 다항식(계수 8개)으로 두고 snap(4차 미분)의 제곱 적분을 최소화
- 등식 제약: 웨이포인트 통과, 시작/끝 미분(vel/acc/jerk)=0, 세그먼트 경계에서
  1~4차 미분 연속
- 등식 제약을 nullspace 로 소거한 무제약 QP 를 폐형식으로 풀이

수치 안정성을 위해 세그먼트마다 정규화 시간 tau = t / T_i ∈ [0, 1] 로 풀고,
결과는 실제 시간 기준 계수 c_k / T_i^k 로 변환해 저장한다.

좌표계: ENU(map). yaw 는 +x 기준 CCW [rad], unwrap 된 연속값으로 다룬다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import factorial

import numpy as np

ORDER = 7                 # 다항식 차수
N_COEF = ORDER + 1
SNAP = 4                  # 최소화할 미분 차수
CONT_DERIVS = 4           # 경계에서 연속시킬 미분 차수 (1..4)
BOUNDARY_DERIVS = 3       # 시작/끝에서 0 으로 고정할 미분 차수 (vel, acc, jerk)


def _tvec(tau: float, d: int, T: float = 1.0) -> np.ndarray:
    """d차 미분 다항식 basis 벡터. p^(d)(t) = tvec · c, 정규화 시간 tau=t/T.

    실제 시간 미분이므로 1/T^d 스케일이 붙는다.
    """
    v = np.zeros(N_COEF)
    for k in range(d, N_COEF):
        v[k] = factorial(k) / factorial(k - d) * tau ** (k - d)
    return v / T ** d


def _snap_cost(T: float) -> np.ndarray:
    """정규화 시간에서의 snap 비용 행렬 Q (∫_0^T p''''(t)^2 dt)."""
    Q = np.zeros((N_COEF, N_COEF))
    for k in range(SNAP, N_COEF):
        for l in range(SNAP, N_COEF):
            ck = factorial(k) / factorial(k - SNAP)
            cl = factorial(l) / factorial(l - SNAP)
            Q[k, l] = ck * cl / (k + l - 2 * SNAP + 1)
    return Q / T ** (2 * SNAP - 1)


def solve_1d(waypoints: np.ndarray, durations: np.ndarray,
             v0: float = 0.0, v1: float = 0.0) -> np.ndarray:
    """1축 min-snap. 반환: (N, 8) 정규화 계수 (tau 기준).

    waypoints: (N+1,), durations: (N,)
    v0/v1: 시작/끝 속도 (acc, jerk 는 0 고정)
    """
    wp = np.asarray(waypoints, dtype=float)
    Ts = np.asarray(durations, dtype=float)
    N = len(Ts)
    if len(wp) != N + 1:
        raise ValueError('waypoints 수는 durations 수 + 1 이어야 함')
    if np.any(Ts <= 0):
        raise ValueError('duration 은 양수여야 함')

    n_var = N_COEF * N
    Q = np.zeros((n_var, n_var))
    for i, T in enumerate(Ts):
        sl = slice(i * N_COEF, (i + 1) * N_COEF)
        Q[sl, sl] = _snap_cost(T)

    rows, rhs = [], []

    def add(row_blocks, b):
        r = np.zeros(n_var)
        for i, vec in row_blocks:
            r[i * N_COEF:(i + 1) * N_COEF] += vec
        rows.append(r)
        rhs.append(b)

    # 웨이포인트 통과
    for i, T in enumerate(Ts):
        add([(i, _tvec(0.0, 0, T))], wp[i])
        add([(i, _tvec(1.0, 0, T))], wp[i + 1])
    # 시작/끝 미분 조건
    add([(0, _tvec(0.0, 1, Ts[0]))], v0)
    add([(N - 1, _tvec(1.0, 1, Ts[-1]))], v1)
    for d in range(2, BOUNDARY_DERIVS + 1):
        add([(0, _tvec(0.0, d, Ts[0]))], 0.0)
        add([(N - 1, _tvec(1.0, d, Ts[-1]))], 0.0)
    # 경계 연속
    for i in range(N - 1):
        for d in range(1, CONT_DERIVS + 1):
            add([(i, _tvec(1.0, d, Ts[i])), (i + 1, -_tvec(0.0, d, Ts[i + 1]))], 0.0)

    A = np.vstack(rows)
    b = np.asarray(rhs)
    # 등식 제약을 nullspace 로 소거한 뒤 무제약 QP 를 푼다 (KKT 직접 풀이보다 조건수에 강함).
    #   c = c_p + Z u,  A c_p = b,  A Z = 0  →  u* = -(ZᵀQZ)⁻¹ ZᵀQ c_p
    c_p, *_ = np.linalg.lstsq(A, b, rcond=None)
    _, sv, vt = np.linalg.svd(A)
    rank = int((sv > sv.max() * 1e-12).sum())
    Z = vt[rank:].T
    if Z.shape[1] > 0:
        H = Z.T @ Q @ Z
        g = Z.T @ Q @ c_p
        u = -np.linalg.solve(H, g)
        c = c_p + Z @ u
    else:
        c = c_p
    return c.reshape(N, N_COEF)


@dataclass
class Segment:
    duration: float
    coef: np.ndarray                 # (4, 8) 실제 시간 tau∈[0,duration] 기준, 행 = x, y, z, yaw

    def eval(self, tau: float, d: int = 0) -> np.ndarray:
        v = np.zeros(N_COEF)
        for k in range(d, N_COEF):
            v[k] = factorial(k) / factorial(k - d) * tau ** (k - d)
        return self.coef @ v


@dataclass
class Trajectory:
    segments: list[Segment] = field(default_factory=list)

    @property
    def duration(self) -> float:
        return float(sum(s.duration for s in self.segments))

    def _locate(self, t: float) -> tuple[Segment, float]:
        t = min(max(t, 0.0), self.duration)
        acc = 0.0
        for s in self.segments:
            if t <= acc + s.duration or s is self.segments[-1]:
                return s, t - acc
            acc += s.duration
        raise RuntimeError('unreachable')

    def sample(self, t: float):
        """t[s] 에서 (pos[3], vel[3], acc[3], yaw, yaw_rate). 범위 밖은 양 끝으로 클램프."""
        seg, tau = self._locate(t)
        p = seg.eval(tau, 0)
        v = seg.eval(tau, 1)
        a = seg.eval(tau, 2)
        return p[:3], v[:3], a[:3], float(p[3]), float(v[3])

    def sample_all(self, dt: float) -> np.ndarray:
        """(M, 1+3+3+3+2) 행렬: t, p, v, a, yaw, yaw_rate. 시각화/검사용."""
        ts = np.arange(0.0, self.duration + 1e-9, dt)
        out = np.zeros((len(ts), 12))
        for i, t in enumerate(ts):
            p, v, a, yaw, yr = self.sample(t)
            out[i] = [t, *p, *v, *a, yaw, yr]
        return out

    def peak(self, dt: float = 0.02) -> tuple[float, float]:
        s = self.sample_all(dt)
        v = np.linalg.norm(s[:, 4:7], axis=1).max()
        a = np.linalg.norm(s[:, 7:10], axis=1).max()
        return float(v), float(a)

    # ---- 직렬화 (msg 변환은 ROS 노드에서, 여기선 plain list) ----
    def to_plain(self) -> list[dict]:
        return [{'duration': s.duration,
                 'cx': s.coef[0].tolist(), 'cy': s.coef[1].tolist(),
                 'cz': s.coef[2].tolist(), 'cyaw': s.coef[3].tolist()}
                for s in self.segments]

    @classmethod
    def from_plain(cls, segs: list[dict]) -> 'Trajectory':
        out = cls()
        for s in segs:
            coef = np.zeros((4, N_COEF))
            for r, key in enumerate(('cx', 'cy', 'cz', 'cyaw')):
                c = np.asarray(s[key], dtype=float)
                coef[r, :len(c)] = c
            out.segments.append(Segment(float(s['duration']), coef))
        return out


def unwrap_yaw(yaws: np.ndarray) -> np.ndarray:
    return np.unwrap(np.asarray(yaws, dtype=float))


def allocate_time(waypoints: np.ndarray, v_avg: float, t_min: float = 0.3,
                  rest_extra: float = 0.0) -> np.ndarray:
    """세그먼트 길이에 비례해 시간 할당.

    rest_extra: 정지 상태에서 출발/정지로 끝나는 첫·마지막 세그먼트에 더해 줄 시간.
    (v_avg/a_max 정도를 주면 출발·정지 가속 스파이크가 줄어든다)
    """
    d = np.linalg.norm(np.diff(np.asarray(waypoints, dtype=float)[:, :3], axis=0), axis=1)
    Ts = np.maximum(d / v_avg, t_min)
    Ts[0] += rest_extra
    Ts[-1] += rest_extra
    return Ts


def plan(waypoints, yaws, v_avg: float = 3.0, v_max: float | None = None,
         a_max: float | None = None, durations=None, t_min: float = 0.3) -> Trajectory:
    """웨이포인트(N+1,3), yaw(N+1) → min-snap Trajectory (ENU).

    v_max/a_max 를 주면 샘플링으로 최대치를 검사해 전체 시간을 균일 스케일한다
    (최대 5회 반복).
    """
    wp = np.asarray(waypoints, dtype=float)
    if wp.ndim != 2 or wp.shape[1] != 3 or len(wp) < 2:
        raise ValueError('waypoints 는 (N+1, 3), N>=1')
    yw = unwrap_yaw(yaws)
    if len(yw) != len(wp):
        raise ValueError('yaws 길이는 waypoints 와 같아야 함')
    if durations is not None:
        Ts = np.asarray(durations, dtype=float)
    else:
        rest_extra = v_avg / a_max if a_max else 0.5
        Ts = allocate_time(wp, v_avg, t_min, rest_extra)

    def build(Ts):
        cs = [solve_1d(wp[:, k], Ts) for k in range(3)] + [solve_1d(yw, Ts)]
        traj = Trajectory()
        for i, T in enumerate(Ts):
            coef = np.zeros((4, N_COEF))
            for r in range(4):
                # 정규화 계수 → 실제 시간 계수: c_k / T^k
                coef[r] = cs[r][i] / T ** np.arange(N_COEF)
            traj.segments.append(Segment(float(T), coef))
        return traj

    traj = build(Ts)
    if v_max is None and a_max is None:
        return traj
    for _ in range(5):
        vp, ap = traj.peak()
        s = 1.0
        if v_max is not None and vp > v_max:
            s = max(s, vp / v_max)
        if a_max is not None and ap > a_max:
            s = max(s, np.sqrt(ap / a_max))
        if s <= 1.001:
            break
        Ts = Ts * s
        traj = build(Ts)
    return traj
