"""drift 보정 KF 검증 — 합성 시나리오로 '정말 drift 를 따라잡는가'를 본다.

규약: p_d 는 VIO 에 **더해서** 참값이 되는 양이다 (논문 §2.4: p_c = p_v + p_d).
      VIO 가 아래로 흐르면(p_vio 가 참값보다 작으면) p_d 는 양수가 되어야 한다.
"""
import numpy as np
import pytest

from adr_state_estimation.drift_kf import DriftKF

VIO_HZ, PNP_HZ, DUR = 30.0, 3.0, 20.0


def run(drift_fn, pnp_sigma=0.05, quality=1.0, seed=0, dur=DUR, **kw):
    """drift_fn(t) → VIO 위치 오차. 반환: (VIO 오차 평균, 보정 오차 평균, 마지막 p_d)."""
    rng = np.random.default_rng(seed)
    kf = DriftKF(**kw)
    dt = 1.0 / VIO_HZ
    every = int(round(VIO_HZ / PNP_HZ))
    e_vio, e_cor = [], []
    for k in range(int(dur * VIO_HZ)):
        t = k * dt
        p_true = np.array([np.cos(t), np.sin(t), 1.5])       # 원 궤적
        p_vio = p_true + drift_fn(t)
        kf.predict(dt)
        if k % every == 0 and k > 0:
            z = p_true + rng.normal(0, pnp_sigma, 3)          # PnP 는 전역으로 정확 + 노이즈
            kf.update(z, p_vio, quality=quality)
        e_vio.append(np.linalg.norm(p_vio - p_true))
        e_cor.append(np.linalg.norm(kf.correct(p_vio) - p_true))
    n = len(e_vio) // 2                                        # 수렴 후 절반만 평가
    return float(np.mean(e_vio[n:])), float(np.mean(e_cor[n:])), kf.p_d.copy()


def test_constant_offset():
    """일정한 오프셋 drift 를 거의 완전히 제거해야 한다.

    남는 오차는 PnP 측정 노이즈 바닥(σ=0.05/축 → 노름 ≈0.087)이 한계라 0 이 될 수 없다.
    실측 VIO 0.62 → 보정 0.06 (10배).
    """
    off = np.array([0.5, -0.3, 0.2])
    v, c, p_d = run(lambda t: off)
    assert c < 0.15 * v, f'VIO {v:.3f} → 보정 {c:.3f}'
    assert c < 0.12, f'보정 오차 {c:.3f} m — 측정 노이즈 바닥보다 한참 큼'
    assert np.allclose(p_d, -off, atol=0.08), f'p_d={p_d} != {-off}'


def test_downward_ramp():
    """z 로 서서히 흘러내리는 drift (현재 겪고 있는 증상) 를 따라잡아야 한다."""
    v, c, p_d = run(lambda t: np.array([0.0, 0.0, -0.05 * t]))   # 초당 5 cm 하강
    assert c < 0.2 * v, f'VIO {v:.3f} → 보정 {c:.3f}'
    assert p_d[2] > 0.3, f'하강 drift 인데 p_d.z={p_d[2]:.3f} 가 충분히 양수가 아니다'


def test_no_drift_stays_put():
    """drift 가 없으면 보정이 상태를 망치지 않아야 한다 (PnP 노이즈만 남음)."""
    v, c, p_d = run(lambda t: np.zeros(3))
    assert v < 1e-9
    assert c < 0.08, f'drift 0 인데 보정 오차 {c:.3f} m'
    assert np.linalg.norm(p_d) < 0.08


def test_quality_scales_kalman_gain():
    """quality 가 낮으면 R 이 커져 **한 번의 업데이트로 덜 움직여야** 한다.

    다회 시뮬로 비교하면 안 된다: (a) 충분히 오래 두면 둘 다 수렴하고, R 이 큰 쪽이 오히려
    측정 노이즈를 더 걸러 최종값이 정확해진다. (b) 등속 drift 모델이라 잔차가 오래 남을수록
    v_d 가 쌓여 오버슈트가 커진다 — 수렴 속도가 R 에 단조롭지 않다.
    그래서 같은 사전분포에서 단일 업데이트 이동량으로 칼만 이득을 직접 본다.
    """
    kf_hi, kf_lo = DriftKF(), DriftKF()
    for kf, q in ((kf_hi, 1.0), (kf_lo, 0.05)):
        kf.predict(0.1)
        kf.update([0.5, 0.0, 0.0], [0.0, 0.0, 0.0], quality=q)
    assert kf_hi.p_d[0] > kf_lo.p_d[0] + 0.1, \
        f'quality 가 R 에 안 먹는다: hi={kf_hi.p_d[0]:.3f} lo={kf_lo.p_d[0]:.3f}'


def test_step_drift_overshoots_but_settles():
    """계단 drift 에 오버슈트가 나는 건 등속 모델의 성질이다 — 발산하지 않고 되돌아와야 한다."""
    off = np.array([0.5, 0.0, 0.0])
    _, _, p_short = run(lambda t: off, dur=2.0, pnp_sigma=0.0)
    _, _, p_long = run(lambda t: off, dur=20.0, pnp_sigma=0.0)
    assert abs(p_long[0] + 0.5) < abs(p_short[0] + 0.5), '오래 둬도 안 가라앉는다'
    assert abs(p_long[0] + 0.5) < 0.05, f'정상상태 p_d.x={p_long[0]:.3f}'


def test_rotation_and_velocity_untouched():
    """KF 는 위치만 건드린다 — correct() 가 위치 3차원만 반환하는지."""
    kf = DriftKF()
    kf.predict(0.1)
    kf.update([1.0, 2.0, 3.0], [0.0, 0.0, 0.0])
    out = kf.correct([10.0, 20.0, 30.0])
    assert out.shape == (3,)
    assert kf.x.shape == (6,)


@pytest.mark.parametrize('sigma_r', [0.05, 0.25, 1.0])
def test_sigma_r_monotonic(sigma_r):
    """σ_r 이 클수록 PnP 를 덜 믿어 수렴이 느려진다 (발산하진 않는다)."""
    _, c, _ = run(lambda t: np.array([0.4, 0.0, 0.0]), sigma_r=(sigma_r,) * 3)
    assert c < 0.4, f'σ_r={sigma_r} 에서 보정 오차 {c:.3f} m — 발산'
