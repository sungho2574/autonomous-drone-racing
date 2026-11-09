"""Timestamped drift updates; output prediction never advances measurement state."""
from copy import deepcopy
import numpy as np


def interpolate(rows, t, max_gap):
    if not rows or t < rows[0][0] or t > rows[-1][0]:
        return None
    i = int(np.searchsorted([r[0] for r in rows], t))
    if rows[i][0] == t:
        return rows[i][1].copy()
    a, b = rows[i - 1], rows[i]
    if b[0] - a[0] > max_gap:
        return None
    return a[1] + (b[1] - a[1]) * ((t - a[0]) / (b[0] - a[0]))


class TimedDrift:
    def __init__(self, kf, hold_timeout, recovery_samples=3, recovery_spread=0.5,
                 recovery_max_gap=0.4, recovery_min_span=0.15):
        self.kf = kf
        if (recovery_samples < 3 or not np.isfinite([recovery_spread, recovery_max_gap, recovery_min_span]).all()
                or recovery_spread <= 0 or recovery_max_gap <= 0 or recovery_min_span <= 0):
            raise ValueError('Invalid recovery confirmation settings')
        self.recovery_samples = int(recovery_samples)
        self.recovery_spread = float(recovery_spread)
        self.recovery_max_gap = float(recovery_max_gap)
        self.recovery_min_span = float(recovery_min_span)
        self.candidates = []
        self.last_recovered = False
        self.time = None
        self.last_update = None
        self.hold_timeout = hold_timeout

    def _advance(self, kf, t):
        if self.time is None:
            return
        end = self.time if self.last_update is None else self.last_update + self.hold_timeout
        active = max(0., min(t, end) - self.time)
        kf.predict(active)
        kf.predict(max(0., t - self.time - active), hold=True)

    def update(self, t, pnp, vio, quality, gate_id=None):
        self.last_recovered = False
        if self.time is not None and t <= self.time:
            return False
        self._advance(self.kf, t)
        self.time = t
        accepted = self.kf.update(pnp, vio, quality) is not None
        if accepted:
            self.candidates.clear()
        elif self.kf.last_rejection in ('position_residual', 'innovation'):
            residual = np.asarray(pnp) - np.asarray(vio)
            if self.candidates and (t - self.candidates[-1][0] > self.recovery_max_gap
                    or gate_id != self.candidates[-1][2]
                    or any(np.linalg.norm(residual - c[1]) > self.recovery_spread for c in self.candidates)):
                self.candidates.clear()
            self.candidates.append((t, residual.copy(), gate_id, quality))
            self.candidates = self.candidates[-self.recovery_samples:]
            if len(self.candidates) >= self.recovery_samples and t - self.candidates[0][0] >= self.recovery_min_span:
                # Confirmed relocalization: reset drift, never infer drift velocity from the jump.
                self.kf.x[:3] = np.median([c[1] for c in self.candidates], axis=0)
                self.kf.x[3:] = 0
                variance = self.kf.sigma_r ** 2 * self.kf.lambda_r / min(c[3] for c in self.candidates)
                self.kf.P = np.diag([*variance, 1., 1., 1.])
                self.kf.n_rejected -= 1
                self.kf.n_update += 1
                self.kf.last_rejection = None
                self.candidates.clear()
                self.last_recovered = accepted = True
        else:
            self.candidates.clear()
        if accepted:
            self.last_update = t
        return accepted

    def at(self, t):
        predicted = deepcopy(self.kf)
        self._advance(predicted, t)
        return predicted
