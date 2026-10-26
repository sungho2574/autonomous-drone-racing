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
    def __init__(self, kf, hold_timeout):
        self.kf = kf
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

    def update(self, t, pnp, vio, quality):
        if self.time is not None and t <= self.time:
            return False
        self._advance(self.kf, t)
        self.time = t
        accepted = self.kf.update(pnp, vio, quality) is not None
        if accepted:
            self.last_update = t
        return accepted

    def at(self, t):
        predicted = deepcopy(self.kf)
        self._advance(predicted, t)
        return predicted
