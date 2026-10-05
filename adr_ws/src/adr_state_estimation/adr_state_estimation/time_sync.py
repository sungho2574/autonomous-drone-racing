"""Bounded, timestamp-based VIO/PnP pairing, independent of ROS."""
from bisect import bisect_left
from collections import Counter
import numpy as np


class PoseSync:
    def __init__(self, max_gap=0.15, history=10.0, wait=5.0, limit=500):
        if min(max_gap, history, wait, limit) <= 0:
            raise ValueError('Synchronization limits must be positive')
        self.max_gap, self.history, self.wait, self.limit = max_gap, history, wait, limit
        self.poses, self.pending = [], []
        self.counts = Counter()
        self.last = float('-inf')

    def add_pose(self, t, position):
        i = bisect_left([r[0] for r in self.poses], t)
        row = (t, np.asarray(position, dtype=float))
        if i < len(self.poses) and self.poses[i][0] == t:
            self.poses[i] = row
        else:
            self.poses.insert(i, row)
        cutoff = self.poses[-1][0] - self.history
        i = bisect_left([r[0] for r in self.poses], cutoff)
        self.poses = self.poses[max(0, i-1):][-10000:]

    def add_pnp(self, t, payload, now):
        self.counts['received'] += 1
        if t <= self.last or any(r[0] == t for r in self.pending):
            self.counts['duplicate_or_late'] += 1
            return
        self.pending.append((t, now, payload))
        self.pending.sort(key=lambda r: r[0])
        if len(self.pending) > self.limit:
            self.pending.pop(0)
            self.counts['overflow'] += 1

    def drain(self, now):
        ready = []
        while self.pending:
            t, arrival, payload = self.pending[0]
            times = [r[0] for r in self.poses]
            i = bisect_left(times, t)
            reason, p = None, None
            if t <= self.last:
                reason = 'duplicate_or_late'
            elif i < len(times) and times[i] == t:
                p = self.poses[i][1]
            elif 0 < i < len(times):
                a, b = self.poses[i-1], self.poses[i]
                if b[0] - a[0] <= self.max_gap:
                    f = (t-a[0])/(b[0]-a[0])
                    p = (1-f)*a[1] + f*b[1]
                elif now-arrival >= self.wait:
                    reason = 'vio_gap'
            elif times and t < times[0] and times[-1]-t > self.history:
                reason = 'history_expired'
            if p is None and reason is None:
                if now-arrival < self.wait:
                    break
                reason = 'wait_timeout'
            self.pending.pop(0)
            if reason:
                self.counts[reason] += 1
            else:
                self.last = t
                self.counts['matched'] += 1
                ready.append((t, p.copy(), payload))
        return ready
