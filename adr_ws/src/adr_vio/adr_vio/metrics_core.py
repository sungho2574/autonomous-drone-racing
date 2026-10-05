"""ROS-independent freshness/rate and timing CSV helpers."""

from collections import deque
from pathlib import Path
import math


class MetricStore:
    def __init__(self, stale_after=2.0):
        self.values, self.events = {}, {}
        self.stale_after = stale_after

    def put(self, name, value, now):
        if math.isfinite(float(value)):
            self.values[name] = (float(value), now)

    def event(self, name, stamp, now):
        q = self.events.setdefault(name, deque(maxlen=300))
        if q and stamp <= q[-1][0]:
            if stamp < q[-1][0]:
                q.clear()
            else:
                return
        q.append((stamp, now))

    def snapshot(self, now):
        out = {}
        for k, (value, received) in self.values.items():
            if 0 <= now - received <= self.stale_after:
                out[k] = value
        for k, q in self.events.items():
            if q:
                out[k + "_age_s"] = max(0.0, now - q[-1][1])
                recent = [s for s, wall in q if now - wall <= self.stale_after]
                if len(recent) >= 2 and recent[-1] > recent[0]:
                    out[k + "_hz"] = (len(recent) - 1) / (recent[-1] - recent[0])
        return out


class TimingTail:
    """Read only completed CSV lines; upstream flushes each estimator update."""

    def __init__(self, path):
        self.path, self.offset, self.columns = path, 0, []

    def read(self):
        if not self.path or not Path(self.path).is_file():
            return None
        result = None
        with open(self.path) as f:
            if Path(self.path).stat().st_size < self.offset:
                self.offset, self.columns = 0, []
            f.seek(self.offset)
            while True:
                line = f.readline()
                if not line or not line.endswith("\n"):
                    break
                self.offset = f.tell()
                if line.startswith("#"):
                    self.columns = [
                        x.strip().replace(" ", "_").replace("&", "and")
                        for x in line[1:].split(",")
                    ]
                elif self.columns:
                    try:
                        values = [float(x) for x in line.split(",")]
                        if len(values) == len(self.columns):
                            result = {
                                ("timing_" + k + "_ms"): v * 1000
                                for k, v in zip(self.columns[1:], values[1:])
                            }
                    except ValueError:
                        continue
        return result
