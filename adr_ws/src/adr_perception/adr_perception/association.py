"""Map gate association and timestamped pose interpolation, independent of ROS."""

from dataclasses import dataclass
import numpy as np
import cv2
from scipy.spatial.transform import Rotation, Slerp
from adr_perception.pnp import gate_object_points, gate_world_transform


@dataclass
class MatchConfig:
    size_fraction: float = 0.15
    min_px: float = 10.0
    max_px: float = 45.0
    max_corner_factor: float = 1.5
    ratio: float = 0.7
    margin_px: float = 5.0

    def __post_init__(self):
        if not np.isfinite(list(vars(self).values())).all():
            raise ValueError("Association thresholds must be finite")
        if not (
            0 < self.size_fraction
            and 0 < self.min_px <= self.max_px
            and self.max_corner_factor >= 1
            and 0 < self.ratio < 1
            and self.margin_px > 0
        ):
            raise ValueError("Invalid association threshold range")


class PoseBuffer:
    def __init__(self, max_gap=0.2, horizon=3.0):
        self.rows = []
        self.max_gap, self.horizon = max_gap, horizon

    def add(self, t, p, q):
        if not np.isfinite([t, *p, *q]).all() or np.linalg.norm(q) < 1e-8:
            return
        if self.rows and t < self.rows[-1][0] - self.horizon:
            self.rows.clear()  # simulation clock reset
        self.rows = [r for r in self.rows if r[0] != t]
        self.rows.append((t, np.asarray(p), np.asarray(q)))
        self.rows.sort(key=lambda r: r[0])
        self.rows = [r for r in self.rows if r[0] >= self.rows[-1][0] - self.horizon]

    def sample(self, t):
        if not self.rows:
            return None
        times = [r[0] for r in self.rows]
        i = int(np.searchsorted(times, t))
        if i < len(times) and abs(times[i] - t) < 1e-8:
            _, p, q = self.rows[i]
            rot = Rotation.from_quat(q).as_matrix()
        elif i == 0 or i == len(times) or times[i] - times[i - 1] > self.max_gap:
            return None
        else:
            a, b = self.rows[i - 1], self.rows[i]
            f = (t - a[0]) / (b[0] - a[0])
            p = a[1] + f * (b[1] - a[1])
            rot = Slerp([0, 1], Rotation.from_quat([a[2], b[2]]))([f]).as_matrix()[0]
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = rot, p
        return T


def project_gates(gates, inner_size, T_map_base, T_base_opt, K, D, width, height):
    T_opt_map = np.linalg.inv(T_map_base @ T_base_opt)
    camera = (T_map_base @ T_base_opt)[:3, 3]
    result = []
    for g in gates:
        # Only front-face observations: current PnP object frame assumes this side.
        if (camera - g.center) @ g.normal >= 0:
            continue
        T = T_opt_map @ gate_world_transform(g.center, g.yaw)
        p = gate_object_points(inner_size) @ T[:3, :3].T + T[:3, 3]
        if np.any(p[:, 2] <= 0.05):
            continue
        uv = cv2.projectPoints(p, np.zeros(3), np.zeros(3), K, D)[0].reshape(4, 2)
        if not np.isfinite(uv).all():
            continue
        if (
            np.any(uv[:, 0] < 0)
            or np.any(uv[:, 0] >= width)
            or np.any(uv[:, 1] < 0)
            or np.any(uv[:, 1] >= height)
        ):
            continue
        # Degenerate/edge-on quadrilaterals do not support reliable four-corner PnP.
        edges = np.linalg.norm(uv - np.roll(uv, 1, axis=0), axis=1)
        if edges.min() < 4 or abs(cv2.contourArea(uv.astype(np.float32))) < 25:
            continue
        result.append((g, uv))
    return result


def associate(detections, projected, config):
    """Compare every detection against all visible gates; never use route order as ID.

    Cyclic permutations preserve winding and return physical object-point order.
    Runner-up is retained even if it narrowly fails the absolute threshold.
    """
    accepted, rejected = [], []
    for index, detection in enumerate(detections):
        points = np.asarray(detection, dtype=float).reshape(4, 2)
        if not np.isfinite(points).all():
            continue
        scores = []
        for gate, prior in projected:
            choices = []
            for shift in range(4):
                corners = np.roll(points, shift, axis=0)
                errors = np.linalg.norm(corners - prior, axis=1)
                choices.append(
                    (float(np.sqrt(np.mean(errors**2))), float(errors.max()), corners)
                )
            rmse, worst, corners = min(choices, key=lambda x: x[0])
            size = float(
                np.linalg.norm(prior - np.roll(prior, 1, axis=0), axis=1).mean()
            )
            limit = float(
                np.clip(size * config.size_fraction, config.min_px, config.max_px)
            )
            scores.append(
                dict(
                    gate=gate,
                    rmse=rmse,
                    worst=worst,
                    corners=corners,
                    limit=limit,
                    detection=index,
                )
            )
        if not scores:
            continue
        scores.sort(key=lambda s: s["rmse"])
        best = scores[0]
        second = scores[1] if len(scores) > 1 else None
        best["second"] = second
        best["reason"] = "accepted"
        if (
            best["rmse"] > best["limit"]
            or best["worst"] > config.max_corner_factor * best["limit"]
        ):
            best["reason"] = "absolute_error"
        elif second and (
            best["rmse"] > config.ratio * second["rmse"]
            or second["rmse"] - best["rmse"] < config.margin_px
        ):
            best["reason"] = "ambiguous"
        (accepted if best["reason"] == "accepted" else rejected).append(best)
    # Multiple plausible detections claiming one ID are ambiguous as well.
    unique = []
    for candidate in accepted:
        if sum(c["gate"].id == candidate["gate"].id for c in accepted) > 1:
            candidate["reason"] = "duplicate_id"
            rejected.append(candidate)
        else:
            unique.append(candidate)
    pool = unique or rejected
    return min(pool, key=lambda c: c["rmse"] / c["limit"]) if pool else None
