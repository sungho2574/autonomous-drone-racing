"""Offline perception-aware yaw overlay (spec §3.2) for the min-snap position path.

Gate headings mean line-of-sight bearings, not the map's gate normals. Distance
weights blend the active and following gate; lambda_g blends that with velocity.
A noncausal smoother anticipates gate handoffs. Optional rate/acceleration limits use uniform time dilation. They are disabled
by default, preserving both the position path and its timing.
Visibility is a measured objective, not a promise: yaw cannot fix vertical FOV,
occlusion, or an opening larger than the image at traversal.
"""
from dataclasses import dataclass
from math import factorial

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d

from adr_planning.min_snap import Segment, Trajectory


@dataclass(frozen=True)
class HeadingOptions:
    d_min: float = 1.0
    d_max: float = 8.0
    lambda_g: float = 0.95
    sample_dt: float = 0.1
    smoothing_time: float = 0.3
    max_rate_deg: float = 0.0  # 0 disables this limit
    max_accel_deg: float = 0.0  # 0 disables this limit
    center_tolerance_deg: float = 10.0
    hfov_deg: float = 90.0
    camera_width: int = 640
    camera_height: int = 480
    camera_tilt_deg: float = 20.0
    camera_xyz: tuple = (0.045, 0.0, 0.022)

    def __post_init__(self):
        values = [self.d_min, self.d_max, self.lambda_g, self.sample_dt,
                  self.smoothing_time, self.max_rate_deg, self.max_accel_deg,
                  self.center_tolerance_deg, self.hfov_deg, self.camera_width,
                  self.camera_height, self.camera_tilt_deg, *self.camera_xyz]
        if not np.isfinite(values).all():
            raise ValueError('heading parameters must be finite')
        if not 0 <= self.d_min < self.d_max or not 0 <= self.lambda_g <= 1:
            raise ValueError('heading requires 0 <= d_min < d_max and 0 <= lambda_g <= 1')
        if min(self.sample_dt, self.smoothing_time, self.camera_width, self.camera_height) <= 0:
            raise ValueError('heading time and camera dimensions must be positive')
        if min(self.max_rate_deg, self.max_accel_deg) < 0:
            raise ValueError('heading limits must be nonnegative (0 disables)')
        if not 0 < self.hfov_deg < 180 or not 0 <= self.center_tolerance_deg < self.hfov_deg / 2:
            raise ValueError('invalid heading FOV / center tolerance')
        if len(self.camera_xyz) != 3:
            raise ValueError('camera_xyz must have three elements')


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def blend_angle(a, b, weight):
    """Shortest-arc interpolation; deterministic +pi tie for a 180 degree turn."""
    delta = wrap(b - a)
    if abs(abs(delta) - np.pi) < 1e-10:
        delta = np.pi
    return a + weight * delta


def gate_schedule(traj, course, laps, approach_dist, end_at_start=True):
    """Gate-center passage times from the waypoint layout, including lap wrap."""
    count = max(1, int(laps)) * len(course.gates)
    stride = 3 if approach_dist > 0 else 1
    expected = count * stride + int(end_at_start)
    if len(traj.segments) != expected:
        raise ValueError('position trajectory does not match course waypoint layout')
    times = np.r_[0.0, np.cumsum([s.duration for s in traj.segments])]
    centers = 1 + np.arange(count) * stride + int(approach_dist > 0)
    return times[centers], [course.gates[i % len(course.gates)] for i in range(count)]


def body_rotation(acceleration, yaw):
    """Flatness attitude estimate: body z follows thrust, heading fixes body x."""
    z = np.asarray(acceleration, float) + [0, 0, 9.8]
    z /= max(np.linalg.norm(z), 1e-9)
    y = np.cross(z, [np.cos(yaw), np.sin(yaw), 0])
    if np.linalg.norm(y) < 1e-8:
        return np.eye(3)
    y /= np.linalg.norm(y)
    return np.column_stack((np.cross(y, z), y, z))


def optical_points(points, position, acceleration, yaw, options):
    R = body_rotation(acceleration, yaw)
    tilt = np.radians(options.camera_tilt_deg)
    c, s = np.cos(tilt), np.sin(tilt)
    # Optical x=right, y=down, z=forward; upward camera mount tilt.
    R_bo = np.array([[0, s, c], [-1, 0, 0], [0, -c, s]])
    origin = position + R @ np.asarray(options.camera_xyz)
    return (np.asarray(points) - origin) @ R @ R_bo


def camera_bearing(gate, p, a, fallback, options):
    delta = gate.center - p
    # At/above the opening center, horizontal bearing is ill-defined.
    if np.linalg.norm(delta[:2]) < 0.25:
        return fallback
    yaw = np.arctan2(delta[1], delta[0])
    for _ in range(4):
        optical = optical_points(gate.center, p, a, yaw, options)
        yaw -= np.clip(np.arctan2(optical[0], optical[2]), -0.4, 0.4)
    return yaw


def desired_headings(samples, passage_times, gates, options):
    values = []
    fallback = gates[0].yaw
    for row in samples:
        t, p, v, a = row[0], row[1:4], row[4:7], row[7:10]
        i = int(np.searchsorted(passage_times, t, side='right'))
        if i == len(gates):
            # Race finished: retain last viewing direction through return/hover.
            values.append(fallback)
            continue
        current = camera_bearing(gates[i], p, a, fallback, options)
        next_i = min(i + 1, len(gates) - 1)
        following = camera_bearing(gates[next_i], p, a, current, options)
        distances = np.array([np.linalg.norm(gates[j].center - p) for j in (i, next_i)])
        weights = np.clip((options.d_max - distances) / (options.d_max - options.d_min), 0, 1)
        gate_heading = blend_angle(current, following, weights[1] / weights.sum()) if weights.sum() > 1e-9 else current
        forward = np.arctan2(v[1], v[0]) if np.linalg.norm(v[:2]) > 0.05 else current
        target = blend_angle(forward, gate_heading, options.lambda_g)
        # Centering constraint on the unsmoothed target. Smoothing/finite turn rate
        # may temporarily violate it at handoffs, explicitly reported below.
        tol = np.radians(options.center_tolerance_deg)
        target = current + np.clip(wrap(target - current), -tol, tol)
        values.append(target)
        fallback = target
    return np.unwrap(values)


def angular_peaks(traj):
    """Exact peak |yaw rate| / |yaw acceleration| over each polynomial segment."""
    peaks = [0.0, 0.0]
    for seg in traj.segments:
        for index, order in enumerate((1, 2)):
            coef = np.polynomial.polynomial.polyder(seg.coef[3], order)
            roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(coef))
            times = [0.0, seg.duration] + [float(r.real) for r in roots
                      if abs(r.imag) < 1e-8 and 0 < r.real < seg.duration]
            peaks[index] = max(peaks[index], float(np.max(np.abs(np.polynomial.polynomial.polyval(times, coef)))))
    return np.array(peaks)


def visibility(traj, passage_times, gates, inner_size, options):
    """Full opening-corner visibility using predicted thrust attitude and camera mount.

    Excludes post-race return. No occlusion model; this is predicted visibility,
    not a measurement of the flown camera stream.
    """
    seen, centered, total = 0, 0, 0
    dt = max(0.05, options.sample_dt)
    fx = options.camera_width / (2 * np.tan(np.radians(options.hfov_deg) / 2))
    for t in np.arange(0, passage_times[-1], dt):
        i = min(np.searchsorted(passage_times, t, side='right'), len(gates) - 1)
        g = gates[i]
        p, _, a, yaw, _ = traj.sample(t)
        right = np.array([np.sin(g.yaw), -np.cos(g.yaw), 0]) * inner_size / 2
        up = np.array([0, 0, inner_size / 2])
        points = np.array([g.center-right+up, g.center+right+up,
                           g.center+right-up, g.center-right-up, g.center])
        opt = optical_points(points, p, a, yaw, options)
        z = opt[:, 2]
        pixels = fx * opt[:, :2] / np.maximum(z[:, None], 1e-9)
        inside = (z > 0) & (abs(pixels[:, 0]) <= options.camera_width / 2) & (abs(pixels[:, 1]) <= options.camera_height / 2)
        seen += bool(inside[:4].all())
        centered += bool(inside[4] and abs(np.arctan2(opt[4, 0], z[4])) <= np.radians(options.center_tolerance_deg))
        total += 1
    return {'visible_fraction': seen / max(total, 1), 'centered_fraction': centered / max(total, 1)}


def perception_aware_heading(traj, course, laps=1, approach_dist=0.8,
                             end_at_start=True, options=None):
    """Return serialized-compatible polynomial trajectory and diagnostics.

    Original xyz polynomials are subdivided algebraically, never refitted. Only
    yaw changes geometrically. Timing is preserved unless optional angular limits
    are explicitly enabled.
    """
    options = options or HeadingOptions()
    passages, gates = gate_schedule(traj, course, laps, approach_dist, end_at_start)
    ts = np.linspace(0, traj.duration, max(3, int(np.ceil(traj.duration / options.sample_dt)) + 1))
    rows = []
    for t in ts:
        p, v, a, _, _ = traj.sample(t)
        rows.append([t, *p, *v, *a])
    samples = np.asarray(rows)
    targets = desired_headings(samples, passages, gates, options)
    smooth = gaussian_filter1d(targets, options.smoothing_time / (ts[1] - ts[0]), mode='nearest')
    spline = CubicSpline(ts, smooth, bc_type=((1, 0.0), (1, 0.0)))
    old_times = np.r_[0.0, np.cumsum([s.duration for s in traj.segments])]
    knots = np.unique(np.r_[ts, old_times])
    # Merge numerical duplicates at gate boundaries.
    knots = knots[np.r_[True, np.diff(knots) > 1e-8]]
    result = Trajectory()
    for lo, hi in zip(knots[:-1], knots[1:]):
        index = min(np.searchsorted(old_times, lo + 1e-9, side='right') - 1, len(traj.segments) - 1)
        seg = traj.segments[index]
        offset = lo - old_times[index]
        coef = np.zeros((4, 8))
        for d in range(8):
            coef[:3, d] = seg.eval(offset, d)[:3] / factorial(d)
        for d in range(4):
            coef[3, d] = spline(lo, d) / factorial(d)
        result.segments.append(Segment(float(hi - lo), coef))
    rate, accel = angular_peaks(result)
    stretch = 1.0
    if options.max_rate_deg > 0:
        stretch = max(stretch, rate / np.radians(options.max_rate_deg))
    if options.max_accel_deg > 0:
        stretch = max(stretch, np.sqrt(accel / np.radians(options.max_accel_deg)))
    for seg in result.segments:
        seg.duration *= stretch
        seg.coef /= stretch ** np.arange(8)
    matched = Trajectory([Segment(seg.duration * stretch, seg.coef / stretch**np.arange(8))
                          for seg in traj.segments])
    report = {'time_stretch': float(stretch), 'max_rate_deg': float(np.degrees(rate / stretch)),
              'max_accel_deg': float(np.degrees(accel / stretch**2)),
              'baseline': visibility(traj, passages, gates, course.inner_size, options),
              'baseline_matched_time': visibility(matched, passages * stretch, gates, course.inner_size, options),
              'planned': visibility(result, passages * stretch, gates, course.inner_size, options)}
    return result, report
