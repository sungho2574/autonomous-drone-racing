// Rows: [seconds since bag origin, x, y, z, qw, qx, qy, qz].
// Never clamp to an endpoint or interpolate across a measurement outage.
function rotation(row) {
  const q = [row[5], row[6], row[7], row[4]];
  const length = Math.hypot(...q);
  return Number.isFinite(length) && length > 1e-9
    ? q.map((v) => v / length)
    : null;
}
function slerp(a, b, f) {
  if (!a || !b) return null;
  let dot = a.reduce((v, x, i) => v + x * b[i], 0);
  if (dot < 0) {
    b = b.map((v) => -v);
    dot = -dot;
  }
  if (dot > 0.9995) {
    const q = a.map((v, i) => v + f * (b[i] - v));
    const n = Math.hypot(...q);
    return q.map((v) => v / n);
  }
  const theta = Math.acos(Math.min(1, dot)),
    sin = Math.sin(theta);
  return a.map(
    (v, i) =>
      (Math.sin((1 - f) * theta) * v + Math.sin(f * theta) * b[i]) / sin,
  );
}
export function samplePose(rows, t, maxGap = 0.3) {
  if (
    !Number.isFinite(t) ||
    !rows?.length ||
    t < rows[0][0] ||
    t > rows[rows.length - 1][0]
  )
    return null;
  let lo = 0,
    hi = rows.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (rows[mid][0] < t) lo = mid + 1;
    else hi = mid;
  }
  const b = rows[lo];
  if (b[0] === t) return { position: b.slice(1, 4), quaternion: rotation(b) };
  if (lo === 0) return null;
  const a = rows[lo - 1],
    dt = b[0] - a[0];
  if (dt <= 0 || dt > maxGap) return null;
  const f = (t - a[0]) / dt;
  return {
    position: a.slice(1, 4).map((v, i) => v + f * (b[i + 1] - v)),
    quaternion: slerp(rotation(a), rotation(b), f),
  };
}
