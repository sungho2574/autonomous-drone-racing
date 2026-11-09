// Count sorted samples at or before the selected recording time.
export function upperBound(values, time, getTime = value => value) {
  let lo = 0, hi = values.length;
  while (lo < hi) {
    const mid = (lo + hi) >>> 1;
    if (getTime(values[mid]) <= time) lo = mid + 1;
    else hi = mid;
  }
  return lo;
}
