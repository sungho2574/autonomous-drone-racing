"""Translation-only APE on a uniform clock, without post-hoc spatial alignment."""

import numpy as np


def interpolate(rows, times, max_gap=0.2):
    out = np.full((len(times), 3), np.nan)
    if not rows:
        return out
    a = np.asarray(rows, dtype=float)
    right = np.searchsorted(a[:, 0], times)
    exact = (right < len(a)) & (
        np.abs(a[np.minimum(right, len(a) - 1), 0] - times) < 1e-8
    )
    out[exact] = a[right[exact], 1:4]
    between = (~exact) & (right > 0) & (right < len(a))
    indices = np.flatnonzero(between)
    r = right[indices]
    dt = a[r, 0] - a[r - 1, 0]
    indices = indices[dt <= max_gap + 1e-8]
    r = right[indices]
    f = (times[indices] - a[r - 1, 0]) / (a[r, 0] - a[r - 1, 0])
    out[indices] = a[r - 1, 1:4] + f[:, None] * (a[r, 1:4] - a[r - 1, 1:4])
    return out


def statistics(errors):
    valid = errors[np.all(np.isfinite(errors), axis=1)]
    result = dict(
        samples=len(valid),
        expected_samples=len(errors),
        coverage_pct=100 * len(valid) / len(errors) if len(errors) else 0,
        rmse_m=None,
        mean_ape_m=None,
        p95_m=None,
        max_m=None,
        xyz_rmse_m=[None, None, None],
    )
    if len(valid):
        ape = np.linalg.norm(valid, axis=1)
        result.update(
            rmse_m=float(np.sqrt(np.mean(ape**2))),
            mean_ape_m=float(np.mean(ape)),
            p95_m=float(np.percentile(ape, 95)),
            max_m=float(np.max(ape)),
            xyz_rmse_m=np.sqrt(np.mean(valid**2, axis=0)).tolist(),
        )
    return result


def error_report(data):
    from analysis import crossings

    start = data["track_start"]
    report = dict(sample_hz=20, alignment="existing_map_only", periods=[], notes=[])
    if start is None:
        report["notes"].append(
            "TRACK 시작 시각이 없어 시간 대응 오차를 계산할 수 없습니다."
        )
        return report
    origin = data["origin"]
    end = origin + data["duration"]
    if data["plan"]:
        end = min(end, data["plan"][-1][0])
    seen_track = False
    for t, state in sorted(data["states"]):
        if state.startswith("TRACK"):
            seen_track = True
        elif seen_track:
            end = min(end, t + origin)
            break
    windows = [("overall", "전체 비행 · TRACK", start, max(start, end), True)]
    gates = data["course"]["gates"]
    half = data["course"]["gate"]["inner_size"] / 2
    scale = float(data["meta"]["parameters"].get("time_scale", 1))
    planned = {
        g["id"]: crossings(data["plan"], g, max_gap=max(0.3, 0.021 / scale))
        for g in gates
    }
    previous = start - 1e-5
    lap_start = start
    for lap in range(1, int(data["meta"]["parameters"].get("laps", 1)) + 1):
        if not gates:
            break
        for gate in gates:
            candidates = [
                h["t"]
                for h in planned[gate["id"]]
                if h["t"] > previous + 1e-6
                and abs(h["u"]) <= half
                and abs(h["v"]) <= half
            ]
            if not candidates:
                report["notes"].append(
                    f"Lap {lap}부터 계획 통과 순서를 확인할 수 없어 랩 통계를 생략했습니다."
                )
                break
            previous = min(candidates)
        else:
            windows.append(
                (f"lap-{lap}", f"Lap {lap}", lap_start, previous, end >= previous)
            )
            lap_start = previous
            continue
        break
    report["notes"].append(
        "20 Hz 동일 시각 보간 · 측정 공백 > 0.2 s 제외 · 사후 회전/이동/스케일 정렬 없음. Max도 평가 샘플 기준입니다."
    )
    report["notes"].append(
        "랩 경계는 계획의 마지막 게이트 통과 시각입니다. 첫 랩은 TRACK 시작부터, 이후 랩은 직전 경계부터 계산합니다. 마지막 게이트 이후 복귀는 전체에만 포함합니다. 랩 완료 표시는 실제 게이트 통과 성공을 뜻하지 않습니다."
    )
    pairs = [("raw", "actual"), ("corrected", "actual"), ("tracking", "plan")]
    for key, label, lo, hi, complete in windows:
        # Half-open intervals partition laps without counting a boundary twice.
        times = (
            start
            + np.arange(
                max(0, int(np.ceil((lo - start) * 20 - 1e-7))),
                max(0, int(np.ceil((hi - start) * 20 - 1e-7))),
            )
            / 20
        )
        sampled = {
            name: interpolate(rows, times) for name, rows in data["poses"].items()
        }
        sampled["plan"] = interpolate(data["plan"], times, max(0.2, 0.021 / scale))
        results = {}
        for name, reference in pairs:
            estimate = "actual" if name == "tracking" else name
            delta = sampled[estimate] - sampled[reference]
            delta[(times < start) | (times >= end)] = np.nan
            results[name] = statistics(delta)
            if key == "overall":
                distances = np.linalg.norm(delta, axis=1)
                data["series"][name + "_error_sync_m"] = [
                    [float(t - origin), float(v) if np.isfinite(v) else None]
                    for t, v in zip(times, distances)
                ]
        report["periods"].append(
            dict(
                id=key,
                label=label,
                start_s=lo - origin,
                end_s=hi - origin,
                complete=complete,
                recorded=end > lo,
                comparisons=results,
            )
        )
    return report
