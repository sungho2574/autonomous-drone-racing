"""Read SQLite ROS2 bags without a ROS installation or bag metadata.yaml.

Always use read-only connections. Committed rows of a crashed/active SQLite bag
can be analyzed, including WAL. Header stamps define sample time; bag timestamps
are used only for headerless controller messages.
"""

import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import yaml
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

POSE_TOPICS = {
    "/adr/odom": "actual",
    "/adr/vio/odom": "raw",
    "/adr/state/corrected": "corrected",
}
COLORS = {
    "plan": "#81c784",
    "actual": "#ff6b6b",
    "corrected": "#4ba3ff",
    "raw": "#bd94ff",
}
LABELS = {
    "plan": "Plan",
    "actual": "Actual",
    "corrected": "PnP-corrected VIO",
    "raw": "Raw VIO (map aligned)",
}


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def bag_rows(folder):
    files = sorted((folder / "bag").glob("*.db3"))
    if not files:
        raise ValueError("SQLite bag가 아직 없거나 이 기록은 SQLite 형식이 아닙니다.")
    wanted = set(POSE_TOPICS) | {
        "/adr/vio/metrics",
        "/ov_msckf/tracking_metrics",
        "/adr/controller_state",
        "/adr/trajectory",
    }
    for path in files:
        with sqlite3.connect(
            path.resolve().as_uri() + "?mode=ro", uri=True, timeout=2
        ) as db:
            topics = {
                row[0]: (row[1], row[2])
                for row in db.execute("SELECT id,name,type FROM topics")
                if row[1] in wanted
            }
            if not topics:
                continue
            sql = (
                "SELECT topic_id,timestamp,data FROM messages WHERE topic_id IN ("
                + ",".join("?" * len(topics))
                + ") ORDER BY timestamp,id"
            )
            for tid, t, data in db.execute(sql, list(topics)):
                yield (topics[tid][0], topics[tid][1], t * 1e-9, data)


def clean_rows(rows):
    if not rows:
        return []
    # Last sample wins at duplicate stamps; keep stamp order for interpolation.
    return [v for _, v in sorted({row[0]: row for row in rows}.items())]


def load_flight(folder):
    meta = json.loads((folder / "session.json").read_text())
    store = get_typestore(Stores.ROS2_HUMBLE)
    defs = {}
    for typ, text in meta.get("message_definitions", {}).items():
        defs.update(get_types_from_msg(text, typ))
    if defs:
        store.register(defs)
    metrics = []
    tracker_metrics = []
    poses = {k: [] for k in ("actual", "raw", "corrected")}
    states = []
    poly = None
    warnings = []
    for topic, typ, t, data in bag_rows(folder):
        try:
            msg = store.deserialize_cdr(data, typ)
        except Exception as exc:
            note = f"{topic}: deserialize failed ({type(exc).__name__})"
            if note not in warnings:
                warnings.append(note)
            continue
        if topic in POSE_TOPICS:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            row = [stamp(msg.header), p.x, p.y, p.z, q.w, q.x, q.y, q.z]
            if all(math.isfinite(float(x)) for x in row):
                poses[POSE_TOPICS[topic]].append(row)
        elif topic in ("/adr/vio/metrics", "/ov_msckf/tracking_metrics"):
            values = {}
            for status in msg.status:
                for kv in status.values:
                    try:
                        v = float(kv.value)
                        if math.isfinite(v):
                            values[kv.key] = v
                    except ValueError:
                        pass
            destination = metrics if topic == "/adr/vio/metrics" else tracker_metrics
            destination.append((stamp(msg.header), values))
        elif topic == "/adr/controller_state":
            states.append((t, msg.data))
        elif topic == "/adr/trajectory" and poly is None:
            poly = msg
    poses = {k: clean_rows(v) for k, v in poses.items()}
    metrics = sorted({t: v for t, v in metrics}.items())
    source_times = [r[0] for rows in poses.values() for r in rows]
    source_times += [t for t, _ in metrics] + [t for t, _ in tracker_metrics]
    if not source_times:
        raise ValueError(
            "기록에서 pose/metrics 샘플을 찾지 못했습니다. 녹화 시작 전이거나 데이터가 없습니다."
        )
    origin = min(source_times)
    end_time = max(source_times)
    if "map" not in meta.get("configs", {}):
        raise ValueError("시작 시 저장된 map snapshot이 없습니다.")
    course = yaml.safe_load(meta["configs"]["map"]["text"])
    scale = float(meta["parameters"].get("time_scale", 1))
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid time_scale in manifest")
    start = None
    for t, state in states:
        if state.startswith("TRACK "):
            try:
                start = t - float(state.split("t_traj=")[1].split()[0]) / scale
            except (ValueError, IndexError):
                continue
            break
    plan = []
    if poly is not None:

        def segment_pose(seg, dt):
            xyz = [
                float(np.polynomial.polynomial.polyval(dt, getattr(seg, "c" + axis)))
                for axis in "xyz"
            ]
            yaw = (
                float(np.polynomial.polynomial.polyval(dt, seg.cyaw))
                if len(seg.cyaw)
                else None
            )
            # A plan supplies yaw, not measured roll/pitch. Display yaw-only axes.
            q = (
                [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
                if yaw is not None
                else [0.0, 0.0, 0.0, 0.0]
            )
            return [*xyz, *q]

        offset = 0.0
        for seg in poly.segments:
            for dt in np.arange(0, seg.duration, 0.02):
                plan.append([offset + dt, *segment_pose(seg, dt)])
            offset += seg.duration
        if poly.segments:
            seg = poly.segments[-1]
            plan.append([offset, *segment_pose(seg, seg.duration)])
        if start is not None:
            plan = [[start + r[0] / scale, *r[1:]] for r in plan]
        else:
            warnings.append(
                "TRACK 시작 시각 없음: 계획 통과점은 계산하지만 다른 궤적과 시간 대응은 하지 않습니다."
            )
    else:
        warnings.append(
            "계획 다항식이 bag에 없습니다. 녹화 준비/latched QoS를 확인하세요."
        )
    series = {}
    for t, values in metrics:
        for k, v in values.items():
            series.setdefault(k, []).append([t - origin, v])
    # Prefer camera-stamped, per-frame tracker values over 10 Hz aggregate samples.
    tracker_series = {}
    for t, values in tracker_metrics:
        for key, value in values.items():
            tracker_series.setdefault(key, []).append([t - origin, value])
    series.update({key: clean_rows(rows) for key, rows in tracker_series.items()})
    if not series.get("klt_features"):
        warnings.append(
            "KLT 수치가 없습니다. 이전 기록에는 소급 추가할 수 없습니다. 새 비행은 OpenVINS tracker-metrics 패치 적용/재빌드 및 use_klt 설정을 확인하세요."
        )
    # Full per-update CSV preserves timing spikes between the 10 Hz aggregate messages.
    timing_path = folder / "openvins_timing.csv"
    if timing_path.is_file():
        columns = []
        timing = {}
        with timing_path.open() as stream:
            for line in stream:
                if not line.endswith("\n"):
                    break
                if line.startswith("#"):
                    columns = [
                        x.strip().replace(" ", "_").replace("&", "and")
                        for x in line[1:].split(",")
                    ]
                    continue
                try:
                    numbers = [float(v) for v in line.split(",")]
                except ValueError:
                    continue
                if len(numbers) != len(columns) or not all(
                    math.isfinite(v) for v in numbers
                ):
                    continue
                if not origin <= numbers[0] <= end_time:
                    continue
                for key, value in zip(columns[1:], numbers[1:]):
                    timing.setdefault("timing_" + key + "_ms", []).append(
                        [numbers[0] - origin, value * 1000]
                    )
        series.update({k: clean_rows(v) for k, v in timing.items()})
    for name, rows in poses.items():
        for axis, index in zip("xyz", (1, 2, 3)):
            series[name + "_" + axis + "_m"] = [[r[0] - origin, r[index]] for r in rows]
    if start is not None:
        for axis, index in zip("xyz", (1, 2, 3)):
            series["plan_" + axis + "_m"] = [[r[0] - origin, r[index]] for r in plan]
    for name, rows in poses.items():
        if not rows:
            warnings.append(f"{LABELS[name]} pose 데이터가 없습니다.")
    if not metrics:
        warnings.append(
            "/adr/vio/metrics 데이터가 없습니다. VIO 비활성화 또는 노드/토픽 상태를 확인하세요."
        )
    if meta.get("synthetic"):
        warnings.append(
            "SYNTHETIC DEMO: UI/분석 검증용 합성 데이터이며 실제 비행이 아닙니다."
        )
    if meta["status"] in ("recording", "starting"):
        warnings.append(
            "종료 메타데이터가 없는 기록입니다. 진행 중이거나 비정상 종료됐을 수 있습니다. 저장된 구간만 표시합니다."
        )
    data = dict(
        meta=meta,
        course=course,
        poses=poses,
        plan=plan,
        origin=origin,
        track_start=start,
        series=series,
        warnings=warnings,
        duration=max(source_times) - origin,
        states=[[t - origin, s] for t, s in states],
    )

    from errors import error_report
    data["error_report"] = error_report(data)
    return data


def crossings(rows, gate, max_gap=0.3):
    """Negative→positive gate-plane crossing; linear interpolation within a sample pair.

    No projection of nearest points: trajectories that never cross have no hit.
    Zero-plane samples are counted once; discontinuities/time gaps are rejected.
    """
    if len(rows) < 2:
        return []
    a = np.asarray(rows, float)
    p = a[:, 1:4]
    center = np.array([gate[k] for k in "xyz"])
    yaw = math.radians(gate["yaw_deg"])
    n = np.array([math.cos(yaw), math.sin(yaw), 0])
    right = np.array([math.sin(yaw), -math.cos(yaw), 0])
    d = (p - center) @ n
    result = []
    for i in np.flatnonzero((d[:-1] < 0) & (d[1:] >= 0)):
        dt = a[i + 1, 0] - a[i, 0]
        if dt <= 0 or dt > max_gap:
            continue
        f = -d[i] / (d[i + 1] - d[i])
        hit = p[i] + f * (p[i + 1] - p[i])
        relative = hit - center
        result.append(
            dict(
                t=float(a[i, 0] + f * dt),
                u=float(relative @ right),
                v=float(relative[2]),
                jump_m=float(np.linalg.norm(p[i + 1] - p[i])),
            )
        )
    return result


def gate_hits(data):
    gates = data["course"]["gates"]
    laps = max(1, int(data["meta"]["parameters"].get("laps", 1)))
    expected = []
    previous = -float("inf")
    # Ordered planned passes distinguish a nearby gate's infinite plane and return leg.
    planned = {g["id"]: crossings(data["plan"], g, max_gap=1.0) for g in gates}
    for _ in range(laps):
        for gate in gates:
            half = data["course"]["gate"]["inner_size"] / 2
            candidates = [
                h
                for h in planned[gate["id"]]
                if h["t"] > previous + 1e-5
                and abs(h["u"]) <= half
                and abs(h["v"]) <= half
            ]
            if not candidates:
                continue
            h = min(candidates, key=lambda x: x["t"])
            previous = h["t"]
            expected.append((gate["id"], h))
    out = {g["id"]: {k: [] for k in COLORS} for g in gates}
    for gid, hit in expected:
        out[gid]["plan"].append(hit)
    if data["track_start"] is None:
        return out
    observed = {
        (g["id"], name): crossings(rows, g)
        for g in gates
        for name, rows in data["poses"].items()
    }
    for k, (gid, planhit) in enumerate(expected):
        lo = (expected[k - 1][1]["t"] + planhit["t"]) / 2 if k else data["track_start"]
        hi = (
            (expected[k + 1][1]["t"] + planhit["t"]) / 2
            if k + 1 < len(expected)
            else planhit["t"] + 2
        )
        for name, rows in data["poses"].items():
            hits = [h for h in observed[gid, name] if lo <= h["t"] <= hi]
            if hits:
                hit = min(hits, key=lambda x: abs(x["t"] - planhit["t"]))
                # Large correction jumps are marked, not presented as physical passage.
                hit["jump"] = hit["jump_m"] > 0.75
                out[gid][name].append(hit)
    return out


def gate_figure(data, destination):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    gates = data["course"]["gates"]
    hits = gate_hits(data)
    cols = 4
    rows = math.ceil(len(gates) / cols)
    with plt.style.context("default"):
        fig, axes = plt.subplots(rows, cols, figsize=(16, rows * 3.7), squeeze=False)
        fig.patch.set_facecolor("white")
        inner = data["course"]["gate"]["inner_size"]
        outer = data["course"]["gate"]["outer_size"]
        all_offsets = [
            abs(h[axis])
            for group in hits.values()
            for points in group.values()
            for h in points
            for axis in ("u", "v")
        ]
        bound = max(outer / 2 + 0.35, max(all_offsets, default=0) * 1.08)
        for ax, gate in zip(axes.flat, gates):
            ax.set_facecolor("white")
            ax.add_patch(
                Rectangle(
                    (-outer / 2, -outer / 2),
                    outer,
                    outer,
                    fill=False,
                    ec="#ed9b49",
                    lw=2,
                )
            )
            ax.add_patch(
                Rectangle(
                    (-inner / 2, -inner / 2),
                    inner,
                    inner,
                    fill=False,
                    ec="#ed9b49",
                    lw=1,
                    ls="--",
                )
            )
            print_colors = {
                "plan": "#2e7d32",
                "actual": "#d32f2f",
                "corrected": "#1565c0",
                "raw": "#7b1fa2",
            }
            for name, color in print_colors.items():
                points = hits[gate["id"]][name]
                good = [h for h in points if not h.get("jump")]
                jumps = [h for h in points if h.get("jump")]
                ax.scatter(
                    [h["u"] for h in good],
                    [h["v"] for h in good],
                    s=34,
                    c=color,
                    label=f"{LABELS[name]} ({len(good)})",
                    alpha=0.9,
                )
                if jumps:
                    ax.scatter(
                        [h["u"] for h in jumps],
                        [h["v"] for h in jumps],
                        s=48,
                        c=color,
                        marker="x",
                        label="Correction jump",
                    )
            ax.axhline(0, color="#526073", lw=0.5)
            ax.axvline(0, color="#526073", lw=0.5)
            ax.set(
                xlim=(-bound, bound),
                ylim=(-bound, bound),
                xlabel="Gate-right [m]",
                ylabel="Up [m]",
                title=f"G{gate['id']} · front view",
            )
            ax.set_aspect("equal")
            ax.grid(alpha=0.12)
            ax.legend(fontsize=6, loc="upper right")
        for ax in list(axes.flat)[len(gates) :]:
            ax.set_visible(False)
        fig.suptitle(
            f"{data['meta']['track']} · gate passage offsets\nPlane intersections near planned passage time; × = discontinuous correction",
            fontsize=14,
        )
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(destination, dpi=145, facecolor=fig.get_facecolor())
        plt.close(fig)
    return hits


def reduce_series(points, limit=5000):
    """Keep extrema and explicit gap markers, so downsampling cannot hide outages."""
    if not points:
        return []
    explicit = [p for p in points if p[1] is None]
    points = [p for p in points if p[1] is not None]
    if not points:
        return explicit[:1] + explicit[-1:]
    gaps = explicit + [
        [(a[0] + b[0]) / 2, None]
        for a, b in zip(points, points[1:])
        if b[0] - a[0] > 0.5
    ]
    if len(points) <= limit:
        result = list(points)
    else:
        result = [points[0]]
        for chunk in np.array_split(np.asarray(points), max(1, (limit - 2) // 2)):
            indices = sorted(
                set([int(np.argmin(chunk[:, 1])), int(np.argmax(chunk[:, 1]))])
            )
            result.extend(chunk[i].tolist() for i in indices)
        result.append(points[-1])
    return sorted({p[0]: p for p in result + gaps}.values(), key=lambda p: p[0])


def scene_data(data):
    """Full pose samples for accurate hover interpolation, in the same time base as charts."""
    origin = data["origin"]
    trajectories = {
        name: [[r[0] - origin, *r[1:]] for r in rows]
        for name, rows in data["poses"].items()
    }
    aligned = data["track_start"] is not None
    trajectories["plan"] = [
        [r[0] - origin if aligned else r[0], *r[1:]] for r in data["plan"]
    ]
    return {
        "gates": data["course"]["gates"],
        "gate": data["course"]["gate"],
        "trajectories": trajectories,
        "plan_time_aligned": aligned,
        "plan_max_gap_s": max(
            0.3, 0.021 / float(data["meta"]["parameters"].get("time_scale", 1))
        ),
        "initial_time": data["track_start"] - origin if aligned else 0.0,
    }
