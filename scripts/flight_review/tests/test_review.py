import json
from pathlib import Path
import sys
import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
REPO = HERE.parents[1]
sys.path[:0] = [
    str(HERE),
    str(REPO / "adr_ws/src/adr_vio"),
    str(REPO / "adr_ws/src/adr_bringup"),
]
from analysis import (
    load_flight,
    gate_hits,
    crossings,
    reduce_series,
    gate_figure,
    topview_figure,
    scene_data,
)
from app import create_app
from adr_vio.metrics_core import MetricStore, TimingTail
from adr_bringup.flight_session import new_session, repository_root
from fixture_bag import make_bag


def test_pnp_measurements_in_scene(tmp_path):
    data = load_flight(make_bag(tmp_path, include_pnp=True))
    assert scene_data(data)["pnp"] == [[0.25, 1.0, 2.0, 3.0, 2]]
    assert "pnp" not in data["poses"]  # Sparse measurements must not become interpolated poses.


def test_old_bag_without_pnp(tmp_path):
    assert scene_data(load_flight(make_bag(tmp_path)))["pnp"] == []


def test_stale_rate_and_reset():
    store = MetricStore()
    store.put("features", 0, 10)
    store.event("camera", 1, 10)
    store.event("camera", 1.1, 10.1)
    assert store.snapshot(10.2)["features"] == 0
    assert store.snapshot(10.2)["camera_hz"] == pytest.approx(10)
    assert "features" not in store.snapshot(13)
    assert "camera_hz" not in store.snapshot(13)
    assert store.snapshot(13)["camera_age_s"] == pytest.approx(2.9)
    store.event("camera", 0, 14)
    assert "camera_hz" not in store.snapshot(14)


def test_timing_partial_line(tmp_path):
    p = tmp_path / "timing.csv"
    tail = TimingTail(str(p))
    assert tail.read() is None
    p.write_text(
        "# timestamp (sec),tracking,msckf update,total\n1,0.01,0.02,0.03\n2,0.02"
    )
    assert tail.read()["timing_total_ms"] == 30
    assert tail.read() is None
    with p.open("a") as f:
        f.write(",0.03,0.05\n")
    assert tail.read()["timing_total_ms"] == 50
    p.write_text("# timestamp (sec),total\n3,.001\n")
    assert tail.read() == {"timing_total_ms": 1}


def test_crossing_geometry_and_gaps():
    gate = {"x": 0, "y": 0, "z": 1.5, "yaw_deg": 90}
    hit = crossings([[1, 0.2, -1, 1.8], [1.1, 0.2, 1, 1.8]], gate)[0]
    assert hit["u"] == pytest.approx(0.2)
    assert hit["v"] == pytest.approx(0.3)
    assert hit["t"] == pytest.approx(1.05)
    assert crossings([[1, 0, -1, 1.5], [2, 0, 1, 1.5]], gate) == []
    assert crossings([[1, 0, 1, 1.5], [1.1, 0, -1, 1.5]], gate) == []


def test_extrema_and_gaps():
    points = [[i * 0.01, 100 if i == 4111 else 0] for i in range(10000)]
    result = reduce_series(points, 100)
    assert len(result) <= 100 and max(p[1] for p in result) == 100
    assert any(p[1] is None for p in reduce_series([[0, 1], [0.1, 2], [4, 3]]))


def test_session_exists_before_recording(tmp_path):
    a = new_session(tmp_path, "cross", {}, {}, {})
    b = new_session(tmp_path, "cross", {}, {}, {})
    assert a != b
    assert json.loads((a / "session.json").read_text())["status"] == "starting"
    assert repository_root(REPO / "adr_ws/src/adr_bringup") == REPO


def test_real_cdr_crash_bag_and_api(tmp_path):
    folder = make_bag(tmp_path, status="recording")
    assert not (folder / "bag/metadata.yaml").exists()
    data = load_flight(folder)
    hits = gate_hits(data)
    assert len(hits) == 6
    assert all(
        len(h["plan"]) == len(h["actual"]) == len(h["raw"]) == len(h["corrected"]) == 1
        for h in hits.values()
    )
    assert hits[1]["raw"][0]["u"] == pytest.approx(0.06)
    assert hits[1]["raw"][0]["v"] == pytest.approx(0.02)
    assert max(p[1] for p in data["series"]["raw_error_sync_m"]) > 0.5
    assert any("비정상" in s for s in data["warnings"])
    client = create_app(tmp_path).test_client()
    assert client.get("/").status_code == 200
    runs = client.get("/api/runs").json
    assert runs[0]["id"] == folder.name
    response = client.get("/api/runs/" + folder.name)
    assert response.status_code == 200
    assert response.json["series"]["slam_features"]
    png = client.get("/api/runs/" + folder.name + "/gates.png")
    assert png.status_code == 200 and png.data[:8] == b"\x89PNG\r\n\x1a\n"
    assert (folder / "review/gate-passages.png").exists()
    top = client.get(response.json["topview_url"])
    assert top.status_code == 200 and top.data[:8] == b"\x89PNG\r\n\x1a\n"
    assert (folder / "review/topview.png").exists()
    assert client.get("/api/runs/does-not-exist").status_code == 404
    (tmp_path / "escape").symlink_to(HERE, target_is_directory=True)
    assert client.get("/api/runs/escape").status_code == 404


@pytest.mark.parametrize('include_gt', [False, True])
def test_topview_sources_and_pnp(tmp_path, monkeypatch, include_gt):
    from matplotlib.figure import Figure
    data = load_flight(make_bag(tmp_path, include_pnp=True, include_gt=include_gt))
    assert bool(data['ground_truth']) == include_gt
    figures = []
    monkeypatch.setattr(Figure, 'savefig', lambda fig, *a, **kw: figures.append(fig))
    topview_figure(data, tmp_path/'top.png')
    ax = figures[0].axes[0]
    labels = [line.get_label() for line in ax.lines]
    assert ('Ground truth' in labels) == include_gt
    assert ('PX4 odometry (GT unavailable)' in labels) == (not include_gt)
    assert 'Raw VIO (map aligned)' in labels and 'PnP-corrected VIO' in labels
    np.testing.assert_allclose(ax.collections[0].get_offsets(), [[2, 1]])
    assert ax.get_aspect() == 1.0
    assert ax.yaxis_inverted()
    assert ax.get_xlabel() == "Y / North [m]"


def test_topview_clips_divergence_to_gate_area(tmp_path, monkeypatch):
    from matplotlib.figure import Figure
    data = load_flight(make_bag(tmp_path, include_pnp=True, include_gt=True))
    data['poses']['raw'].append([999, 10000, 20000, 1, 1, 0, 0, 0])
    data['ground_truth'].append([999, 5000, 10000, 1, 1, 0, 0, 0])
    data['pnp'].append([999, -10000, -10000, 1, 1])
    figures = []
    monkeypatch.setattr(Figure, 'savefig', lambda fig, *a, **kw: figures.append(fig))
    topview_figure(data, tmp_path/'top.png')
    ax = figures[0].axes[0]
    assert max(abs(v) for v in (*ax.get_xlim(), *ax.get_ylim())) < 30
    assert not ax.child_axes
    # Rendering clips only; the recorded values are preserved.
    assert data['poses']['raw'][-1][1] == 10000


def test_twelve_gate_figure(tmp_path):
    data = load_flight(make_bag(tmp_path, gates=12))
    path = tmp_path / "twelve.png"
    hits = gate_figure(data, path)
    from PIL import Image

    with Image.open(path) as im:
        assert im.height > 1500 and im.width > 2000
        assert im.convert("RGB").getpixel((0, 0)) == (255, 255, 255)
    assert len(hits) == 12


def test_full_timing_csv_preserves_spike(tmp_path):
    folder = make_bag(tmp_path)
    (folder / "openvins_timing.csv").write_text(
        "# timestamp (sec),tracking,total\n100,.01,.02\n100.01,.01,.2\n100.02,.01,.02\n200,.9,.9\n200.03,.0"
    )
    data = load_flight(folder)
    assert len(data["series"]["timing_total_ms"]) == 3
    assert max(p[1] for p in data["series"]["timing_total_ms"]) == 200


def test_planned_time_scale(tmp_path):
    folder = make_bag(tmp_path)
    meta = json.loads((folder / "session.json").read_text())
    meta["parameters"]["time_scale"] = "2"
    (folder / "session.json").write_text(json.dumps(meta))
    data = load_flight(folder)
    # At twice playback speed the first planned plane intersection is at 101, not 102.
    assert gate_hits(data)[1]["plan"][0]["t"] == pytest.approx(101)


def test_large_correction_jump_marked(tmp_path):
    data = load_flight(make_bag(tmp_path))
    data["poses"]["corrected"] = [[101.9, -1, 0, 1.5], [102.1, 1, 0, 1.5]]
    hit = gate_hits(data)[1]["corrected"][0]
    assert hit["jump"] is True


def test_tracker_full_rate_and_scene_time(tmp_path):
    folder = make_bag(tmp_path)
    data = load_flight(folder)
    klt = data["series"]["klt_features"]
    assert len(klt) > len(data["series"]["slam_features"])
    assert min(p[1] for p in klt) == 0
    assert next(p[0] for p in klt if p[1] == 0) == pytest.approx(44 / 30)
    scene = scene_data(data)
    assert scene["plan_time_aligned"] is True
    assert len(scene["trajectories"]["actual"][0]) == 8
    assert scene["trajectories"]["actual"][0][0] == 0
    assert scene["trajectories"]["plan"][0][4:] == [1.0, 0.0, 0.0, 0.0]
    assert len(scene["gates"]) == 6
    response = create_app(tmp_path).test_client().get("/api/runs/" + folder.name)
    assert response.json["scene"] == scene


def test_legacy_bag_klt_not_fabricated(tmp_path):
    data = load_flight(make_bag(tmp_path, include_tracker=False))
    assert "klt_features" not in data["series"]
    assert any("KLT 수치가 없습니다" in w for w in data["warnings"])
    data["track_start"] = None
    assert scene_data(data)["plan_time_aligned"] is False


def test_native_association_preserves_rejection_and_absent_runner(tmp_path):
    folder = make_bag(tmp_path, include_association=True)
    data = load_flight(folder)
    series = data['series']
    assert series['pnp_assoc_reason'][44][1] == 5
    assert series['pnp_assoc_selected_id'][44][1] == 0
    assert series['pnp_assoc_second_px'][45][1] is None
    assert series['pnp_assoc_second_px'][46][1] == 25


def test_rl_passages_ignore_planner_timetable(tmp_path):
    data = load_flight(make_bag(tmp_path))
    data["meta"]["parameters"]["controller"] = "a2rl"
    data["plan"] = [[r[0] + 100, *r[1:]] for r in data["plan"]]
    hits = gate_hits(data)
    assert all(len(h["actual"]) == len(h["raw"]) == len(h["corrected"]) == 1
               for h in hits.values())
    data["plan"] = []
    assert all(len(h["actual"]) == 1 for h in gate_hits(data).values())
