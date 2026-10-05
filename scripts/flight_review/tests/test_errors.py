import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from errors import interpolate, statistics, error_report
from analysis import reduce_series


def test_known_offset_and_missing():
    s = statistics(np.array([[3.0, 4.0, 12.0], [3.0, 4.0, 12.0], [np.nan] * 3]))
    assert s["rmse_m"] == s["mean_ape_m"] == s["p95_m"] == s["max_m"] == 13
    assert s["xyz_rmse_m"] == [3, 4, 12]
    assert s["coverage_pct"] == pytest.approx(200 / 3)
    assert statistics(np.empty((0, 3)))["rmse_m"] is None


def test_interpolation_endpoints_and_gap():
    rows = [[0, 0, 0, 0], [0.1, 1, 0, 0], [1, 2, 0, 0]]
    a = interpolate(rows, np.array([-0.1, 0, 0.05, 0.1, 0.5, 1, 2]))
    assert a[[1, 2, 3, 5], 0].tolist() == [0, 0.5, 1, 2]
    assert np.isnan(a[[0, 4, 6]]).all()


def sample_data(end=4):
    times = np.arange(0, 4.01, 0.05)
    # Two forward passes through x=0 at t=1 and t=3.
    plan = [[float(t), float(-np.cos(np.pi * t / 2)), 0, 0] for t in times]
    actual = [r for r in plan if r[0] <= end]
    return dict(
        track_start=0.0,
        origin=0.0,
        duration=end,
        states=[[0, "TRACK t_traj=0"]],
        plan=plan,
        poses=dict(
            actual=actual,
            raw=[[t, x + 3, y + 4, z] for t, x, y, z in actual],
            corrected=actual,
        ),
        course=dict(
            gates=[dict(id=1, x=0, y=0, z=0, yaw_deg=0)], gate=dict(inner_size=2)
        ),
        meta=dict(parameters=dict(laps=1, time_scale=1)),
        series={},
    )


def test_report_alignment_and_partial_lap():
    d = sample_data(0.5)
    r = error_report(d)
    full, lap = r["periods"]
    assert full["comparisons"]["raw"]["rmse_m"] == pytest.approx(5)
    assert full["comparisons"]["corrected"]["rmse_m"] == 0
    assert not lap["complete"]
    assert lap["end_s"] == pytest.approx(1)
    assert lap["comparisons"]["raw"]["coverage_pct"] == 50
    assert d["series"]["tracking_error_sync_m"][0] == [0, 0]


def test_multiple_laps_and_track_end():
    d = sample_data(8)
    times = np.arange(0, 8.01, 0.05)
    d["plan"] = [[float(t), float(-np.cos(np.pi * t / 2)), 0, 0] for t in times]
    d["poses"] = {k: d["plan"] for k in ("actual", "raw", "corrected")}
    d["meta"]["parameters"]["laps"] = 2
    d["states"].append([6, "HOLD"])
    r = error_report(d)
    assert [(p["start_s"], round(p["end_s"])) for p in r["periods"]] == [
        (0, 6),
        (0, 1),
        (1, 5),
    ]
    assert r["periods"][0]["comparisons"]["raw"]["expected_samples"] == 120
    assert r["periods"][2]["comparisons"]["raw"]["samples"] == 80


def test_no_start_and_explicit_chart_gaps():
    d = sample_data()
    d["track_start"] = None
    assert error_report(d)["periods"] == []
    points = [[i / 20, None if i == 21 else float(i)] for i in range(100)]
    assert [1.05, None] in reduce_series(points, limit=10)


def test_percentile_and_uniform_rate_weighting():
    s = statistics(np.array([[0.0, 0, 0], [0.0, 0, 0], [0.0, 0, 0], [4.0, 0, 0]]))
    assert s["rmse_m"] == 2
    assert s["p95_m"] == pytest.approx(3.4)
    d = sample_data()
    # Irregular source timestamps still evaluate on the same 20 Hz clock.
    d["poses"]["raw"] = [
        [float(t), 3 + float(t), 4, 0] for t in np.arange(0, 4.01, 0.1)
    ]
    d["poses"]["actual"] = [
        [float(t), float(t), 0, 0] for t in np.arange(0, 4.01, 0.025)
    ]
    s = error_report(d)["periods"][0]["comparisons"]["raw"]
    assert s["rmse_m"] == pytest.approx(5)
    assert s["coverage_pct"] == 100


def test_bag_api_error_report(tmp_path):
    from fixture_bag import make_bag
    from app import create_app

    folder = make_bag(tmp_path)
    response = create_app(tmp_path).test_client().get("/api/runs/" + folder.name)
    assert response.status_code == 200
    payload = response.get_json()
    report = payload["error_report"]
    assert len(report["periods"]) == 2
    assert report["periods"][1]["end_s"] == pytest.approx(17)
    assert report["periods"][0]["comparisons"]["tracking"]["rmse_m"] > 0
    assert payload["series"]["tracking_error_sync_m"]
