"""Local flight review. Run: python app.py --root ../../flight_logs --port 5050."""

import argparse
import json
import os
import sqlite3
from pathlib import Path
import threading

from flask import Flask, jsonify, render_template, send_file, abort
from analysis import load_flight, gate_figure, reduce_series, scene_data


def create_app(root=None):
    app = Flask(__name__)
    root = Path(
        root
        or os.environ.get(
            "ADR_FLIGHT_LOGS", Path(__file__).resolve().parents[2] / "flight_logs"
        )
    ).resolve()
    lock = threading.Lock()
    cache = {}

    def session(run):
        if not run or Path(run).name != run:
            abort(404)
        path = (root / run).resolve()
        if path.parent != root or not (path / "session.json").is_file():
            abort(404)
        return path

    def analyze(run):
        path = session(run)
        sources = [
            path / "session.json",
            *path.glob("openvins_timing.csv"),
            *(path / "bag").glob("*.db3"),
            *(path / "bag").glob("*-wal"),
        ]
        signature = tuple(
            (str(p), p.stat().st_mtime_ns, p.stat().st_size) for p in sources
        )
        with lock:
            if run in cache and cache[run][0] == signature:
                return cache[run][1]
            data = load_flight(path)
            output = path / "review"
            output.mkdir(exist_ok=True)
            temporary = output / "gate-passages.tmp.png"
            gate_figure(data, temporary)
            os.replace(temporary, output / "gate-passages.png")
            payload = {k: data[k] for k in ("meta", "duration", "warnings", "states", "error_report")}
            payload["series"] = {k: reduce_series(v) for k, v in data["series"].items()}
            payload["image_url"] = f"/api/runs/{run}/gates.png"
            payload["point_limit"] = 5000
            payload["scene"] = scene_data(data)
            cache[run] = (signature, payload)
            # Bound server memory when browsing many runs.
            while len(cache) > 4:
                cache.pop(next(iter(cache)))
            return payload

    @app.get("/")
    def home():
        return render_template("index.html")

    @app.get("/api/runs")
    def runs():
        result = []
        if root.exists():
            for path in sorted(root.iterdir(), reverse=True):
                if path.is_symlink() or not path.is_dir():
                    continue
                try:
                    m = json.loads((path / "session.json").read_text())
                    result.append(
                        {
                            k: m.get(k)
                            for k in (
                                "id",
                                "track",
                                "created_utc",
                                "status",
                                "parameters",
                            )
                        }
                    )
                except (OSError, ValueError):
                    continue
        return jsonify(result)

    @app.get("/api/runs/<run>")
    def report(run):
        try:
            return jsonify(analyze(run))
        except (ValueError, OSError, RuntimeError, sqlite3.DatabaseError) as exc:
            return jsonify(error=str(exc)), 422

    @app.get("/api/runs/<run>/gates.png")
    def gates(run):
        try:
            analyze(run)
        except (ValueError, OSError, RuntimeError, sqlite3.DatabaseError) as exc:
            return jsonify(error=str(exc)), 422
        return send_file(
            session(run) / "review/gate-passages.png", mimetype="image/png", max_age=0
        )

    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--root")
    parser.add_argument("--port", type=int, default=5050)
    args = parser.parse_args()
    create_app(args.root).run(
        host="127.0.0.1", port=args.port, debug=False, threaded=True
    )
