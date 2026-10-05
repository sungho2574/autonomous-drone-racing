"""Small real CDR/SQLite fixture; deliberately has no rosbag metadata.yaml."""

import json
from pathlib import Path
import sqlite3
import sys
import numpy as np
import yaml
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "adr_ws/src/adr_bringup"))
from adr_bringup.flight_session import new_session, atomic_json


def make_bag(root, gates=6, status="completed", include_tracker=True, include_association=False):
    course = {
        "gate": {"inner_size": 1.5, "outer_size": 2.1},
        "gates": [
            {"id": i + 1, "x": float(i * 3), "y": 0.0, "z": 1.5, "yaw_deg": 0.0}
            for i in range(gates)
        ],
    }
    defs = {
        "adr_msgs/msg/" + p.stem: p.read_text()
        for p in (REPO / "adr_ws/src/adr_msgs/msg").glob("*.msg")
    }
    folder = new_session(
        root,
        "DEMO — synthetic",
        {"laps": "1", "time_scale": "1", "vio_profile": "slam_dense"},
        {"map": {"path": "synthetic", "text": yaml.safe_dump(course)}},
        defs,
    )
    meta = json.loads((folder / "session.json").read_text())
    meta.update(status=status, synthetic=True)
    atomic_json(folder / "session.json", meta)
    store = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name, definition in defs.items():
        types.update(get_types_from_msg(definition, name))
    store.register(types)

    def msg(name, *args):
        return store.types[name](*args)

    def header(t):
        ns = round(t * 1e9)
        return msg(
            "std_msgs/msg/Header",
            msg("builtin_interfaces/msg/Time", ns // 10**9, ns % 10**9),
            "map",
        )

    folder.joinpath("bag").mkdir()
    db = sqlite3.connect(folder / "bag/bag_0.db3")
    db.executescript(
        "CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT); CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);"
    )
    topics = {}

    def write(topic, m, t):
        if topic not in topics:
            topics[topic] = len(topics) + 1
            db.execute(
                "INSERT INTO topics VALUES(?,?,?,?,?)",
                (topics[topic], topic, m.__msgtype__, "cdr", ""),
            )
        db.execute(
            "INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
            (
                topics[topic],
                round(t * 1e9),
                bytes(store.serialize_cdr(m, m.__msgtype__)),
            ),
        )

    write(
        "/adr/controller_state", msg("std_msgs/msg/String", "TRACK t_traj=0.00"), 100.0
    )
    duration = gates * 3 + 2.0
    seg = msg(
        "adr_msgs/msg/PolynomialSegment",
        duration,
        np.array([-2.0, 1.0]),
        np.array([0.0]),
        np.array([1.5]),
        np.array([0.0]),
    )
    write(
        "/adr/trajectory",
        msg("adr_msgs/msg/PolynomialTrajectory", header(100), [seg]),
        100,
    )
    for i in range(int(duration * 30) + 1):
        t = i / 30.0
        x = t - 2
        for topic, dy, dz in [
            ("/adr/odom", 0.08 * np.sin(t), 0.02),
            ("/adr/vio/odom", -0.03 * t, 0.01 * t),
            ("/adr/state/corrected", 0.09 * np.cos(t), 0.04),
        ]:
            pose = msg(
                "geometry_msgs/msg/Pose",
                msg("geometry_msgs/msg/Point", x, dy, 1.5 + dz),
                msg("geometry_msgs/msg/Quaternion", 0.0, 0.0, 0.0, 1.0),
            )
            vec = lambda: msg("geometry_msgs/msg/Vector3", 1.0, 0.0, 0.0)
            odom = msg(
                "nav_msgs/msg/Odometry",
                header(100 + t),
                "base_link",
                msg("geometry_msgs/msg/PoseWithCovariance", pose, np.zeros(36)),
                msg(
                    "geometry_msgs/msg/TwistWithCovariance",
                    msg("geometry_msgs/msg/Twist", vec(), vec()),
                    np.zeros(36),
                ),
            )
            write(topic, odom, 100 + t)
        if include_association:
            values = {"pnp_assoc_selected_id": 0 if i == 44 else 6,
                      "pnp_assoc_reason": 5 if i == 44 else 0,
                      "pnp_assoc_accepted": int(i != 44), "pnp_assoc_best_px": 6.0}
            if i != 45:
                values["pnp_assoc_second_px"] = 7.0 if i == 44 else 25.0
            diagnostic = msg("diagnostic_msgs/msg/DiagnosticStatus", 1 if i == 44 else 0,
                             "gate_association", "ambiguous" if i == 44 else "accepted", "synthetic",
                             [msg("diagnostic_msgs/msg/KeyValue", k, str(v)) for k,v in values.items()])
            write("/adr/pnp/association", msg("diagnostic_msgs/msg/DiagnosticArray", header(100+t), [diagnostic]), 100+t)
        if include_tracker:
            # One-frame loss between 10 Hz aggregate ticks must survive in the viewer.
            count = 0 if i == 44 else 250 + i % 30
            kv = [
                msg("diagnostic_msgs/msg/KeyValue", k, str(v))
                for k, v in {
                    "klt_features": count,
                    "klt_observations": count,
                    "tracker_active_features": count,
                    "tracker_is_klt": 1,
                }.items()
            ]
            tracking = msg(
                "diagnostic_msgs/msg/DiagnosticStatus",
                0,
                "openvins/tracker",
                "demo",
                "synthetic",
                kv,
            )
            write(
                "/ov_msckf/tracking_metrics",
                msg("diagnostic_msgs/msg/DiagnosticArray", header(100 + t), [tracking]),
                100 + t,
            )
        if i % 3 == 0:
            values = {
                "slam_features": 70 + 20 * np.sin(t / 3),
                "slam_capacity": 100,
                "slam_utilization_pct": 70 + 20 * np.sin(t / 3),
                "msckf_update_features": 20 + 10 * np.cos(t),
                "loop_triangulated_features": 50 + 20 * np.sin(t),
                "camera_hz": 30,
                "imu_hz": 250,
                "raw_vio_hz": 30,
                "raw_vio_lag_s": 0.03 + 0.02 * np.sin(t),
                "raw_vio_age_s": 0.02,
                "timing_tracking_ms": 8 + 3 * np.sin(t),
                "timing_total_ms": 20 + 7 * np.sin(t),
                "pnp_quality": 0.8,
                "pnp_reproj_px": 1 + np.sin(t) ** 2,
                "vio_sigma_x": 0.05 + t * 0.001,
            }
            kv = [
                msg("diagnostic_msgs/msg/KeyValue", k, str(v))
                for k, v in values.items()
            ]
            statusmsg = msg(
                "diagnostic_msgs/msg/DiagnosticStatus",
                0,
                "openvins",
                "demo",
                "synthetic",
                kv,
            )
            write(
                "/adr/vio/metrics",
                msg(
                    "diagnostic_msgs/msg/DiagnosticArray", header(100 + t), [statusmsg]
                ),
                100 + t,
            )
    db.commit()
    db.close()
    return folder


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--gates", type=int, default=6)
    args = parser.parse_args()
    print(make_bag(args.root, args.gates))
