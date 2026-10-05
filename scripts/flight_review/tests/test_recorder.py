"""Lifecycle contract with a fake DDS/writer boundary; real ROS is tested on-flight."""

import importlib.util
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "adr_ws/src/adr_bringup"))
from adr_bringup.flight_session import new_session


def recorder(monkeypatch, tmp_path, fail_open=False):
    folder = new_session(tmp_path, "cross", {}, {}, {})

    class Node:
        def __init__(self, *a):
            pass

        def declare_parameter(self, *a):
            pass

        def get_parameter(self, *a):
            return SimpleNamespace(value=str(folder))

        def create_subscription(self, typ, topic, callback, qos):
            return (topic, callback, qos)

        def create_timer(self, *a):
            pass

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1234))

        def get_logger(self):
            return SimpleNamespace(info=lambda *a: None, error=lambda *a: None)

    class Writer:
        def __init__(self):
            self.rows = []
            self.closed = False

        def open(self, storage, converter):
            assert not (folder / "ready.json").exists()
            assert (
                json.loads((folder / "session.json").read_text())["status"]
                == "starting"
            )
            assert (
                storage.storage_preset_profile == "resilient"
                and storage.max_cache_size == 0
            )
            if fail_open:
                raise RuntimeError("disk unavailable")

        def create_topic(self, *a):
            pass

        def write(self, *args):
            self.rows.append(args)

        def close(self):
            self.closed = True

    modules = {
        "rclpy": {},
        "rclpy.node": {"Node": Node},
        "rclpy.serialization": {"serialize_message": lambda m: b"cdr"},
        "rclpy.qos": {
            "QoSProfile": lambda **kw: SimpleNamespace(**kw),
            "ReliabilityPolicy": SimpleNamespace(RELIABLE=1, BEST_EFFORT=2),
            "DurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=1, VOLATILE=2),
        },
        "rosidl_runtime_py": {},
        "rosidl_runtime_py.utilities": {"get_message": lambda typ: object},
        "rosbag2_py": {
            "SequentialWriter": Writer,
            "StorageOptions": lambda **kw: SimpleNamespace(**kw),
            "ConverterOptions": lambda *a: None,
            "TopicMetadata": lambda **kw: kw,
        },
    }
    for name, attrs in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    spec = importlib.util.spec_from_file_location(
        "test_flight_recorder",
        REPO / "adr_ws/src/adr_bringup/adr_bringup/flight_recorder.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, folder


def test_ready_and_done(monkeypatch, tmp_path):
    module, folder = recorder(monkeypatch, tmp_path)
    node = module.FlightRecorder()
    writer = node.writer
    assert (folder / "ready.json").is_file()
    latched = next(q for t, cb, q in node.subscriptions_kept if t == "/adr/trajectory")
    assert latched.durability == 1 and latched.reliability == 1
    node.record("/adr/trajectory", object())
    node.record("/adr/trajectory", object())
    assert node.counts["/adr/trajectory"] == 1
    node.record("/adr/controller_state", SimpleNamespace(data="DONE t_traj=4.2"))
    assert (
        json.loads((folder / "session.json").read_text())["controller_state"] == "DONE"
    )
    node.close_deadline = 0
    node.tick()
    assert writer.closed
    manifest = json.loads((folder / "session.json").read_text())
    assert (
        manifest["status"] == "completed"
        and manifest["message_counts"]["/adr/trajectory"] == 1
    )
    node.record("/adr/trajectory", object())
    node.finish()
    assert len(writer.rows) == 2


def test_startup_failure_never_ready(monkeypatch, tmp_path):
    import pytest

    module, folder = recorder(monkeypatch, tmp_path, fail_open=True)
    with pytest.raises(RuntimeError, match="disk unavailable"):
        module.FlightRecorder()
    assert not (folder / "ready.json").exists()
    assert json.loads((folder / "session.json").read_text())["status"] == "failed"


def test_interrupted_and_write_failure(monkeypatch, tmp_path):
    module, folder = recorder(monkeypatch, tmp_path)
    node = module.FlightRecorder()

    def fail(*args):
        raise RuntimeError("disk full")

    node.writer.write = fail
    node.record("/adr/odom", object())
    manifest = json.loads((folder / "session.json").read_text())
    assert manifest["status"] == "failed" and manifest["error"] == "disk full"
    assert manifest["message_counts"]["/adr/odom"] == 0
