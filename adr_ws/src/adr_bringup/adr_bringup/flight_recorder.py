"""Per-flight SQLite rosbag writer. Metadata exists before node startup.

Cache disabled so messages reach SQLite without a large in-memory bag buffer.
Each successful initialization writes a readiness file before flight starts.
A killed process leaves an honest 'recording' manifest and readable committed rows.
"""

import gc
import json
from pathlib import Path
from datetime import datetime, timezone
import time

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from adr_bringup.flight_session import atomic_json

TOPICS = {
    "/adr/vio/metrics": "diagnostic_msgs/msg/DiagnosticArray",
    "/adr/controller_state": "std_msgs/msg/String",
    "/adr/trajectory": "adr_msgs/msg/PolynomialTrajectory",
    "/adr/odom": "nav_msgs/msg/Odometry",
    "/adr/vio/odom": "nav_msgs/msg/Odometry",
    "/adr/state/corrected": "nav_msgs/msg/Odometry",
    "/ov_msckf/poseimu": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "/ov_msckf/odomimu": "nav_msgs/msg/Odometry",
    "/adr/pnp/gate": "adr_msgs/msg/GatePnP",
    "/adr/gate_detections": "adr_msgs/msg/GateDetectionArray",
    "/adr/state/drift": "geometry_msgs/msg/Vector3Stamped",
    "/adr/camera/camera_info": "sensor_msgs/msg/CameraInfo",
    "/adr/imu": "sensor_msgs/msg/Imu",
    "/clock": "rosgraph_msgs/msg/Clock",
}


class FlightRecorder(Node):
    def __init__(self):
        super().__init__("flight_recorder")
        self.declare_parameter("session_dir", "")
        self.folder = Path(self.get_parameter("session_dir").value)
        self.meta = json.loads((self.folder / "session.json").read_text())
        self.meta["topics"] = TOPICS
        self.counts = {k: 0 for k in TOPICS}
        self.closed = False
        self.completed = False
        self.close_deadline = None
        self.writer = rosbag2_py.SequentialWriter()
        try:
            self.writer.open(
                rosbag2_py.StorageOptions(
                    uri=str(self.folder / "bag"),
                    storage_id="sqlite3",
                    max_cache_size=0,
                    storage_preset_profile="resilient",
                ),
                rosbag2_py.ConverterOptions("", ""),
            )
            self.subscriptions_kept = []
            for topic, typ in TOPICS.items():
                # rosbag2_py TopicMetadata added an id argument in newer ROS releases.
                try:
                    metadata = rosbag2_py.TopicMetadata(
                        name=topic, type=typ, serialization_format="cdr"
                    )
                except TypeError:
                    metadata = rosbag2_py.TopicMetadata(
                        id=0, name=topic, type=typ, serialization_format="cdr"
                    )
                self.writer.create_topic(metadata)
                latched = topic == "/adr/trajectory"
                qos = QoSProfile(
                    depth=100,
                    reliability=ReliabilityPolicy.RELIABLE
                    if latched
                    else ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL
                    if latched
                    else DurabilityPolicy.VOLATILE,
                )
                self.subscriptions_kept.append(
                    self.create_subscription(
                        get_message(typ),
                        topic,
                        lambda msg, t=topic: self.record(t, msg),
                        qos,
                    )
                )
            self.meta.update(
                status="recording",
                recorder_started_utc=datetime.now(timezone.utc).isoformat(),
            )
            atomic_json(self.folder / "session.json", self.meta)
            atomic_json(self.folder / "ready.json", {"ready": True})
        except Exception as exc:
            self.meta.update(status="failed", error=str(exc))
            atomic_json(self.folder / "session.json", self.meta)
            raise
        self.create_timer(0.25, self.tick)
        self.get_logger().info(f"Flight recording: {self.folder}")

    def record(self, topic, msg):
        if self.closed:
            return
        if topic == "/adr/trajectory" and self.counts[topic]:
            return
        now = self.get_clock().now().nanoseconds
        try:
            self.writer.write(topic, serialize_message(msg), now)
        except Exception as exc:
            self.meta["error"] = str(exc)
            self.finish("failed")
            self.get_logger().error(f"bag write failed: {exc}")
            return
        self.counts[topic] += 1
        if topic == "/adr/controller_state":
            state = msg.data.split()[0]
            if state != self.meta.get("controller_state"):
                self.meta.update(controller_state=state, last_state_sim_ns=now)
                atomic_json(self.folder / "session.json", self.meta)
            if state == "DONE" and not self.completed:
                self.completed = True
                self.close_deadline = time.monotonic() + 1.0

    def tick(self):
        if self.close_deadline is not None and time.monotonic() >= self.close_deadline:
            self.finish("completed")

    def finish(self, status="interrupted"):
        if self.closed:
            return
        self.closed = True
        try:
            if hasattr(self.writer, "close"):
                self.writer.close()
        except Exception as exc:
            status = "failed"
            self.meta["error"] = f"bag finalization failed: {exc}"
        finally:
            self.writer = None
            gc.collect()
        self.meta.update(
            status=status,
            closed_utc=datetime.now(timezone.utc).isoformat(),
            message_counts=self.counts,
        )
        atomic_json(self.folder / "session.json", self.meta)
        self.get_logger().info(f"Flight recording closed: {status}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.finish("completed" if node.completed else "interrupted")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
