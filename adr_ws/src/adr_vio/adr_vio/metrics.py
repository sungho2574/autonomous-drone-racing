"""Public OpenVINS observables → /adr/vio/metrics (DiagnosticArray, 10 Hz).

Cloud counts have distinct semantics: in-state SLAM, last MSCKF update, and
triangulated loop features. Patched tracker diagnostics separately report KLT IDs.
Missing/stale values are omitted, never silently reported as zero.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import PointCloud2, PointCloud, Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped
from std_msgs.msg import Float32
from adr_msgs.msg import GateDetectionArray, GatePnP


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


from adr_vio.metrics_core import MetricStore, TimingTail


class VioMetrics(Node):
    def __init__(self):
        super().__init__("vio_metrics")
        self.declare_parameter("timing_path", "")
        self.declare_parameter("slam_capacity", 50)
        self.store = MetricStore()
        self.tail = TimingTail(self.get_parameter("timing_path").value)
        self.pub = self.create_publisher(DiagnosticArray, "/adr/vio/metrics", 10)
        self.subs = []

        def sub(typ, topic, callback):
            self.subs.append(
                self.create_subscription(typ, topic, callback, qos_profile_sensor_data)
            )

        sub(
            PointCloud2,
            "/ov_msckf/points_slam",
            lambda m: self.cloud("slam_features", m),
        )
        sub(
            PointCloud2,
            "/ov_msckf/points_msckf",
            lambda m: self.cloud("msckf_update_features", m),
        )
        sub(
            PointCloud,
            "/ov_msckf/loop_feats",
            lambda m: self.point("loop_triangulated_features", len(m.points), m.header),
        )
        sub(DiagnosticArray, "/ov_msckf/tracking_metrics", self.tracking)
        sub(Image, "/adr/camera/image_raw", lambda m: self.rate("camera", m.header))
        sub(Imu, "/adr/imu", lambda m: self.rate("imu", m.header))
        sub(Odometry, "/adr/vio/odom", lambda m: self.odom("raw_vio", m))
        sub(Odometry, "/adr/state/corrected", lambda m: self.odom("corrected", m))
        sub(Odometry, "/adr/odom", lambda m: self.odom("actual", m))
        sub(PoseWithCovarianceStamped, "/ov_msckf/poseimu", self.covariance)
        sub(Float32, "/adr/vio/error", lambda m: self.put("raw_error_online_m", m.data))
        sub(
            Float32,
            "/adr/state/error",
            lambda m: self.put("corrected_error_online_m", m.data),
        )
        sub(Vector3Stamped, "/adr/state/drift", self.drift)
        sub(GateDetectionArray, "/adr/gate_detections", self.detections)
        sub(DiagnosticArray, "/adr/pnp/association", self.association)
        sub(GatePnP, "/adr/pnp/gate", self.pnp)
        self.create_timer(0.1, self.publish)

    def association(self, msg):
        # Do not carry the previous frame's runner-up into a one-candidate frame.
        for key in list(self.store.values):
            if key.startswith('pnp_assoc_'):
                self.store.values.pop(key)
        for status in msg.status:
            for item in status.values:
                if item.key.startswith('pnp_assoc_'):
                    try:
                        self.put(item.key, float(item.value))
                    except ValueError:
                        pass

    def tracking(self, msg):
        self.rate("tracker", msg.header)
        allowed = {
            "klt_features",
            "klt_observations",
            "tracker_is_klt",
            "tracker_active_features",
        }
        for status in msg.status:
            for item in status.values:
                if item.key in allowed:
                    try:
                        self.put(item.key, float(item.value))
                    except ValueError:
                        continue

    def put(self, k, v):
        self.store.put(k, v, time.monotonic())

    def rate(self, name, header):
        self.store.event(name, stamp_seconds(header), time.monotonic())
        self.put(
            name + "_lag_s",
            self.get_clock().now().nanoseconds * 1e-9 - stamp_seconds(header),
        )

    def point(self, name, count, header):
        self.put(name, count)
        self.rate(name, header)

    def cloud(self, name, m):
        self.point(name, m.width * m.height, m.header)

    def odom(self, name, m):
        self.rate(name, m.header)
        p, v = m.pose.pose.position, m.twist.twist.linear
        for axis in "xyz":
            self.put(name + "_" + axis + "_m", getattr(p, axis))
        self.put(name + "_speed_mps", math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
        q = m.pose.pose.orientation
        self.put(
            name + "_yaw_deg",
            math.degrees(
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ),
        )

    def covariance(self, m):
        for i, axis in enumerate(("x", "y", "z", "roll", "pitch", "yaw")):
            var = m.pose.covariance[i * 7]
            if var >= 0:
                self.put("vio_sigma_" + axis, math.sqrt(var))

    def drift(self, m):
        for axis in "xyz":
            self.put("drift_" + axis + "_m", getattr(m.vector, axis))

    def detections(self, m):
        self.put("gate_detections", len(m.detections))
        self.put("gate_valid_quads", sum(d.has_corners for d in m.detections))

    def pnp(self, m):
        self.rate("pnp", m.header)
        for key, value in [
            ("pnp_quality", m.quality),
            ("pnp_reproj_px", m.reproj_px),
            ("pnp_distance_m", m.distance_m),
            ("pnp_gate_id", m.gate_id),
        ]:
            self.put(key, value)

    def publish(self):
        timing = self.tail.read()
        if timing:
            for k, v in timing.items():
                self.put(k, v)
        data = self.store.snapshot(time.monotonic())
        capacity = self.get_parameter("slam_capacity").value
        data["slam_capacity"] = float(capacity)
        if capacity > 0 and "slam_features" in data:
            data["slam_utilization_pct"] = 100 * data["slam_features"] / capacity
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "openvins"
        status.hardware_id = "adr_vio"
        status.level = (
            DiagnosticStatus.OK
            if data.get("raw_vio_age_s", float("inf")) < 2
            else DiagnosticStatus.STALE
        )
        status.message = "Public observables; unavailable/stale fields omitted; KLT requires tracker-metrics patch"
        status.values = [KeyValue(key=k, value=str(v)) for k, v in sorted(data.items())]
        msg.status = [status]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VioMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
