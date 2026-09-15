"""실기체용 mocap → PX4 외부 위치 브릿지.

입력(둘 중 하나, 파라미터 `source`)
  - 'poses' : motion_capture_tracking 의 /poses (NamedPoseArray) 에서 rigid_body_name 항목
  - 'tf'    : TF mocap_frame → rigid_body_name
출력
  /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry, NED/FRD, 속도 NaN)

EKF2 가 이를 쓰려면 airframe 4031 파라미터(EKF2_EV_CTRL=15, EKF2_HGT_REF=3, EKF2_GPS_CTRL=0)가 필요하다.
sim 에서는 model.sdf 의 OdometryPublisher 가 같은 역할을 하므로 이 노드를 띄우지 않는다(중복 주입 금지).

mocap 좌표계는 ENU(z-up) 이고 기체 정지 시 body x 가 mocap x 와 같은 방향(yaw=0)이어야
PX4 heading 과 일치한다 — 실기체 세팅 시 rigid body 정의 방향을 확인할 것.
"""
import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from adr_control import frames as F
from adr_control.offboard_base import PX4_PUB_QOS, px4_topic

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class MocapBridge(Node):
    def __init__(self):
        super().__init__('mocap_bridge')
        self.declare_parameter('source', 'poses')
        self.declare_parameter('poses_topic', '/poses')
        self.declare_parameter('rigid_body_name', 'adr_racer')
        self.declare_parameter('mocap_frame', 'mocap')
        self.declare_parameter('tf_rate', 100.0)
        self.declare_parameter('fmu_ns', '')
        self.declare_parameter('position_variance', 0.0004)      # 0.02 m ²
        self.declare_parameter('orientation_variance', 0.0004)   # 0.02 rad ²

        ns = self.get_parameter('fmu_ns').value
        self.name = self.get_parameter('rigid_body_name').value
        self.mocap_frame = self.get_parameter('mocap_frame').value
        self.pvar = float(self.get_parameter('position_variance').value)
        self.ovar = float(self.get_parameter('orientation_variance').value)
        self.pub = self.create_publisher(VehicleOdometry,
                                         px4_topic(f'{ns}/fmu/in/vehicle_visual_odometry', VehicleOdometry),
                                         PX4_PUB_QOS)
        self.n_sent = 0

        if self.get_parameter('source').value == 'tf':
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_timer(1.0 / float(self.get_parameter('tf_rate').value), self._on_tf_timer)
        else:
            from motion_capture_tracking_interfaces.msg import NamedPoseArray
            self.create_subscription(NamedPoseArray, self.get_parameter('poses_topic').value,
                                     self._on_poses, SENSOR_QOS)
        self.create_timer(5.0, lambda: self.get_logger().info(f'sent {self.n_sent} odometry msgs'))

    def _publish(self, p_enu, q_enu_flu_wxyz):
        m = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        m.timestamp = now_us
        m.timestamp_sample = now_us
        m.pose_frame = VehicleOdometry.POSE_FRAME_NED
        m.position = [float(x) for x in F.enu_to_ned(p_enu)]
        m.q = [float(x) for x in F.quat_enu_flu_to_ned_frd(q_enu_flu_wxyz)]
        m.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        m.velocity = [float('nan')] * 3
        m.angular_velocity = [float('nan')] * 3
        m.position_variance = [self.pvar] * 3
        m.orientation_variance = [self.ovar] * 3
        m.velocity_variance = [float('nan')] * 3
        m.quality = 100
        self.pub.publish(m)
        self.n_sent += 1

    def _on_poses(self, msg):
        for np_ in msg.poses:
            if np_.name == self.name:
                o = np_.pose.orientation
                self._publish([np_.pose.position.x, np_.pose.position.y, np_.pose.position.z],
                              [o.w, o.x, o.y, o.z])
                return

    def _on_tf_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(self.mocap_frame, self.name, rclpy.time.Time())
        except Exception:
            return
        tr, o = t.transform.translation, t.transform.rotation
        self._publish([tr.x, tr.y, tr.z], [o.w, o.x, o.y, o.z])


def main(args=None):
    rclpy.init(args=args)
    node = MocapBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
