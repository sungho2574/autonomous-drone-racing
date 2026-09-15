"""/fmu/out/vehicle_odometry (NED/FRD) → TF map→base_link (ENU/FLU) + nav_msgs/Odometry.

rviz 에서 기체 위치를 보고, gate_markers 가 실제 비행 궤적을 누적하는 데 쓴다.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from adr_control import frames as F
from adr_control.offboard_base import PX4_SUB_QOS


class PX4OdomToTF(Node):
    def __init__(self):
        super().__init__('px4_odom_to_tf')
        self.declare_parameter('fmu_ns', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        ns = self.get_parameter('fmu_ns').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child = self.get_parameter('child_frame_id').value
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/adr/odom', 10)
        self.create_subscription(VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry', self._cb, PX4_SUB_QOS)

    def _cb(self, m: VehicleOdometry):
        if m.pose_frame != VehicleOdometry.POSE_FRAME_NED or m.q[0] != m.q[0]:
            return
        p = F.ned_to_enu(m.position)
        q = F.quat_ned_frd_to_enu_flu(m.q)
        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = map(float, q)
        self.br.sendTransform(t)

        o = Odometry()
        o.header = t.header
        o.child_frame_id = self.child
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
        o.pose.pose.orientation = t.transform.rotation
        if m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED and m.velocity[0] == m.velocity[0]:
            v = F.ned_to_enu(m.velocity)
            o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, v)
        self.odom_pub.publish(o)


def main(args=None):
    rclpy.init(args=args)
    node = PX4OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
