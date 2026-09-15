"""gates.yaml 의 게이트를 rviz MarkerArray 로 그리고, PX4 odometry 로 실제 비행 궤적(Path)을 누적한다.

발행
  /adr/gate_markers   visualization_msgs/MarkerArray (latched)  게이트 프레임(주황) + 법선 화살표 + id
  /adr/gates          adr_interfaces/GateArray (latched)         맵 기반 게이트 목록 (step3 인식 결과와 같은 형식)
  /adr/flown_path     nav_msgs/Path                              TF map→base_link 를 주기적으로 샘플링
"""
import os
from math import cos, sin

import rclpy
from adr_interfaces.msg import Gate as GateMsg
from adr_interfaces.msg import GateArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from adr_planning.course import load_course

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
ORANGE = (1.0, 0.45, 0.0, 1.0)


def _quat_z(yaw):
    return 0.0, 0.0, sin(yaw / 2), cos(yaw / 2)


class GateMarkers(Node):
    def __init__(self):
        super().__init__('gate_markers')
        self.declare_parameter('gates_file', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('path_rate', 10.0)
        self.declare_parameter('path_max_points', 6000)

        gates_file = self.get_parameter('gates_file').value or os.path.join(
            get_package_share_directory('adr_bringup'), 'config', 'gates.yaml')
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.course = load_course(gates_file)

        self.marker_pub = self.create_publisher(MarkerArray, '/adr/gate_markers', LATCHED)
        self.gates_pub = self.create_publisher(GateArray, '/adr/gates', LATCHED)
        self.path_pub = self.create_publisher(Path, '/adr/flown_path', 10)
        self.marker_pub.publish(self._markers())
        self.gates_pub.publish(self._gate_array())

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = Path()
        self.path.header.frame_id = self.frame_id
        self.max_points = int(self.get_parameter('path_max_points').value)
        self.create_timer(1.0 / float(self.get_parameter('path_rate').value), self._on_path_timer)

    def _gate_array(self) -> GateArray:
        msg = GateArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for g in self.course.gates:
            m = GateMsg()
            m.id = g.id
            m.pose.position.x, m.pose.position.y, m.pose.position.z = g.x, g.y, g.z
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = _quat_z(g.yaw)
            m.inner_size, m.outer_size = self.course.inner_size, self.course.outer_size
            msg.gates.append(m)
        return msg

    def _markers(self) -> MarkerArray:
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        half_in = self.course.inner_size / 2
        half_out = self.course.outer_size / 2
        w = half_out - half_in
        for g in self.course.gates:
            base = Marker()
            base.header.frame_id = self.frame_id
            base.header.stamp = stamp
            base.ns = 'gate'
            base.pose.position.x, base.pose.position.y, base.pose.position.z = g.x, g.y, g.z
            (base.pose.orientation.x, base.pose.orientation.y,
             base.pose.orientation.z, base.pose.orientation.w) = _quat_z(g.yaw)
            base.color.r, base.color.g, base.color.b, base.color.a = ORANGE

            # 프레임: 게이트 로컬 좌표(x=법선, y=가로, z=세로)에서 4개 CUBE_LIST
            frame = Marker()
            frame.header, frame.ns, frame.pose = base.header, 'gate_frame', base.pose
            frame.id = g.id
            frame.type = Marker.LINE_LIST        # 폭 w 의 선 4개 = 프레임
            frame.action = Marker.ADD
            frame.color = base.color
            frame.scale.x = w
            c = half_in + w / 2
            for (y0, z0, y1, z1) in ((c, -half_out, c, half_out), (-c, -half_out, -c, half_out),
                                     (-half_in, c, half_in, c), (-half_in, -c, half_in, -c)):
                frame.points.append(Point(x=0.0, y=y0, z=z0))
                frame.points.append(Point(x=0.0, y=y1, z=z1))
            arr.markers.append(frame)

            arrow = Marker()
            arrow.header, arrow.ns, arrow.pose = base.header, 'gate_normal', base.pose
            arrow.id = g.id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 1.0, 0.08, 0.08
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.8, 0.1, 0.9
            arr.markers.append(arrow)

            text = Marker()
            text.header, text.ns = base.header, 'gate_id'
            text.id = g.id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = g.x, g.y
            text.pose.position.z = g.z + half_out + 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.4
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = f'G{g.id}'
            arr.markers.append(text)

        start = Marker()
        start.header.frame_id = self.frame_id
        start.header.stamp = stamp
        start.ns, start.id = 'start', 0
        start.type = Marker.CYLINDER
        start.action = Marker.ADD
        start.pose.position.x, start.pose.position.y = float(self.course.start[0]), float(self.course.start[1])
        start.pose.position.z = 0.01
        start.pose.orientation.w = 1.0
        start.scale.x = start.scale.y = 0.5
        start.scale.z = 0.02
        start.color.r, start.color.g, start.color.b, start.color.a = 0.2, 0.4, 1.0, 0.8
        arr.markers.append(start)
        return arr

    def _on_path_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.0))
        except Exception:
            return
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = tf.header.stamp
        ps.pose.position.x = tf.transform.translation.x
        ps.pose.position.y = tf.transform.translation.y
        ps.pose.position.z = tf.transform.translation.z
        ps.pose.orientation = tf.transform.rotation
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_points:
            del self.path.poses[:len(self.path.poses) - self.max_points]
        self.path.header.stamp = tf.header.stamp
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = GateMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
