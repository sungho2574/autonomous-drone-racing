"""gates.yaml → min-snap 궤적 → PolynomialTrajectory / nav_msgs/Path 발행.

한 번 계산해 transient_local 로 latch 하므로 컨트롤러가 나중에 떠도 받는다.
"""
import os

import numpy as np
import rclpy
from adr_msgs.msg import PolynomialSegment, PolynomialTrajectory
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from adr_planning.course import course_waypoints, load_course
from adr_planning.min_snap import Trajectory, plan

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def trajectory_to_msg(traj: Trajectory, frame_id: str, stamp) -> PolynomialTrajectory:
    msg = PolynomialTrajectory()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    for s in traj.to_plain():
        seg = PolynomialSegment()
        seg.duration = s['duration']
        seg.cx, seg.cy, seg.cz, seg.cyaw = s['cx'], s['cy'], s['cz'], s['cyaw']
        msg.segments.append(seg)
    return msg


def trajectory_from_msg(msg: PolynomialTrajectory) -> Trajectory:
    return Trajectory.from_plain([
        {'duration': s.duration, 'cx': list(s.cx), 'cy': list(s.cy),
         'cz': list(s.cz), 'cyaw': list(s.cyaw)} for s in msg.segments])


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2)))


class GatePlanner(Node):
    def __init__(self):
        super().__init__('gate_planner')
        self.declare_parameter('gates_file', '')
        self.declare_parameter('laps', 1)
        self.declare_parameter('approach_dist', 0.8)
        self.declare_parameter('v_avg', 3.0)
        self.declare_parameter('v_max', 6.0)
        self.declare_parameter('a_max', 8.0)
        self.declare_parameter('t_min', 0.3)
        self.declare_parameter('end_at_start', True)
        self.declare_parameter('path_dt', 0.05)
        self.declare_parameter('frame_id', 'map')

        gates_file = self.get_parameter('gates_file').value or os.path.join(
            get_package_share_directory('adr_bringup'), 'config', 'gates.yaml')
        frame_id = self.get_parameter('frame_id').value

        course = load_course(gates_file)
        wp, yaw = course_waypoints(
            course,
            laps=int(self.get_parameter('laps').value),
            approach_dist=float(self.get_parameter('approach_dist').value),
            end_at_start=bool(self.get_parameter('end_at_start').value))
        traj = plan(wp, yaw,
                    v_avg=float(self.get_parameter('v_avg').value),
                    v_max=float(self.get_parameter('v_max').value),
                    a_max=float(self.get_parameter('a_max').value),
                    t_min=float(self.get_parameter('t_min').value))
        v_pk, a_pk = traj.peak()
        self.get_logger().info(
            f'{gates_file}: {len(wp)} waypoints, {len(traj.segments)} segments, '
            f'T={traj.duration:.1f}s, v_peak={v_pk:.2f} m/s, a_peak={a_pk:.2f} m/s^2')

        stamp = self.get_clock().now().to_msg()
        self.traj_pub = self.create_publisher(PolynomialTrajectory, '/adr/trajectory', LATCHED)
        self.path_pub = self.create_publisher(Path, '/adr/planned_path', LATCHED)
        self.traj_pub.publish(trajectory_to_msg(traj, frame_id, stamp))

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp
        for row in traj.sample_all(float(self.get_parameter('path_dt').value)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, row[1:4])
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = yaw_to_quat(row[10])
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = GatePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
