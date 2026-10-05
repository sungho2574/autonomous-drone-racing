"""맵 yaml(config/maps/*.yaml) → min-snap 궤적 → PolynomialTrajectory / nav_msgs/Path 발행.

한 번 계산해 transient_local 로 latch 하므로 컨트롤러가 나중에 떠도 받는다.
"""
import os
from dataclasses import fields
import time

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
from adr_planning.perception_heading import HeadingOptions, perception_aware_heading

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
        self.declare_parameter('heading_mode', 'perception_aware')
        defaults = HeadingOptions()
        for field in fields(defaults):
            value = getattr(defaults, field.name)
            self.declare_parameter('heading_' + field.name, list(value) if isinstance(value, tuple) else value)

        # 빈 값이면 기본 맵. launch 는 map:= 에서 이 경로를 만들어 넘긴다.
        gates_file = self.get_parameter('gates_file').value or os.path.join(
            get_package_share_directory('adr_bringup'), 'config', 'maps', 'cross.yaml')
        frame_id = self.get_parameter('frame_id').value

        course = load_course(gates_file)
        laps = int(self.get_parameter('laps').value)
        wp, yaw = course_waypoints(
            course,
            laps=laps,
            approach_dist=float(self.get_parameter('approach_dist').value),
            end_at_start=bool(self.get_parameter('end_at_start').value))
        # 계획 전후를 반드시 찍는다. 예전엔 여기서 조용히 몇 분을 태우는 바람에
        # (dense SVD, big_track 10바퀴 = 361 세그먼트 → 약 4분) 노드 등록조차 못 했고,
        # 경로도 비행도 없으니 '고장난 것'처럼 보였다. 지금은 희소 KKT 라 1초 안쪽이지만
        # 맵·laps 를 키우면 다시 길어질 수 있으므로 진행 상황은 계속 보여 준다.
        self.get_logger().info(f'{gates_file}: 게이트 {len(course.gates)}개 × {laps}바퀴 → '
                               f'웨이포인트 {len(wp)}개, 세그먼트 {len(wp) - 1}개. min-snap 계획 시작...')
        t_plan = time.time()
        traj = plan(wp, yaw,
                    v_avg=float(self.get_parameter('v_avg').value),
                    v_max=float(self.get_parameter('v_max').value),
                    a_max=float(self.get_parameter('a_max').value),
                    t_min=float(self.get_parameter('t_min').value))
        heading_mode = self.get_parameter('heading_mode').value
        if heading_mode == 'perception_aware':
            options = HeadingOptions(**{field.name: self.get_parameter('heading_' + field.name).value
                                        for field in fields(HeadingOptions)})
            traj, report = perception_aware_heading(
                traj, course, laps, float(self.get_parameter('approach_dist').value),
                bool(self.get_parameter('end_at_start').value), options)
            self.get_logger().info(
                f"perception heading: full-gate visibility {report['baseline']['visible_fraction']:.1%}"
                f" → {report['planned']['visible_fraction']:.1%}, "
                f"baseline at same duration {report['baseline_matched_time']['visible_fraction']:.1%}, "
                f"centered {report['planned']['centered_fraction']:.1%}, "
                f"time ×{report['time_stretch']:.2f}, "
                f"yaw rate {report['max_rate_deg']:.1f} deg/s, "
                f"yaw acceleration {report['max_accel_deg']:.1f} deg/s² (predicted, not flown)")
            if report['planned']['visible_fraction'] < 1.0:
                self.get_logger().warn('시야 밖 구간이 남아 있습니다. yaw만으로 수직 FOV·통과 직전 잘림은 해결할 수 없습니다.')
        elif heading_mode != 'gate_normal':
            raise ValueError('heading_mode must be perception_aware or gate_normal')
        dt_plan = time.time() - t_plan
        v_pk, a_pk = traj.peak()
        self.get_logger().info(
            f'계획 완료 ({dt_plan:.2f}s): {len(traj.segments)} segments, '
            f'T={traj.duration:.1f}s, v_peak={v_pk:.2f} m/s, a_peak={a_pk:.2f} m/s^2')
        if dt_plan > 3.0:
            self.get_logger().warn(
                f'계획 및 시야 평가에 {dt_plan:.1f}초 걸렸다. laps 또는 heading_sample_dt로 계산량을 조절할 수 있다.')

        self.traj_pub = self.create_publisher(PolynomialTrajectory, '/adr/trajectory', LATCHED)
        self.path_pub = self.create_publisher(Path, '/adr/planned_path', LATCHED)
        self._traj, self._frame_id = traj, frame_id
        # 여기서 바로 발행하면 안 된다 — use_sim_time 일 때 /clock 은 아직 한 번도 안 들어왔고,
        # get_clock().now() 가 0 을 준다. stamp=0 인 Path 는 rviz 가 TF 버퍼(기본 10 s)에서
        # 조회에 실패해 **통째로 버린다** → 경로가 안 그려진다.
        # gz 기동이 오래 걸리는 큰 맵일수록 sim 시각이 앞서 있어 확실히 재현된다
        # (작은 맵은 sim 시각이 0 근처라 우연히 통과해서 '맵마다 된다/안 된다'로 보였다).
        # 시계가 살아난 뒤 한 번만 발행하고 타이머를 끈다.
        self._sent_traj = False
        self._path_pts = None
        self._pub_timer = self.create_timer(1.0, self._tick)

    def _tick(self):
        """궤적은 한 번만, 경로(rviz)는 1 Hz 로 계속 보낸다.

        latched 라 원칙적으로 한 번이면 되지만, 그러면 타이밍에 휘둘린다:
        stamp 가 sim 시각보다 한참 뒤처지면 rviz 가 TF 조회에 실패해 버리고,
        구독 시점이 어긋나도 복구할 길이 없다. 1 Hz 재발행이면 언제 붙어도 1 초 안에 최신 stamp 가 간다.
        궤적은 재발행하지 않는다 — 컨트롤러가 '비행 중 궤적 갱신은 무시함' 을 찍게 되므로.
        """
        if self.get_parameter('use_sim_time').value and self.get_clock().now().nanoseconds == 0:
            return                                   # /clock 대기
        stamp = self.get_clock().now().to_msg()
        if not self._sent_traj:
            self.traj_pub.publish(trajectory_to_msg(self._traj, self._frame_id, stamp))
            self._path_pts = [(row[1:4], row[10])
                              for row in self._traj.sample_all(float(self.get_parameter('path_dt').value))]
            self._sent_traj = True
            self.get_logger().info(f'궤적 발행 (stamp={stamp.sec}.{stamp.nanosec // 1000000:03d}), '
                                   f'경로 {len(self._path_pts)} points 를 1 Hz 로 재발행')

        path = Path()
        path.header.frame_id = self._frame_id
        path.header.stamp = stamp
        for xyz, yaw in self._path_pts:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, xyz)
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = yaw_to_quat(yaw)
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
