"""OpenVINS 추정 결과를 map 프레임에 정렬해 rviz 궤적으로 그리고, 기준 대비 오차를 잰다.

OpenVINS 는 자기 원점(`global` 프레임: 시작 자세, 중력 정렬, yaw=0)에서 추정을 낸다.
그대로는 rviz 의 map 과 연결이 없어 아무것도 안 보인다. 이 노드가 시작 시점의 기준 포즈
(TF map→base_link: sim 은 PX4 EKF, 실기체는 mocap)로 map↔global 변환을 한 번 구해서

  * TF `map → global` (static) 을 쏜다
      → OpenVINS 가 직접 내는 /ov_msckf/pathimu, points_slam(특징점) 도 rviz 에서 같이 보인다
  * /adr/vio/path   (nav_msgs/Path, map)      VIO 궤적 — rviz 에 그릴 것
  * /adr/vio/odom   (nav_msgs/Odometry, map)  + TF map→vio_base_link (추정 포즈)
  * /adr/vio/error  (std_msgs/Float32)        기준 대비 위치 오차 [m]

정렬 방식(align_mode)
  yaw  (기본) 평행이동 + yaw 만 맞춘다. VIO 의 global 은 이미 중력 정렬이라 roll/pitch 는
             원래 0 이어야 하므로, 남는 기울기 오차를 감추지 않고 그대로 드러낸다.
  se3        시작 포즈를 완전히 일치시킨다(6-DoF).
  none       변환 없음(= map 과 global 이 같다고 가정). 디버그용.

오차는 '진짜 오차'가 아니라 **기준(TF map→base_link) 대비 차이**다. sim 에서 기준은 gz 진실값이
EKF2 를 통해 들어온 값이라 사실상 진실값이고, 실기체에서는 mocap 이다.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quat_to_R(q) -> np.ndarray:
    """(w, x, y, z) → 회전행렬."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n == 0.0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R: np.ndarray) -> np.ndarray:
    """회전행렬 → (w, x, y, z). trace 기반, 수치적으로 안전한 분기."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def yaw_of(R: np.ndarray) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def Rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class VioAlign(Node):
    def __init__(self):
        super().__init__('vio_align')
        self.declare_parameter('odom_topic', '/ov_msckf/odomimu')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('ref_frame', 'base_link')      # 기준(진실값) 프레임
        self.declare_parameter('vio_global_frame', 'global')  # OpenVINS 가 쓰는 프레임 이름
        self.declare_parameter('vio_base_frame', 'vio_base_link')
        self.declare_parameter('align_mode', 'yaw')           # yaw | se3 | none
        self.declare_parameter('align_delay', 0.0)            # 이 시간[s] 동안의 VIO 포즈는 버리고 정렬
        self.declare_parameter('path_min_dist', 0.03)         # Path 에 점을 더할 최소 이동 [m]
        self.declare_parameter('path_max_points', 5000)
        self.declare_parameter('log_period', 5.0)             # 오차 요약 로그 주기 [s]

        self.map_frame = self.get_parameter('map_frame').value
        self.ref_frame = self.get_parameter('ref_frame').value
        self.global_frame = self.get_parameter('vio_global_frame').value
        self.vio_base_frame = self.get_parameter('vio_base_frame').value
        self.align_mode = self.get_parameter('align_mode').value
        if self.align_mode not in ('yaw', 'se3', 'none'):
            raise ValueError(f"align_mode 는 yaw|se3|none: {self.align_mode}")
        self.align_delay = float(self.get_parameter('align_delay').value)
        self.path_min_dist = float(self.get_parameter('path_min_dist').value)
        self.path_max_points = int(self.get_parameter('path_max_points').value)

        self.R_mg = np.eye(3)       # map ← global
        self.t_mg = np.zeros(3)
        self.aligned = (self.align_mode == 'none')
        self.t_first = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_bc = TransformBroadcaster(self)
        self.static_bc = StaticTransformBroadcaster(self)

        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.path_pub = self.create_publisher(Path, '/adr/vio/path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/adr/vio/odom', 10)
        self.err_pub = self.create_publisher(Float32, '/adr/vio/error', 10)

        # 통계 (오차 요약 로그용)
        self.n_err = 0
        self.sum_sq = 0.0
        self.max_err = 0.0
        self.last_err = float('nan')
        self.dist = 0.0             # VIO 가 이동한 누적 거리 [m] — drift 비율 계산용
        self.n_odom = 0

        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_timer(float(self.get_parameter('log_period').value), self._log)
        self.get_logger().info(
            f"vio_align: {self.get_parameter('odom_topic').value} → /adr/vio/path (align={self.align_mode}), "
            f"기준 TF {self.map_frame}→{self.ref_frame}")

    # ------------------------------------------------------------------
    def _ref_pose(self):
        """기준 포즈 (R, t) in map. 없으면 None.

        가장 최근 TF 를 쓴다(스탬프 지정 lookup 은 단일 스레드 executor 에서 블로킹 위험).
        TF 는 100 Hz, VIO 는 20~30 Hz 라 시간 어긋남은 10 ms 수준.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.ref_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        tr = tf.transform.translation
        return quat_to_R([q.w, q.x, q.y, q.z]), np.array([tr.x, tr.y, tr.z])

    def _align(self, R_gi, t_gi) -> bool:
        """map ← global 변환 계산. 기준 TF 가 아직 없으면 False."""
        ref = self._ref_pose()
        if ref is None:
            return False
        R_mb, t_mb = ref
        if self.align_mode == 'se3':
            self.R_mg = R_mb @ R_gi.T
        else:   # yaw: 평행이동 + yaw 만
            self.R_mg = Rz(yaw_of(R_mb) - yaw_of(R_gi))
        self.t_mg = t_mb - self.R_mg @ t_gi
        self.aligned = True

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.global_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, self.t_mg)
        qw, qx, qy, qz = R_to_quat(self.R_mg)
        (tf.transform.rotation.w, tf.transform.rotation.x,
         tf.transform.rotation.y, tf.transform.rotation.z) = float(qw), float(qx), float(qy), float(qz)
        self.static_bc.sendTransform(tf)
        self.get_logger().info(
            f'정렬 완료 ({self.align_mode}): map ← {self.global_frame} '
            f't={np.round(self.t_mg, 3)} yaw={math.degrees(yaw_of(self.R_mg)):.1f}°  '
            f'→ TF 발행. OpenVINS 의 /ov_msckf/pathimu·points_slam 도 rviz 에서 보인다')
        return True

    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        self.n_odom += 1
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        t_gi = np.array([p.x, p.y, p.z])
        R_gi = quat_to_R([o.w, o.x, o.y, o.z])

        if self.t_first is None:
            self.t_first = self.get_clock().now()
        if not self.aligned:
            elapsed = (self.get_clock().now() - self.t_first).nanoseconds * 1e-9
            if elapsed < self.align_delay:
                return
            if not self._align(R_gi, t_gi):
                if self.n_odom % 50 == 1:
                    self.get_logger().warn(
                        f'기준 TF {self.map_frame}→{self.ref_frame} 가 없어 정렬 대기 중 '
                        f'(sim: px4_odom_to_tf, 실기체: mocap 이 떠 있어야 한다)')
                return

        # map 기준 추정 포즈
        t_mi = self.R_mg @ t_gi + self.t_mg
        R_mi = self.R_mg @ R_gi
        qw, qx, qy, qz = R_to_quat(R_mi)
        stamp = msg.header.stamp

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.vio_base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, t_mi)
        (odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
         odom.pose.pose.orientation.y, odom.pose.pose.orientation.z) = float(qw), float(qx), float(qy), float(qz)
        odom.pose.covariance = msg.pose.covariance
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.vio_base_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, t_mi)
        (tf.transform.rotation.w, tf.transform.rotation.x,
         tf.transform.rotation.y, tf.transform.rotation.z) = float(qw), float(qx), float(qy), float(qz)
        self.tf_bc.sendTransform(tf)

        # 궤적 (일정 거리 이상 움직였을 때만 점 추가)
        if not self.path.poses or \
                np.linalg.norm(t_mi - self._last_path_point()) >= self.path_min_dist:
            if self.path.poses:
                self.dist += float(np.linalg.norm(t_mi - self._last_path_point()))
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = self.map_frame
            pose.pose = odom.pose.pose
            self.path.poses.append(pose)
            if len(self.path.poses) > self.path_max_points:
                del self.path.poses[:len(self.path.poses) - self.path_max_points]
            self.path.header.stamp = stamp
            self.path_pub.publish(self.path)

        # 기준 대비 오차
        ref = self._ref_pose()
        if ref is not None:
            err = float(np.linalg.norm(t_mi - ref[1]))
            self.err_pub.publish(Float32(data=err))
            self.last_err = err
            self.n_err += 1
            self.sum_sq += err * err
            self.max_err = max(self.max_err, err)

    def _last_path_point(self) -> np.ndarray:
        p = self.path.poses[-1].pose.position
        return np.array([p.x, p.y, p.z])

    def _log(self):
        if not self.aligned:
            if self.n_odom == 0:
                self.get_logger().info('VIO odometry 대기 중 — OpenVINS 초기화 전이면 정상 '
                                       '(정지 후 이륙하면 static init 이 걸린다)')
            return
        if self.n_err == 0:
            return
        rmse = math.sqrt(self.sum_sq / self.n_err)
        drift = 100.0 * self.last_err / self.dist if self.dist > 0.5 else float('nan')
        self.get_logger().info(
            f'VIO 오차 now={self.last_err:.3f} m  rmse={rmse:.3f} m  max={self.max_err:.3f} m  '
            f'이동거리={self.dist:.1f} m  drift={drift:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    node = VioAlign()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
