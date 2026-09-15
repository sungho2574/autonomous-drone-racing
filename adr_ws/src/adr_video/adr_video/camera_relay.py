"""카메라 입력 relay: 임의의 소스 토픽을 /adr/camera/image_raw 규격으로 맞춘다.

- sim : ros_gz_bridge 가 이미 /adr/camera/image_raw 를 내므로 relay 가 필요 없다.
- 실기체: v4l2_camera / gscam 등의 출력을 source_topic 으로 받아 frame_id·throttle 을 통일해 재발행.
  (해상도 변환은 perception 쪽 전처리에서 처리하는 것이 낫다 — 여기선 복사만)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class CameraRelay(Node):
    def __init__(self):
        super().__init__('camera_relay')
        self.declare_parameter('source_topic', '/image_raw')
        self.declare_parameter('source_info_topic', '/camera_info')
        self.declare_parameter('target_topic', '/adr/camera/image_raw')
        self.declare_parameter('target_info_topic', '/adr/camera/camera_info')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('target_fps', 30.0)   # 0 이면 throttle 없음

        self.frame_id = self.get_parameter('frame_id').value
        fps = float(self.get_parameter('target_fps').value)
        self.min_dt = 1.0 / fps if fps > 0 else 0.0
        self._last = None

        self.img_pub = self.create_publisher(Image, self.get_parameter('target_topic').value, SENSOR_QOS)
        self.info_pub = self.create_publisher(CameraInfo, self.get_parameter('target_info_topic').value, SENSOR_QOS)
        self.create_subscription(Image, self.get_parameter('source_topic').value, self._on_image, SENSOR_QOS)
        self.create_subscription(CameraInfo, self.get_parameter('source_info_topic').value, self._on_info, SENSOR_QOS)

    def _on_image(self, msg: Image):
        t = self.get_clock().now().nanoseconds * 1e-9
        if self._last is not None and t - self._last < self.min_dt:
            return
        self._last = t
        msg.header.frame_id = self.frame_id
        self.img_pub.publish(msg)

    def _on_info(self, msg: CameraInfo):
        msg.header.frame_id = self.frame_id
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
