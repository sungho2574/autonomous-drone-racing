"""step1 placeholder: 이미지를 구독만 하고 빈 GateArray 를 발행해 토픽/메시지 계약을 고정한다.

step3 에서 Gatenet(코너 검출) + PnP 노드로 교체. 출력 규약:
  /adr/detected_gates  adr_interfaces/GateArray  (header.frame_id = 카메라 프레임, pose = 카메라 기준 게이트 포즈)
"""
import rclpy
from adr_interfaces.msg import GateArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class GateDetectorStub(Node):
    def __init__(self):
        super().__init__('gate_detector')
        self.declare_parameter('image_topic', '/adr/camera/image_raw')
        self.pub = self.create_publisher(GateArray, '/adr/detected_gates', 10)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image, SENSOR_QOS)
        self.n = 0

    def _on_image(self, msg: Image):
        self.n += 1
        out = GateArray()
        out.header = msg.header
        self.pub.publish(out)
        if self.n % 300 == 1:
            self.get_logger().info(f'{msg.width}x{msg.height} {msg.encoding}, frames={self.n} (stub: no detection)')


def main(args=None):
    rclpy.init(args=args)
    node = GateDetectorStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
