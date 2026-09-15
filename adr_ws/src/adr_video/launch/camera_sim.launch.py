"""sim 카메라만 단독으로 브릿지할 때 (sim.launch.py 는 이미 gz_bridge.yaml 로 같은 일을 한다)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
             arguments=['/adr_racer/camera@sensor_msgs/msg/Image[gz.msgs.Image',
                        '/adr_racer/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
             remappings=[('/adr_racer/camera', '/adr/camera/image_raw'),
                         ('/adr_racer/camera_info', '/adr/camera/camera_info')],
             output='screen'),
    ])
