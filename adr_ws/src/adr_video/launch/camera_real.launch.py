"""실기체 USB 카메라 예시: v4l2_camera → camera_relay → /adr/camera/image_raw"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        Node(package='v4l2_camera', executable='v4l2_camera_node', name='v4l2_camera',
             parameters=[{'video_device': LaunchConfiguration('video_device'),
                          'image_size': [640, 480]}], output='screen'),
        Node(package='adr_video', executable='camera_relay', name='camera_relay',
             parameters=[{'source_topic': '/image_raw', 'source_info_topic': '/camera_info'}],
             output='screen'),
    ])
