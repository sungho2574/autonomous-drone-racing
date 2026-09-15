"""실기체(mocap) 골격: mocap → PX4 브릿지 + rviz 시각화. step1 노드는 step1.launch.py 를 use_sim_time:=false 로.

선행
  - motion_capture_tracking (Qualisys 등) 이 /poses 를 발행 중
  - 기체 PX4 는 airframe 4031 과 동일한 EKF2 파라미터(EKF2_EV_CTRL=15, EKF2_HGT_REF=3, EKF2_GPS_CTRL=0)
  - MicroXRCEAgent 가 기체(시리얼/UDP)에 연결됨
인자: rigid_body:=adr_racer  source:=poses|tf  rviz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('adr_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('rigid_body', default_value='adr_racer'),
        DeclareLaunchArgument('source', default_value='poses'),
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(package='adr_bringup', executable='mocap_bridge', name='mocap_bridge',
             parameters=[{'rigid_body_name': LaunchConfiguration('rigid_body'),
                          'source': LaunchConfiguration('source')}], output='screen'),
        Node(package='adr_bringup', executable='gate_markers', name='gate_markers', output='screen'),
        Node(package='adr_bringup', executable='px4_odom_to_tf', name='px4_odom_to_tf', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(bringup_share, 'config', 'adr.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
