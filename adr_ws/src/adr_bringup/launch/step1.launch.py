"""step1 파이프라인: gate_planner(min-snap) + px4_position_controller (+ perception stub).

선행: sim.launch.py (또는 real.launch.py) + PX4 + MicroXRCEAgent 가 떠 있어야 한다.
인자: laps:=2  time_scale:=1.0  land_after:=true  perception:=false  use_sim_time:=true
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
    control_share = get_package_share_directory('adr_control')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('laps', default_value='2'),
        DeclareLaunchArgument('time_scale', default_value='1.0'),
        DeclareLaunchArgument('land_after', default_value='true'),
        DeclareLaunchArgument('perception', default_value='false'),

        Node(package='adr_planning', executable='gate_planner', name='gate_planner',
             parameters=[os.path.join(bringup_share, 'config', 'planner.yaml'),
                         {'laps': LaunchConfiguration('laps'), **sim_time}],
             output='screen'),
        Node(package='adr_control', executable='px4_position_controller', name='px4_position_controller',
             parameters=[os.path.join(control_share, 'config', 'controller.yaml'),
                         {'time_scale': LaunchConfiguration('time_scale'),
                          'land_after': LaunchConfiguration('land_after'), **sim_time}],
             output='screen'),
        Node(package='adr_perception', executable='gate_detector_stub', name='gate_detector',
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('perception')),
             output='screen'),
    ])
