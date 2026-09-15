"""gz Harmonic 월드(adr_cross) + ros_gz_bridge + rviz 시각화 노드.

PX4 SITL 과 MicroXRCEAgent 는 별도 터미널에서:
  T2: MicroXRCEAgent udp4 -p 8888
  T3: adr_ws/src/adr_sim/scripts/run_px4_sitl.sh [--ev]
그 다음 T4: ros2 launch adr_bringup step1.launch.py

인자: world:=adr_cross.sdf  gui:=true|false(headless)  rviz:=true|false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory('adr_sim')
    bringup_share = get_package_share_directory('adr_bringup')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='adr_cross.sdf'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 모델/월드 검색 경로 (install/setup.bash 의 env hook 과 중복돼도 무해)
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'worlds')),

        # gz 서버(+GUI). -r: 즉시 시뮬 시작, -s: 서버만(headless)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -s ', world], 'on_exit_shutdown': 'true'}.items(),
            condition=UnlessCondition(gui)),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': os.path.join(bringup_share, 'config', 'gz_bridge.yaml'), **sim_time}],
             output='screen'),

        Node(package='adr_bringup', executable='gate_markers', name='gate_markers',
             parameters=[sim_time], output='screen'),
        Node(package='adr_bringup', executable='px4_odom_to_tf', name='px4_odom_to_tf',
             parameters=[sim_time], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(bringup_share, 'config', 'adr.rviz')],
             parameters=[sim_time], condition=IfCondition(rviz), output='screen'),
    ])
