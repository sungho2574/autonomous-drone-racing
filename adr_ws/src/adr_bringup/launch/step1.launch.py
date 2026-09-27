"""step1 파이프라인: gate_planner(min-snap) + px4_position_controller (+ perception, PnP, VIO).

선행: ros2 launch adr_sim sim.launch.py (gz+PX4+Agent+rviz 일괄) 또는 real.launch.py
인자: map:=cross   비행할 맵 = adr_bringup/config/maps/<맵>.yaml. **sim.launch.py 의 map 과 같아야 한다**
      laps:=2  time_scale:=1.0  land_after:=true  perception:=true  pnp:=true  use_sim_time:=true
      origin_mode:=world|start  (PX4 local 원점이 map 과 다를 때 = GPS 시뮬 모드(ev:=false) 면 start)
      vio:=true|false           OpenVINS VIO + drift 보정 KF (기본 true). OpenVINS 미설치면 launch 가 멈추며
                                설치 절차를 안내한다. 끄려면 vio:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        DeclareLaunchArgument('perception', default_value='true'),
        DeclareLaunchArgument('pnp', default_value='true'),
        DeclareLaunchArgument('origin_mode', default_value='world'),
        # VIO(OpenVINS) + drift 보정 KF. 기본 켬.
        # OpenVINS 가 설치돼 있지 않으면 이 launch 전체가 **에러와 함께 멈춘다**(조용히 넘어가지 않는다).
        # 그 에러 메시지가 설치 절차를 안내하므로 그대로 두는 편이 낫다 — adr_vio/launch/vio.launch.py 참고.
        # CPU 가 부족하거나 VIO 가 필요 없으면 vio:=false.
        DeclareLaunchArgument('vio', default_value='true'),
        # 맵 = 코스 정의의 단일 진실 원천. planner/markers/pnp 로, 그리고 vio 의 gz IMU 토픽 경로(월드 이름)로 간다.
        DeclareLaunchArgument('map', default_value='cross'),

        Node(package='adr_planning', executable='gate_planner', name='gate_planner',
             parameters=[os.path.join(bringup_share, 'config', 'planner.yaml'),
                         {'laps': LaunchConfiguration('laps'),
                          'gates_file': [os.path.join(bringup_share, 'config', 'maps') + '/',
                                         LaunchConfiguration('map'), '.yaml'],
                          **sim_time}],
             output='screen'),
        Node(package='adr_control', executable='px4_position_controller', name='px4_position_controller',
             parameters=[os.path.join(control_share, 'config', 'controller.yaml'),
                         {'time_scale': LaunchConfiguration('time_scale'),
                          'land_after': LaunchConfiguration('land_after'),
                          'origin_mode': LaunchConfiguration('origin_mode'),
                          # origin_mode=start 일 때 start 좌표를 여기서 읽는다 — planner 와 같은 맵이어야 한다
                          'gates_file': [os.path.join(bringup_share, 'config', 'maps') + '/',
                                         LaunchConfiguration('map'), '.yaml'],
                          **sim_time}],
             output='screen'),
        Node(package='adr_perception', executable='gate_detector', name='gate_detector',
             parameters=[os.path.join(get_package_share_directory('adr_perception'), 'config', 'gate_detector.yaml'),
                         sim_time], condition=IfCondition(LaunchConfiguration('perception')),
             output='screen'),
        # PnP 위치 추정 — 시각화/평가 전용. 제어에는 절대 안 들어간다 (TF pnp_base_link 로만 나감)
        Node(package='adr_perception', executable='gate_pnp', name='gate_pnp',
             parameters=[os.path.join(get_package_share_directory('adr_perception'), 'config', 'gate_pnp.yaml'),
                         {'gates_file': [os.path.join(bringup_share, 'config', 'maps') + '/',
                                         LaunchConfiguration('map'), '.yaml']},
                         sim_time], condition=IfCondition(LaunchConfiguration('pnp')),
             output='screen'),
        # VIO drift 보정 (논문 §2.4 KF) — VIO + 게이트 PnP → /adr/state/corrected.
        # vio:=true 일 때만 의미가 있다(VIO 가 있어야 보정할 대상이 있음). 제어에는 미반영.
        Node(package='adr_state_estimation', executable='drift_corrector', name='drift_corrector',
             parameters=[os.path.join(get_package_share_directory('adr_state_estimation'),
                                      'config', 'drift_corrector.yaml'), sim_time],
             condition=IfCondition(LaunchConfiguration('vio')),
             output='screen'),
        # VIO (선택) — 제어에는 안 들어간다. sim 이면 gz IMU 브릿지도 같이 뜬다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('adr_vio'), 'launch', 'vio.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'world': LaunchConfiguration('map'),
                              'imu_bridge': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('vio'))),
    ])
