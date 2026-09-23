"""OpenVINS VIO 를 sim 에 붙여 돌린다 (위치 추정 성능 확인용, 제어에는 안 쓴다).

    ros2 launch adr_sim sim.launch.py            # T1: gz + PX4 + 브릿지 + rviz
    ros2 launch adr_vio vio.launch.py            # T2: IMU 브릿지 + OpenVINS + 정렬/궤적
    ros2 launch adr_bringup step1.launch.py      # T3: 미션 (이륙 → 코스 비행)

보는 법
  rviz    : /adr/vio/path (VIO 궤적) 과 /adr/flown_path (실제) 를 겹쳐 본다.
            vio_align 이 TF map→global 을 쏴 주므로 OpenVINS 가 직접 내는
            /ov_msckf/points_slam(특징점 구름) 도 같은 화면에 뜬다.
  특징점  : ros2 run rqt_image_view rqt_image_view /adr/vio/debug_image
            (OpenVINS 가 구독자 수를 세서 **볼 때만** 그린다 — 안 보면 비용 0)
  숫자    : ros2 topic echo /adr/vio/error   또는 vio_align 이 5 초마다 찍는 rmse/drift 로그

인자
  imu_bridge : gz IMU → /adr/imu 브릿지 실행 (sim 기본 true. 실기체는 FC/IMU 드라이버가 낸다)
  world      : gz 월드 이름 (IMU gz 토픽 경로에 들어간다). sim.launch.py 와 같아야 한다
  config     : estimator_config.yaml 경로 (기본 adr_vio/config)
  align_mode : yaw | se3 | none  (vio_align 정렬 방식)
  verbosity  : OpenVINS 로그 레벨 (ALL/DEBUG/INFO/WARNING/ERROR/SILENT)
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VIO_SHARE = get_package_share_directory('adr_vio')
MODEL_NAME = 'adr_racer'
IMU_SENSOR_PATH = 'link/base_link/sensor/imu_sensor/imu'   # 모델 SDF 의 센서 이름 (PX4 규약)


def _nodes(context, *args, **kwargs):
    """world 를 실제 문자열로 풀어 IMU 브릿지를 만들고, ov_msckf 가 빌드돼 있는지 확인한다."""
    # launch 인자는 여기서 실제 파이썬 타입으로 풀어서 넘긴다. launch_ros 가 yaml 로 자동
    # 추론해 주긴 하지만("true"→bool), C++ 노드는 타입이 어긋나면 그 자리에서 죽으므로 명시한다.
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg('world')
    use_sim_time = arg('use_sim_time').lower() in ('true', '1')
    sim_time = {'use_sim_time': use_sim_time}
    imu_gz = f'/world/{world}/model/{MODEL_NAME}/{IMU_SENSOR_PATH}'
    nodes = [
        # gz IMU(250 Hz) → /adr/imu. PX4 내부 gz_bridge 와는 별개로 ROS 쪽에도 IMU 가 필요하다.
        # 모델 SDF 의 imu_sensor 에는 <topic> 을 주면 안 된다(PX4 가 이 긴 기본 이름을 구독한다).
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='imu_bridge',
             arguments=[f'{imu_gz}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
             remappings=[(imu_gz, '/adr/imu')],
             parameters=[sim_time], condition=IfCondition(LaunchConfiguration('imu_bridge')),
             output='screen'),
    ]
    try:
        get_package_share_directory('ov_msckf')
    except PackageNotFoundError:
        raise RuntimeError(
            'ov_msckf(OpenVINS) 패키지를 찾을 수 없다.\n'
            '  1) adr_ws/src/adr_vio/scripts/setup_openvins.sh   (adr_ws/src/open_vins 로 sparse clone)\n'
            '  2) sudo apt install libeigen3-dev libboost-all-dev libopencv-dev libceres-dev\n'
            '  3) colcon build --symlink-install --packages-select ov_core ov_init ov_msckf adr_vio\n'
            '  4) source install/setup.bash')
    config = LaunchConfiguration('config').perform(context)
    if not os.path.exists(config):
        raise RuntimeError(f'estimator_config.yaml 없음: {config}')
    nodes += [
        LogInfo(msg=f'[vio] IMU {imu_gz} → /adr/imu,  OpenVINS config={config}'),
        Node(package='ov_msckf', executable='run_subscribe_msckf', name='ov_msckf',
             namespace='ov_msckf', output='screen',
             parameters=[{
                 'config_path': config,
                 'verbosity': arg('verbosity'),
                 'use_stereo': False,
                 'max_cameras': 1,
                 'save_total_state': False,
                 **sim_time,
             }],
             # 특징점 디버그 영상. OpenVINS 는 이 퍼블리셔의 구독자가 0 이면 아예 그리지 않는다
             # (ROS2Visualizer::publish_images 의 getNumSubscribers()==0 → return).
             # 원래 이름(/ov_msckf/trackhist)으로 보고 싶으면 이 remap 을 지우면 된다.
             remappings=[('trackhist', '/adr/vio/debug_image')],
             on_exit=[LogInfo(msg='[vio] ov_msckf 가 종료됐다. 위 로그의 첫 에러를 볼 것 '
                                  '(설정 파일 경로/형식, 파라미터 타입, 토픽 이름 순으로 자주 걸린다)')]),
        Node(package='adr_vio', executable='vio_align', name='vio_align', output='screen',
             parameters=[{'align_mode': arg('align_mode'), **sim_time}]),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='adr_cross'),
        DeclareLaunchArgument('imu_bridge', default_value='true'),
        DeclareLaunchArgument('verbosity', default_value='INFO'),
        DeclareLaunchArgument('align_mode', default_value='yaw'),
        DeclareLaunchArgument('config', default_value=os.path.join(VIO_SHARE, 'config', 'estimator_config.yaml')),
        OpaqueFunction(function=_nodes),
    ])
