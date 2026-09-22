"""시뮬 전체를 한 번에: gz Harmonic(adr_cross) + MicroXRCEAgent + PX4 SITL + ros_gz_bridge + rviz.

    ros2 launch adr_sim sim.launch.py [ev:=true] [gui:=false] [rviz:=false] [soft_gl:=0]

이후 별도 터미널에서 미션만:  ros2 launch adr_bringup step1.launch.py

인자
  ev        : true(기본) 면 airframe 4031 — gz 진실값을 외부 비전으로 주입(mocap 에뮬레이션).
              PX4 local 프레임 == gz 월드 == map 이라 gates.yaml 좌표를 그대로 쓴다.
              false 면 4030(GPS 시뮬): local 원점이 부팅 위치가 되므로 step1.launch.py 에 origin_mode:=start 필요,
              자기장 편각 차이로 yaw 도 수 도 어긋난다. 실기체(mocap)와 같은 경로는 ev=true.
  gui       : gz GUI (false 면 headless -s)
  rviz      : rviz2 실행
  px4_dir   : PX4-Autopilot 위치 (기본 $PX4_DIR 또는 ~/PX4-Autopilot). make px4_sitl 이 끝나 있어야 함
  agent     : DDS 에이전트 실행 파일 (소스 빌드: MicroXRCEAgent, snap: micro-xrce-dds-agent)
  soft_gl   : 1 이면 llvmpipe 소프트웨어 렌더링 (GPU 없는 VM). 기본 $ADR_SOFT_GL 또는 1
  world     : 월드 이름 (assets/worlds/<world>.sdf, <world name=...> 과 동일해야 함)

PX4 기동 방식 (ARMS 의 px4_sitl.launch.py 와 같은 패턴, 셸 스크립트 없음)
  - assets/px4/airframes/* 를 매번 $PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes/ 에 복사
    → PX4 소스(ROMFS) 수정·재빌드 불필요. make px4_sitl 이 etc 를 다시 만들어도 다음 실행에 다시 들어간다.
  - 남아 있는 px4 프로세스 정리 후, gz 는 ros_gz_sim 이 띄우고 PX4 는 standalone 으로 붙는다
    (PX4_GZ_STANDALONE=1, PX4_GZ_MODEL_NAME=adr_racer). 월드 clock 토픽이 보일 때까지 기다린 뒤 실행.
  - -d 데몬 모드라 pxh 셸이 없다. 명령은 build/px4_sitl_default/bin/px4-commander, px4-param 등 클라이언트로.
  - Ctrl+C 시 px4 도 같이 정리.
"""
import os
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

SIM_SHARE = Path(get_package_share_directory('adr_sim'))
ASSETS = SIM_SHARE / 'assets'
MODEL_NAME = 'adr_racer'          # worlds/*.sdf 의 <include><name> 과 동일
AIRFRAME = {'false': 4030, 'true': 4031}


def _kill_stale_px4():
    subprocess.run(['pkill', '-9', '-f', 'px4_sitl_default/bin/px4'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _px4(context, *args, **kwargs):
    px4_dir = Path(os.path.expanduser(LaunchConfiguration('px4_dir').perform(context)))
    world = LaunchConfiguration('world').perform(context)
    ev = LaunchConfiguration('ev').perform(context).lower()
    autostart = AIRFRAME.get(ev, 4030)

    build = px4_dir / 'build' / 'px4_sitl_default'
    px4_bin = build / 'bin' / 'px4'
    if not px4_bin.exists():
        raise RuntimeError(f'PX4 SITL 바이너리 없음: {px4_bin}\n'
                           f'  경로가 다르면 px4_dir:=<경로> (또는 PX4_DIR env), 빌드 안 했으면 cd {px4_dir} && make px4_sitl')

    # airframe 주입 (ROMFS 등록/재빌드 대신 빌드 산출물에 직접 복사)
    af_dst = build / 'etc' / 'init.d-posix' / 'airframes'
    af_dst.mkdir(parents=True, exist_ok=True)
    airframes = sorted((ASSETS / 'px4' / 'airframes').iterdir())
    for f in airframes:
        shutil.copy2(f, af_dst / f.name)

    _kill_stale_px4()

    env = dict(os.environ)
    env.update({
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_MODEL_NAME': MODEL_NAME,
        'PX4_GZ_WORLD': world,
        'PX4_SYS_AUTOSTART': str(autostart),
        'PX4_SIM_MODEL': f'gz_{MODEL_NAME}',
        'PX4_UXRCE_DDS_PORT': os.environ.get('PX4_UXRCE_DDS_PORT', '8888'),
    })
    # gz 월드가 뜰 때까지 기다린 뒤 PX4 실행 (gz 와 동시에 시작되므로)
    wait_then_run = (
        f"for i in $(seq 1 60); do gz topic -l 2>/dev/null | grep -q '^/world/{world}/clock$' && break; "
        f"[ $i = 1 ] && echo '[px4] waiting for gz world {world} ...'; sleep 1; "
        f"[ $i = 60 ] && {{ echo '[px4] gz world {world} 를 60 s 안에 찾지 못함'; exit 1; }}; done; "
        f"exec ./bin/px4 -d ./etc -s etc/init.d-posix/rcS"
    )
    return [
        LogInfo(msg=f'[px4] {px4_bin}  autostart={autostart}  model={MODEL_NAME}  world={world}  '
                    f'airframes→{af_dst}: {" ".join(f.name for f in airframes)}'),
        ExecuteProcess(cmd=['bash', '-c', wait_then_run], cwd=str(build), env=env,
                       name='px4', output='screen'),
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('adr_bringup')
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    agent = LaunchConfiguration('agent')
    soft_gl = LaunchConfiguration('soft_gl')
    world_sdf = [str(ASSETS / 'worlds') + '/', world, '.sdf']
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='adr_cross'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ev', default_value='true'),
        DeclareLaunchArgument('px4_dir', default_value=EnvironmentVariable(
            'PX4_DIR', default_value=os.path.expanduser('~/PX4-Autopilot'))),
        DeclareLaunchArgument('agent', default_value='MicroXRCEAgent'),
        DeclareLaunchArgument('soft_gl', default_value=EnvironmentVariable('ADR_SOFT_GL', default_value='1')),

        # ---- 환경: 모델 검색 경로, 소프트웨어 GL ----
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(ASSETS / 'models')),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                               condition=IfCondition(PythonExpression(["'", soft_gl, "' == '1'"]))),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe',
                               condition=IfCondition(PythonExpression(["'", soft_gl, "' == '1'"]))),

        # ---- gz 서버(+GUI). -r 즉시 시작, -s 서버만 ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r ', *world_sdf], 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(gui)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -s ', *world_sdf], 'on_exit_shutdown': 'true'}.items(),
            condition=UnlessCondition(gui)),

        # ---- uXRCE-DDS 에이전트 ----
        ExecuteProcess(cmd=[agent, 'udp4', '-p', '8888'], name='xrce_agent', output='screen'),

        # ---- PX4 SITL ----
        OpaqueFunction(function=_px4),
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: _kill_stale_px4())),

        # ---- gz ↔ ROS 브릿지 (clock, 카메라, 진실값) ----
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': str(SIM_SHARE / 'config' / 'gz_bridge.yaml'), **sim_time}],
             output='screen'),

        # ---- 시각화 (adr_bringup 공통 노드) ----
        Node(package='adr_bringup', executable='gate_markers', name='gate_markers',
             parameters=[sim_time], output='screen'),
        Node(package='adr_bringup', executable='px4_odom_to_tf', name='px4_odom_to_tf',
             parameters=[sim_time], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(bringup_share, 'config', 'adr.rviz')],
             parameters=[sim_time], condition=IfCondition(rviz), output='screen'),
    ])
