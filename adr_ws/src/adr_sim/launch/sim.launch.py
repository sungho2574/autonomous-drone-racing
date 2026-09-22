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
  - 이전 실행 잔재(gz·PX4·브릿지)를 먼저 정리한 뒤, gz 는 ros_gz_sim 이 띄우고 PX4 는 standalone 으로 붙는다
    (PX4_GZ_STANDALONE=1, PX4_GZ_MODEL_NAME=adr_racer). 월드 clock 토픽이 보일 때까지 기다린 뒤 실행.
  - -d 데몬 모드라 pxh 셸이 없다. 명령은 build/px4_sitl_default/bin/px4-commander, px4-param 등 클라이언트로.
  - Ctrl+C 시에도 같은 목록을 정리한다. gz 래퍼가 죽어도 'gz sim server' 는 살아남아 월드 이름을
    계속 점유하고, 그러면 다음 실행이 /gazebo/starting_world 에서 멈춰 "가제보가 안 켜진다".
    주의: gz server/gui 는 cmdline 에 월드 이름이 없어 스코프를 못 좁힌다 → 다른 gz 도 같이 죽는다.
    (자세한 내용은 _stale_patterns 주석)
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
AGENT_PORT = '8888'               # uXRCE-DDS. 에이전트 실행과 잔재 정리 패턴이 같이 쓴다
AIRFRAME = {'false': 4030, 'true': 4031}


def _stale_patterns(world: str):
    """(pkill -f 패턴, 설명). 앞에서부터 순서대로 죽인다.

    gz sim 은 래퍼(`gz sim -r <world>`) 가 `gz sim server` / `gz sim gui` 를 자식으로 띄우는데,
    래퍼가 비정상 종료해도 **서버는 살아남아 월드 이름을 계속 점유한다**. 그 상태로 다시 띄우면
    새 서버가 /gazebo/starting_world 에서 멈추고 /world/<world>/clock 이 안 올라와,
    겉보기엔 "가제보가 안 켜지는" 것처럼 보인다. → 기동 전·종료 시 모두 정리한다.

    `gz sim server` / `gz sim gui` 는 cmdline 에 월드 이름이 없어 스코프를 좁힐 수 없다.
    즉 **다른 프로젝트의 gz sim 도 같이 죽는다**. 이 워크스페이스는 한 번에 월드 하나만 쓰는
    전제라 그대로 두지만, 다른 gz 를 띄워 두고 작업한다면 이 목록에서 빼야 한다.
    나머지는 월드 이름이나 이 레포 경로로 스코프를 좁혀 둔다.
    """
    return [
        # PX4: -d 데몬이 락을 쥔 채 남으면 다음 기동이 'PX4 server already running' 으로 막힌다.
        # launch 가 cwd=build 로 띄우므로 실제 cmdline 은 상대경로(`./bin/px4 -d ...`) 다.
        # 절대경로 패턴만 두면 안 잡히므로 둘 다 본다.
        (r'px4_sitl_default/bin/px4', 'PX4 (절대경로)'),
        (r'bin/px4 -d', 'PX4 (상대경로 데몬)'),
        (r'gz sim server', 'gz 서버'),
        (r'gz sim gui', 'gz GUI'),
        (rf'gz sim .*{world}', f'gz 래퍼 ({world})'),
        # 낡은 에이전트가 포트를 쥐고 있으면 새 에이전트가 bind error(errno 98)로 즉사하고,
        # /fmu/out/* 이 안 올라와 컨트롤러가 'pos=False status=False' 로 영영 대기한다.
        # 실행 파일 이름은 설치 방식에 따라 다르므로(MicroXRCEAgent / micro-xrce-dds-agent) 포트로 잡는다.
        (rf'udp4 -p {AGENT_PORT}', f'uXRCE-DDS 에이전트 (포트 {AGENT_PORT})'),
        (r'parameter_bridge.*__node:=gz_bridge', 'ros_gz_bridge'),
        (r'adr_bringup/gate_markers', 'gate_markers'),
        (r'adr_bringup/px4_odom_to_tf', 'px4_odom_to_tf'),
        (r'rviz2.*adr\.rviz', 'rviz2'),
    ]


def _kill_stale(world: str) -> list:
    """이전 실행의 잔재를 정리하고, 실제로 죽인 것들의 설명을 돌려준다."""
    killed = []
    for pattern, desc in _stale_patterns(world):
        r = subprocess.run(['pkill', '-9', '-f', pattern],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if r.returncode == 0:        # 0 = 하나 이상 매칭해서 죽였음
            killed.append(desc)
    return killed


def _cleanup(context, *args, **kwargs):
    """다른 무엇보다 **먼저** 실행돼야 한다 — 우리 gz 가 뜬 뒤에 돌면 그걸 죽인다."""
    world = LaunchConfiguration('world').perform(context)
    killed = _kill_stale(world)
    if killed:
        return [LogInfo(msg=f'[cleanup] 이전 실행 잔재 정리: {", ".join(killed)}')]
    return []


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
        dst = af_dst / f.name
        dst.unlink(missing_ok=True)   # 예전에 걸어둔 심링크가 남아 있으면 copy2 가 링크를 따라가 실패한다
        shutil.copy2(f, dst)

    # 잔재 정리는 _cleanup 이 이 launch 의 맨 앞에서 이미 끝냈다. 여기서 또 부르면
    # 그 사이에 뜬 우리 gz 를 죽이게 된다.

    env = dict(os.environ)
    env.update({
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_MODEL_NAME': MODEL_NAME,
        'PX4_GZ_WORLD': world,
        'PX4_SYS_AUTOSTART': str(autostart),
        'PX4_SIM_MODEL': f'gz_{MODEL_NAME}',
        'PX4_UXRCE_DDS_PORT': os.environ.get('PX4_UXRCE_DDS_PORT', AGENT_PORT),
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

        # ---- 이전 실행 잔재 정리 (gz·PX4·브릿지). 반드시 gz 를 띄우기 전에! ----
        OpaqueFunction(function=_cleanup),

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
        ExecuteProcess(cmd=[agent, 'udp4', '-p', AGENT_PORT], name='xrce_agent', output='screen'),

        # ---- PX4 SITL ----
        OpaqueFunction(function=_px4),
        # Ctrl+C 시에도 정리. gz 래퍼가 먼저 죽으면 'gz sim server' 가 고아로 남아
        # 다음 실행을 막으므로, 종료 경로에서도 같은 목록을 쓸어 준다.
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: _kill_stale(
            LaunchConfiguration('world').perform(context)))),

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
