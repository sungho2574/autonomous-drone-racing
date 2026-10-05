"""시뮬 전체를 한 번에: gz Harmonic + MicroXRCEAgent + PX4 SITL + ros_gz_bridge + rviz.

    ros2 launch adr_sim sim.launch.py [map:=cross] [ev:=true] [gui:=false] [rviz:=false] [soft_gl:=0]

이후 별도 터미널에서 미션만:  ros2 launch adr_bringup step1.launch.py [map:=cross]
(두 launch 의 map 은 같아야 한다 — 다르면 게이트 위치와 월드가 어긋난다)

인자
  map       : 맵 이름 = 월드 이름 = assets/worlds/<맵>.sdf (기본 cross).
              맵 정의는 adr_bringup/config/maps/<맵>.yaml 하나뿐이고, gate_markers 도 이걸 읽는다.
              새 맵을 추가했거나 맵 yaml 을 고쳤으면 먼저
                python3 adr_ws/src/adr_sim/scripts/gen_world.py --map <맵>   (또는 --all)
              을 돌려 월드를 만들어야 한다. 없으면 여기서 그 명령을 알려주며 멈춘다.
  ev        : true(기본) 면 airframe 4031 — gz 진실값을 외부 비전으로 주입(mocap 에뮬레이션).
              PX4 local 프레임 == gz 월드 == map 프레임이라 맵 yaml 좌표를 그대로 쓴다.
              false 면 4030(GPS 시뮬): local 원점이 부팅 위치가 되므로 step1.launch.py 에 origin_mode:=start 필요,
              자기장 편각 차이로 yaw 도 수 도 어긋난다. 실기체(mocap)와 같은 경로는 ev=true.
  gui       : gz GUI (false 면 headless -s)
  rviz      : rviz2 실행
  px4_dir   : PX4-Autopilot 위치 (기본 $PX4_DIR 또는 ~/PX4-Autopilot). make px4_sitl 이 끝나 있어야 함
  agent     : DDS 에이전트 실행 파일 (소스 빌드: MicroXRCEAgent, snap: micro-xrce-dds-agent)
  soft_gl   : 0 이면 GPU 렌더링, 1 이면 llvmpipe (GPU 없는 VM). 기본 $ADR_SOFT_GL 또는 0

PX4 기동 방식 (ARMS 의 px4_sitl.launch.py 와 같은 패턴, 셸 스크립트 없음)
  - assets/px4/airframes/* 를 매번 $PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes/ 에 복사
    → PX4 소스(ROMFS) 수정·재빌드 불필요. make px4_sitl 이 etc 를 다시 만들어도 다음 실행에 다시 들어간다.
  - 이전 실행 잔재(gz·PX4·브릿지)를 먼저 정리한 뒤, gz 는 ros_gz_sim 이 띄우고 PX4 는 standalone 으로 붙는다
    (PX4_GZ_STANDALONE=1, PX4_GZ_MODEL_NAME=adr_racer). 월드 clock 토픽이 보일 때까지 기다린 뒤 실행.
  - -d 데몬 모드라 pxh 셸이 없다. 명령은 build/px4_sitl_default/bin/px4-commander, px4-param 등 클라이언트로.
  - 잔재 정리는 **기동 시에만** 한다. 종료 시엔 안 한다 — 패턴이 인스턴스를 구분 못 해서,
    새 sim 을 띄우면 옛 launch 의 종료 핸들러가 새 sim 의 자식을 죽이는 연쇄가 생긴다.
    주의: gz server/gui 는 cmdline 에 월드 이름이 없어 스코프를 못 좁힌다 → 다른 gz 도 같이 죽는다.
    (자세한 내용은 _stale_patterns / 아래 종료 관련 주석)
"""
import os
import shutil
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression, TextSubstitution)
from launch_ros.actions import Node

from adr_bringup.launch_cleanup import kill_stale

SIM_SHARE = Path(get_package_share_directory('adr_sim'))
ASSETS = SIM_SHARE / 'assets'
MODEL_NAME = 'adr_racer'          # worlds/*.sdf 의 <include><name> 과 동일
AGENT_PORT = '8888'               # uXRCE-DDS. 에이전트 실행과 잔재 정리 패턴이 같이 쓴다
AIRFRAME = {'false': 4030, 'true': 4031}
BRINGUP_SHARE = Path(get_package_share_directory('adr_bringup'))
MAPS = BRINGUP_SHARE / 'config' / 'maps'


def _gen_world_cmd(name: str) -> str:
    return f'python3 <워크스페이스>/src/adr_sim/scripts/gen_world.py --map {name}   (전부: --all)'


def _check_map(context, *args, **kwargs):
    """맵 yaml 과 생성된 월드 SDF 가 둘 다 있는지, 월드가 맵보다 낡지 않았는지 본다.

    gz 는 없는 월드를 줘도 조용히 빈 화면을 띄우고 /world/<맵>/clock 이 안 올라와
    PX4 가 60 s 기다린 끝에 죽는다 — 그 전에 여기서 원인을 말하고 멈춘다.
    """
    name = LaunchConfiguration('map').perform(context)
    installed = sorted(f.stem for f in MAPS.glob('*.yaml'))
    mp, sdf = MAPS / f'{name}.yaml', ASSETS / 'worlds' / f'{name}.sdf'
    if not mp.exists():
        raise RuntimeError(f'맵 "{name}" 없음: {mp}\n'
                           f'  설치된 맵: {", ".join(installed) or "(없음)"}\n'
                           f'  config/maps/ 에 방금 추가했다면 colcon build 를 먼저 (share 로 설치돼야 보인다)')
    if not sdf.exists():
        raise RuntimeError(f'맵 "{name}" 의 월드가 아직 생성되지 않았다: {sdf}\n'
                           f'  → {_gen_world_cmd(name)}\n  그 다음 colcon build')
    if sdf.stat().st_mtime < mp.stat().st_mtime:
        return [LogInfo(msg=f'[map] ⚠ {sdf.name} 이 {mp.name} 보다 낡았다 — 맵 수정이 반영되지 않았을 수 있다. '
                            f'{_gen_world_cmd(name)}')]
    return [LogInfo(msg=f'[map] {name}  (월드 {sdf}, 맵 {mp})')]


def _stale_patterns(world: str):
    """(pkill -f 패턴, 설명). 앞에서부터 순서대로 죽인다.

    gz sim 은 래퍼(`gz sim -r <world>`) 가 `gz sim server` / `gz sim gui` 를 자식으로 띄우는데,
    래퍼가 비정상 종료해도 **서버는 살아남아 월드 이름을 계속 점유한다**. 그 상태로 다시 띄우면
    새 서버가 /gazebo/starting_world 에서 멈추고 /world/<world>/clock 이 안 올라와,
    겉보기엔 "가제보가 안 켜지는" 것처럼 보인다. → **기동 전에만** 정리한다(종료 시엔 하지 않는다).

    `gz sim server` / `gz sim gui` 는 cmdline 에 월드 이름이 없어 스코프를 좁힐 수 없다.
    즉 **다른 프로젝트의 gz sim 도 같이 죽는다**. 이 워크스페이스는 한 번에 월드 하나만 쓰는
    전제라 그대로 두지만, 다른 gz 를 띄워 두고 작업한다면 이 목록에서 빼야 한다.
    나머지는 월드 이름이나 이 레포 경로로 스코프를 좁혀 둔다.
    공통 주의사항은 adr_bringup/launch_cleanup.py 주석 참고.
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




def _cleanup(context, *args, **kwargs):
    """다른 무엇보다 **먼저** 실행돼야 한다 — 우리 gz 가 뜬 뒤에 돌면 그걸 죽인다."""
    world = LaunchConfiguration('map').perform(context)
    killed = kill_stale(_stale_patterns(world))
    if killed:
        return [LogInfo(msg=f'[cleanup] 이전 실행 잔재 정리: {", ".join(killed)}')]
    return []


def _px4(context, *args, **kwargs):
    px4_dir = Path(os.path.expanduser(LaunchConfiguration('px4_dir').perform(context)))
    world = LaunchConfiguration('map').perform(context)
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
    gz_launch = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world = LaunchConfiguration('map')       # 월드 이름 == 맵 이름 == SDF 파일 이름
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    agent = LaunchConfiguration('agent')
    soft_gl = LaunchConfiguration('soft_gl')
    world_sdf = [str(ASSETS / 'worlds') + '/', world, '.sdf']
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='cross'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ev', default_value='true'),
        DeclareLaunchArgument('px4_dir', default_value=EnvironmentVariable(
            'PX4_DIR', default_value=os.path.expanduser('~/PX4-Autopilot'))),
        DeclareLaunchArgument('agent', default_value='MicroXRCEAgent'),
        DeclareLaunchArgument('soft_gl', default_value=EnvironmentVariable('ADR_SOFT_GL', default_value='0')),

        # ---- 맵/월드 확인 (아무것도 띄우기 전에 원인을 알려주고 멈춘다) ----
        OpaqueFunction(function=_check_map),
        # ---- 이전 실행 잔재 정리 (gz·PX4·브릿지). 반드시 gz 를 띄우기 전에! ----
        OpaqueFunction(function=_cleanup),

        # ---- 환경: 모델 검색 경로, 소프트웨어 GL ----
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(ASSETS / 'models')),
        # GPU 모드에서는 부모 셸의 소프트웨어 렌더링 강제 설정도 제거한다.
        UnsetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE',
                                 condition=IfCondition(PythonExpression(["'", soft_gl, "' == '0'"]))),
        UnsetEnvironmentVariable('GALLIUM_DRIVER',
                                 condition=IfCondition(PythonExpression(["'", soft_gl, "' == '0'"]))),
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
        # 종료 시점에는 일부러 아무것도 쓸지 않는다.
        # 예전에 OnShutdown 에서도 같은 패턴으로 pkill 했는데, 그게 연쇄 사고를 냈다:
        #   새 sim 기동 → _cleanup 이 옛 sim 의 자식을 kill → 옛 launch 가 종료 절차 시작
        #   → 그 OnShutdown 이 같은 패턴으로 pkill → **방금 뜬 새 sim** 의 자식을 죽인다.
        # 패턴이 인스턴스를 구분하지 못하니 종료 경로에서 쓰면 안 된다.
        # 고아가 남더라도 다음 실행의 _cleanup 이 정리하므로 기동 시 정리만으로 충분하다.

        # ---- gz ↔ ROS 브릿지 (clock, 카메라, 진실값) ----
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             parameters=[{'config_file': str(SIM_SHARE / 'config' / 'gz_bridge.yaml'), **sim_time}],
             output='screen'),

        # ---- 시각화 (adr_bringup 공통 노드) ----
        Node(package='adr_bringup', executable='gate_markers', name='gate_markers',
             parameters=[{'gates_file': PathJoinSubstitution(
                 [str(MAPS), [LaunchConfiguration('map'), TextSubstitution(text='.yaml')]]),
                          **sim_time}], output='screen'),
        Node(package='adr_bringup', executable='px4_odom_to_tf', name='px4_odom_to_tf',
             parameters=[sim_time], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', str(BRINGUP_SHARE / 'config' / 'adr.rviz')],
             parameters=[sim_time], condition=IfCondition(rviz), output='screen'),
    ])
