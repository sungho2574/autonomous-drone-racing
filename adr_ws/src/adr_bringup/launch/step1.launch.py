"""step1 파이프라인: gate_planner(min-snap) + px4_position_controller (+ perception, PnP, VIO).

선행: ros2 launch adr_sim sim.launch.py (gz+PX4+Agent+rviz 일괄) 또는 real.launch.py
인자: map:=cross   비행할 맵 = adr_bringup/config/maps/<맵>.yaml. **sim.launch.py 의 map 과 같아야 한다**
      laps:=2  time_scale:=1.0  land_after:=true  perception:=true  pnp:=true  use_sim_time:=true
      v_avg:=3.0  a_max:=8.0   궤적 공격성(planner.yaml 기본값 덮어쓰기). a_max 가 클수록 기울기가 커지는데
                              PX4 MPC_TILTMAX_AIR(기본 45°) 를 넘으면 세트포인트를 못 따라간다
      origin_mode:=world|start  (PX4 local 원점이 map 과 다를 때 = GPS 시뮬 모드(ev:=false) 면 start)
      vio:=true|false           OpenVINS VIO + drift 보정 KF (기본 true). OpenVINS 미설치면 launch 가 멈추며
                                설치 절차를 안내한다. 끄려면 vio:=false
"""
import os
import json
import time
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction, SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from adr_bringup.launch_cleanup import kill_stale
from adr_bringup.flight_session import new_session, repository_root


def _stale_patterns():
    """step1 이 띄우는 노드들만. sim.launch.py 가 띄우는 것(gz·PX4·gz_bridge·gate_markers·rviz)은
    건드리지 않는다 — T1 은 보통 계속 띄워 둔 채 T2 만 다시 돌리기 때문이다.

    imu_bridge 는 sim 쪽 gz_bridge 와 같은 parameter_bridge 실행파일이라
    반드시 `__node:=imu_bridge` 로 좁혀야 한다. 안 그러면 T1 의 브릿지까지 죽는다.
    """
    return [
        (r'adr_planning/gate_planner', 'gate_planner'),
        (r'adr_control/px4_position_controller', 'px4_position_controller'),
        (r'adr_control/a2rl_rl_controller', 'a2rl_rl_controller'),
        (r'adr_perception/gate_detector', 'gate_detector'),
        (r'adr_perception/gate_pnp', 'gate_pnp'),
        (r'adr_state_estimation/drift_corrector', 'drift_corrector'),
        (r'adr_vio/vio_align', 'vio_align'),
        (r'adr_vio/vio_metrics', 'vio_metrics'),
        (r'adr_bringup/flight_recorder', 'flight_recorder'),
        (r'ov_msckf/run_subscribe_msckf', 'ov_msckf'),
        (r'parameter_bridge.*__node:=imu_bridge', 'imu_bridge'),
    ]


def _cleanup(context, *args, **kwargs):
    """다른 무엇보다 **먼저** 실행돼야 한다 — 우리 노드가 뜬 뒤에 돌면 그걸 죽인다.
    종료 시에는 하지 않는다(이유는 adr_bringup/launch_cleanup.py 주석)."""
    killed = kill_stale(_stale_patterns())
    if killed:
        return [LogInfo(msg=f'[cleanup] 이전 실행 잔재 정리: {", ".join(killed)}')]
    return []


def _await_recording(context, actions, folder, started):
    meta=json.loads((Path(folder)/'session.json').read_text())
    if meta['status']=='failed':
        raise RuntimeError(f"Flight recorder failed before takeoff: {meta.get('error')}")
    if (Path(folder)/'ready.json').is_file():
        return actions
    if time.monotonic()-started>20:
        from adr_bringup.flight_session import atomic_json
        meta.update(status='failed',error='Recorder readiness timeout; flight not started')
        atomic_json(Path(folder)/'session.json',meta)
        raise RuntimeError(f'Flight recorder not ready; flight was not started: {folder}')
    return [TimerAction(period=.2,actions=[OpaqueFunction(function=_await_recording,
            kwargs=dict(actions=actions,folder=folder,started=started))])]


def _recorded_start(context, actions, controller='position'):
    def arg(name): return LaunchConfiguration(name).perform(context)
    if arg('record').lower() not in ('true','1'):
        return actions
    bringup=Path(get_package_share_directory('adr_bringup'))
    root_arg=arg('record_root')
    repo=None
    try: repo=repository_root(__file__,bringup,Path.cwd())
    except RuntimeError:
        if not root_arg: raise
    root=Path(root_arg).expanduser() if root_arg else repo/'flight_logs'
    params={k:arg(k) for k in ('map','laps','time_scale','land_after','perception','pnp',
            'origin_mode','vio','vio_profile','vio_timing_path','heading_mode','v_avg','a_max','use_sim_time')}
    params['controller'] = controller
    snapshots={}
    if controller == 'a2rl':
        import hashlib
        policy_dir = Path(get_package_share_directory('adr_control')) / 'policies/a2rl_monorace'
        params['policy_sha256'] = hashlib.sha256((policy_dir/'policy.onnx').read_bytes()).hexdigest()
        params['rl_state_source'] = arg('state_source')
        params['rl_position_topic'] = '/adr/ground_truth/odom' if arg('state_source') == 'gt' else '/adr/vio/odom'
        for filename in ('policy_meta.json', 'config_asrun.yaml'):
            path = policy_dir / filename
            snapshots['rl/'+filename] = {'path':str(path), 'text':path.read_text()}
        path = Path(get_package_share_directory('adr_control')) / 'config/a2rl_rl.yaml'
        snapshots['rl/controller'] = {'path':str(path), 'text':path.read_text()}
    sources={'map':bringup/'config/maps'/f"{arg('map')}.yaml",'planner':bringup/'config/planner.yaml'}
    for pkg,files in {'adr_vio':['estimator_config.yaml','kalibr_imucam_chain.yaml','kalibr_imu_chain.yaml'],
                      'adr_control':['controller.yaml'],'adr_perception':['gate_detector.yaml','gate_pnp.yaml'],
                      'adr_state_estimation':['drift_corrector.yaml']}.items():
        for filename in files: sources[pkg+'/'+filename]=Path(get_package_share_directory(pkg))/'config'/filename
    for key,path in sources.items(): snapshots[key]={'path':str(path),'text':path.read_text()}
    estimator=yaml.safe_load('\n'.join(line for line in snapshots['adr_vio/estimator_config.yaml']['text'].splitlines() if not line.startswith('%YAML')))
    params['effective_vio']={key:estimator.get(key) for key in ('max_clones','max_slam','max_slam_in_update','max_msckf_in_update','num_pts','track_frequency')}
    if arg('vio_profile')=='slam_dense':
        params['effective_vio'].update(max_slam=100,max_slam_in_update=50)
    if controller == 'a2rl' and arg('state_source') == 'measured':
        params['effective_vio'].update(try_zupt=True, zupt_only_at_beginning=True)
    definitions={}
    for path in (Path(get_package_share_directory('adr_msgs'))/'msg').glob('*.msg'):
        definitions['adr_msgs/msg/'+path.stem]=path.read_text()
    if 'adr_msgs/msg/PolynomialTrajectory' not in definitions:
        raise RuntimeError('Installed adr_msgs/msg definitions missing; rebuild adr_msgs before recording.')
    folder=new_session(root,arg('map'),params,snapshots,definitions,repo)
    timing=arg('vio_timing_path') or str(folder/'openvins_timing.csv')
    return [SetLaunchConfiguration('vio_timing_path',timing),
            LogInfo(msg=f'[flight] Metadata saved: {folder}'),
            Node(package='adr_bringup',executable='flight_recorder',name='flight_recorder',
                 parameters=[{'session_dir':str(folder),'use_sim_time':arg('use_sim_time').lower()=='true'}],output='screen'),
            TimerAction(period=.2,actions=[OpaqueFunction(function=_await_recording,
                 kwargs=dict(actions=actions,folder=str(folder),started=time.monotonic()))])]


def _check_rl(context):
    source = LaunchConfiguration('state_source').perform(context)
    if source not in ('gt', 'measured'):
        raise RuntimeError('state_source must be gt or measured')
    if source == 'measured' and LaunchConfiguration('vio').perform(context).lower() not in ('true', '1'):
        raise RuntimeError('state_source:=measured requires vio:=true')
    if LaunchConfiguration('map').perform(context) != 'a2rl_monorace':
        raise RuntimeError('Bundled A2RL policy requires map:=a2rl_monorace in both sim and bringup')
    if (float(LaunchConfiguration('time_scale').perform(context)) != 1.0
            or LaunchConfiguration('use_sim_time').perform(context).lower() != 'true'):
        raise RuntimeError('A2RL requires time_scale:=1.0 and use_sim_time:=true')
    try:
        import onnxruntime  # noqa: F401
    except ImportError as exc:
        raise RuntimeError('Install RL runtime: python3 -m pip install --user onnxruntime==1.23.2') from exc
    return []


def generate_launch_description(controller='position'):
    bringup_share = get_package_share_directory('adr_bringup')
    # map:= → config/maps/<맵>.yaml. PathJoinSubstitution 은 Substitution 하나로 평가되므로
    # 파라미터가 문자열 배열로 잘못 해석될 여지가 없다.
    gates_file = PathJoinSubstitution([
        bringup_share, 'config', 'maps',
        [LaunchConfiguration('map'), TextSubstitution(text='.yaml')]])
    control_share = get_package_share_directory('adr_control')
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    pipeline = [
        Node(package='adr_planning', executable='gate_planner', name='gate_planner',
             parameters=[os.path.join(bringup_share, 'config', 'planner.yaml'),
                         {'laps': LaunchConfiguration('laps'),
                          'v_avg': LaunchConfiguration('v_avg'),
                          'a_max': LaunchConfiguration('a_max'),
                          'heading_mode': LaunchConfiguration('heading_mode'),
                          'gates_file': gates_file,
                          **sim_time}],
             output='screen'),
        Node(package='adr_control',
             executable='a2rl_rl_controller' if controller == 'a2rl' else 'px4_position_controller',
             name='a2rl_rl_controller' if controller == 'a2rl' else 'px4_position_controller',
             parameters=[os.path.join(control_share, 'config',
                                      'a2rl_rl.yaml' if controller == 'a2rl' else 'controller.yaml'),
                         {**({'laps': LaunchConfiguration('laps'),
                              'state_source': LaunchConfiguration('state_source')} if controller == 'a2rl' else {}),
                          'time_scale': ParameterValue(LaunchConfiguration('time_scale'), value_type=float),
                          'land_after': LaunchConfiguration('land_after'),
                          'origin_mode': LaunchConfiguration('origin_mode'),
                          # origin_mode=start 일 때 start 좌표를 여기서 읽는다 — planner 와 같은 맵이어야 한다
                          'gates_file': gates_file,
                          **sim_time}],
             output='screen'),
        Node(package='adr_perception', executable='gate_detector', name='gate_detector',
             parameters=[os.path.join(get_package_share_directory('adr_perception'), 'config', 'gate_detector.yaml'),
                         sim_time], condition=IfCondition(LaunchConfiguration('perception')),
             output='screen'),
        # PnP 위치 추정 — 시각화/평가 전용. 제어에는 절대 안 들어간다 (TF pnp_base_link 로만 나감)
        Node(package='adr_perception', executable='gate_pnp', name='gate_pnp',
             parameters=[os.path.join(get_package_share_directory('adr_perception'), 'config', 'gate_pnp.yaml'),
                         {'gates_file': gates_file},
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
                              'profile': LaunchConfiguration('vio_profile'),
                              'stationary_init': PythonExpression([
                                  "'true' if '", LaunchConfiguration('state_source'),
                                  "' == 'measured' else 'false'"]) if controller == 'a2rl' else 'false',
                              'timing_path': LaunchConfiguration('vio_timing_path'),
                              'imu_bridge': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('vio'))),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        *([DeclareLaunchArgument('state_source', default_value='gt',
              description='RL input: gt or measured (OpenVINS position + PX4 velocity/attitude/rates)')]
          if controller == 'a2rl' else []),
        DeclareLaunchArgument('laps', default_value='1' if controller == 'a2rl' else '2'),
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
        DeclareLaunchArgument('vio_profile', default_value='default'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('record_root', default_value=''),
        DeclareLaunchArgument('vio_timing_path', default_value=''),
        # 맵 = 코스 정의의 단일 진실 원천. planner/markers/pnp 로, 그리고 vio 의 gz IMU 토픽 경로(월드 이름)로 간다.
        DeclareLaunchArgument('map', default_value='a2rl_monorace' if controller == 'a2rl' else 'cross'),
        # 궤적 공격성. 빈 값이면 planner.yaml 값을 쓴다 — 논문 트랙처럼 코너가 급한 맵은 낮춰야 한다.
        DeclareLaunchArgument('v_avg', default_value='3.0'),
        DeclareLaunchArgument('a_max', default_value='8.0'),
        DeclareLaunchArgument('heading_mode', default_value='perception_aware'),

        # ---- 이전 실행 잔재 정리. 반드시 우리 노드를 띄우기 전에! ----
        # 고아 노드가 남으면 latched 토픽(/adr/trajectory, /adr/planned_path)의 퍼블리셔가 둘이 되어
        # rviz·컨트롤러가 낡은 샘플을 물고, 경로가 '떴다 안 떴다' 한다.
        *([OpaqueFunction(function=_check_rl)] if controller == 'a2rl' else []),
        OpaqueFunction(function=_cleanup),

        OpaqueFunction(function=_recorded_start, kwargs={'actions': pipeline, 'controller': controller}),
    ])
