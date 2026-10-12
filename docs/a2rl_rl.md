# A2RL GT 강화학습 제어

`handoff_sungho_a2rl_20261012`의 A2RL MonoRace 맵과 M17 30M ONNX 정책을 사용한다.
학습/영상/체크포인트 전체를 가져오지 않고 맵, ONNX, 메타데이터, 설정, 최소 numpy 런타임,
해당 맵의 Gazebo 게이트 모델과 월드만 포함했다. Downloads 폴더는 실행 시 필요하지 않다.

## 실행

최초 설치 및 빌드:

```bash
cd ~/autonomous-drone-racing/adr_ws
source /opt/ros/humble/setup.bash
python3 -m pip install --user 'onnxruntime==1.23.2' 'numpy<2'
colcon build --symlink-install --packages-select adr_control adr_bringup adr_sim
source install/setup.bash
```

터미널 1 — 맵과 시뮬레이터:

```bash
ros2 launch adr_sim sim.launch.py map:=a2rl_monorace
```

터미널 2 — 기존 step1 대신:

```bash
source ~/autonomous-drone-racing/adr_ws/install/setup.bash
ros2 launch adr_bringup a2rl_rl.launch.py laps:=1
```

두 launch의 맵은 반드시 같아야 한다. RL launch는 `a2rl_monorace`가 기본값이다.
`perception`, `pnp`, `vio`, `record`, `record_root`, `origin_mode`, `land_after`는
step1과 동일하게 전달된다. 기본적으로 모두 켜고 `origin_mode:=world`를 사용한다.
OpenVINS 없이 제어만 확인하려면 `vio:=false`를 추가한다.
정책은 시뮬레이션 시간 50 Hz 고정이므로 `time_scale`은 **1.0만** 허용한다.

## 제어 경로

- 이륙: PX4 position offboard로 맵 시작점 `(-6.0909,-38.4545,1.4)`에 도달하고 첫 게이트 방향으로 정렬.
  위치·속도·yaw 조건을 GT로 확인한 뒤 RL로 전환한다. 30초 내 실패하면 착륙한다.
- RL: `/adr/ground_truth/odom`의 위치·자세·선속도·각속도를 사용한다.
  `/adr/odom`은 PX4 EKF 출력이므로 정책 상태로 사용하지 않는다. VIO/PnP도 정책에 들어가지 않는다.
- Gazebo OdometryPublisher의 twist는 기체 FLU이다. 선속도만 자세 회전으로 ENU로 변환하고
  각속도는 FLU 그대로 넣는다. quaternion 순서는 xyzw이다.
- 관측 21개: 목표 게이트 좌표계 위치/속도(6), roll/pitch/상대 yaw(3), FLU 각속도(3),
  직전 clipped action(4), 다음 게이트 상대 위치(3), 상대 yaw(1), 학습 카메라 uptilt(1).
- 정책 출력: 공칭 질량 **0.8 kg** 기준 추력과 FLU 각속도. PX4에는 FRD 각속도 `(p,-q,-r)`와
  `thrust_body=[0,0,-u]`를 `VehicleRatesSetpoint`로 보낸다. 이때 position 플래그를 끈다.
- 추력 변환은 번들 SITL 식을 유지했다. `kf=1.4e-6`, ESC 속도 300–3000 rad/s,
  PX4 기본 `THR_MDL_FAC=0`을 전제로 한다. PX4 rate PID/모터 응답은 학습 RotorPy 제어기와 다르다.
- 게이트 순서와 통과 수는 **GT 선분의 게이트 평면 교차**로 판정한다. 순서대로 정방향으로,
  기체 반경 0.134 m를 제외한 구멍 내부를 지나야 증가한다. PnP의 게이트 ID와는 별개다.
  마지막 11번을 통과하면 한 바퀴가 끝나며, 지정 바퀴 후 hold → 착륙한다.
- GT가 0.1초 이상 지연되거나 추론/제어 주기가 크게 끊긴 경우, 역통과/프레임 충돌/경계 이탈,
  20초간 게이트 진행 없음, 최대 비행시간 180초 초과 시 RL 명령을 중단하고 착륙을 요청한다.
  실패 후 자동 재시작하지 않는다.

`adr_control/config/a2rl_rl.yaml`에 타임아웃 등이 있다. CPU ONNX 추론 스레드는 1개다.
기존 gate_planner 경로는 시각화·로그 비교용으로 계속 나오지만 RL이 따라가는 목표 경로는 아니다.

## 맵과 정책의 한계

게이트 11개 평균 XY가 `(0,0)`이 되도록 원본 맵 전체를 ENU `(15.9090909,-46.4545455,0)`만큼
평행이동했다. Gazebo 게이트·패널·시작점, RL 경계용 추가 점과 기준 선회점에도 같은 이동을 적용했다.
정책 관측은 게이트 상대 위치이므로 동일하게 이동한 상태에서는 관측과 액션이 유지된다.

11개 게이트 모두 실제 게이트다. 5번(높이 5 m)과 6번(높이 1.4 m)은 같은 패널의 두 구멍이며
통과 방향이 반대다. 전달된 Gazebo 패널 형상을 유지하고 배치만 평행이동하여 중간 판의 충돌 형상도 유지한다.
현재 일반 `gen_world.py`는 이 특수 패널을 지원하지 않으므로 이 월드를 재생성하지 말 것.
전달된 월드는 텍스처 바닥/배경 장애물이 없어 기존 맵보다 VIO 특징점이 부족할 수 있다.

정책의 마지막 입력 `camera_uptilt_deg=45`는 학습 범위 40–50도의 중앙값이다.
현재 Gazebo 카메라/PnP의 실제 20도 설정을 바꾸는 옵션이 아니다. 카메라 영상을 정책에
사용하지 않지만 학습 관측 분포를 유지하기 위해 45를 넣는다.

전달된 `closed_loop_check.txt`, `eval_30M.md`, `splitS_5to6.md`에서도 이 가중치는
5번 이후 split-S를 완주하지 못한다. 이번 작업은 정책 재학습이나 비행 성능 보장이 아니라
기존 ROS/PX4 파이프라인에 해당 가중치를 연결하는 작업이다.

## 기록과 검증

기존 비행 rosbag에 `/adr/ground_truth/odom`과 `/adr/rl/debug`를 추가했다.
`/adr/rl/debug`는 50 Hz JSON String으로 obs, action/action_raw, 추력/각속도,
목표 게이트, 통과 이벤트, GT stamp, 실제 명령 송신 여부를 담는다.
`/adr/controller_state`에는 상태, 목표 게이트, 통과 수, 완료 바퀴, 종료 이유가 나온다.
세션 메타데이터에 정책 SHA-256, GT 토픽, 학습 설정, 정책 메타데이터, RL 설정을 저장한다.

번들의 `rollout_ref.npz` 1,280개 샘플을 이 레포 런타임으로 재생한 최대 오차:
obs/action `1.20e-7` 이하, 추력 `2.81e-6 N`, rate `8.95e-7 rad/s`, 목표 게이트 불일치 0개.
첫 샘플을 회귀 테스트 데이터로 포함했다.

Gazebo/PX4 통합 확인(`vio:=false`, perception/PnP/기록 활성화)에서도 이륙 → RL 전환 →
5개 게이트 통과를 확인했다. RL 실행 14.72초에 목표 6번 상태로 경계를 이탈했고,
rate 명령을 중단하고 PX4 착륙 모드로 전환했다. 이 실행은 완주 검증이 아니다.
회귀 테스트 37개(좌표 변환, 전달된 실제 정책 샘플, 공유 패널 통과/충돌,
이륙 실패·GT 지연·추론 오류·경계 이탈·완료 상태 전환 포함)가 통과했다.

```bash
source /opt/ros/humble/setup.bash
source adr_ws/install/setup.bash
ROS_DOMAIN_ID=93 python3 -m pytest adr_ws/src/adr_control/test -q
```

`adr_control/a2rl_runtime`은 전달된 `racing_runtime`의 6개 모듈이다.
로컬 변경은 ONNX 세션 단일 스레드 설정, 입출력 형태 검증, 문서 경로 안내다.

## 측정값으로 RL 실행

```bash
ros2 launch adr_bringup a2rl_rl.launch.py state_source:=measured laps:=1
```

`state_source:=gt`가 기존 기본값이며, `measured`에는 `vio:=true`가 필요하다.
측정값 모드의 신경망 입력은 다음과 같다.

| 입력 | 소스 |
|---|---|
| 위치 | `/adr/vio/odom`의 map 정렬된 OpenVINS 위치 (PnP 보정 전) |
| 선속도 | PX4 `/fmu/out/vehicle_odometry` velocity, NED → ENU |
| 자세 | 같은 PX4 메시지 q, NED/FRD → ENU/FLU, xyzw |
| 각속도 | 같은 PX4 메시지 angular_velocity, FRD → FLU |

측정값 모드에서만 OpenVINS에 `try_zupt=true`, `zupt_only_at_beginning=true`를 적용해
정지 상태에서도 초기화할 수 있게 한다. 움직임 감지 이후에는 초기 정지 보정을 종료한다.
기존 GT/step1의 VIO 초기화 설정은 유지한다. 먼저 PX4 위치 제어로 이륙한다. 시작점에서 위치·속도·yaw가
안정되고 신선한 VIO 위치가 들어와야 RL로 전환한다. 30초 안에 준비되지 않으면 착륙한다.
RL 게이트 통과 판정도 선택한 위치를 사용한다. 완료 후 hold/착륙은 PX4 위치 제어다.
GT 모드와 측정값 모드 모두 기본 50 Hz 정책을 유지한다.

VIO는 메시지 시각 기준 0.20초, PX4는 ROS 수신 시각 기준 0.10초 이내여야 한다.
서로 다른 clock인 PX4 boot timestamp를 ROS 시각과 직접 빼지 않는다.
가장 최근의 두 소스를 조합하며 시간 보간/지연 예측은 하지 않는다.
측정값 유실·잘못된 frame·NaN은 GT로 대체하지 않고 RL 중단/착륙 처리한다.
`/adr/rl/debug`에 입력 상태, 소스, VIO 시각, PX4 수신 시각을 남긴다.

현재 `sim.launch.py ev:=true`는 PX4 EKF에 Gazebo 외부 위치를 주입한다.
따라서 measured는 **정책의 GT 직접 입력을 없애는 옵션**이며, PX4 추정기 자체까지
GT 독립으로 만드는 옵션은 아니다. VIO의 최초 map 정렬도 기존 PX4 기준 TF를 사용한다.

2026-10-13 수정 검증: 기존 측정값 실행은 VIO odometry 0건으로 30초 후 착륙했다.
정지 초기화 적용 후 Gazebo에서 VIO 초기화·map 정렬과 TAKEOFF → TRACK 전환을 확인했다.
이륙 대기 로그에 위치 오차, 속도, VIO 수신 유무/나이를 출력한다.
