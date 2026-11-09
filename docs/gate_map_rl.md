# gate_map RL (2026-11-09)

`handoff_sungho_gatemap_20261108.zip`의 `main`(25M), `slow`(5M)를 각각
`gate_map_main`, `gate_map_slow`로 설치한다. 기존 A2RL 정책은 그대로 유지한다.
시작점은 (-2, -2.8, 0.75), 게이트 좌표는 `scripts/gate_map/gate_map.yaml`과 같다.
원본 파일은 수정하지 않았으며, ROS 설치용 맵에는 학습 설정의 start와 thickness만 보충했다.
정책별 `training_map.yaml`은 원본 지도 사본이다. Downloads/zip 없이 실행 가능하다.

## 빌드

```bash
cd ~/autonomous-drone-racing/adr_ws
source /opt/ros/humble/setup.bash
python3 -m pip install --user 'onnxruntime==1.23.2' 'numpy<2'
colcon build --symlink-install --packages-select adr_control adr_bringup adr_sim
source install/setup.bash
```

## 실행 (ROS 2 / Gazebo / PX4가 설치된 Linux)

터미널 1:

```bash
ros2 launch adr_sim sim.launch.py map:=gate_map camera_tilt_deg:=20
```

터미널 2 (먼저 GT 입력으로 slow 확인):

```bash
source /opt/ros/humble/setup.bash
source ~/autonomous-drone-racing/adr_ws/install/setup.bash
ros2 launch adr_bringup gate_map_rl.launch.py policy:=gate_map_slow state_source:=gt laps:=2 camera_tilt_deg:=20
```

종료 후 main으로 교체하려면 `policy:=gate_map_main`을 쓴다.
OpenVINS+PnP 위치 입력은 `state_source:=measured`, raw VIO 비교는 여기에
`rl_position_topic:=/adr/vio/odom`을 추가한다. measured의 기본 위치 토픽은
`/adr/state/corrected`, 속도/자세/각속도는 PX4 입력이다. 기존 launch처럼
state_source 기본은 measured이며 gate_map wrapper의 정책 기본은 slow다.
`a2rl_rl.launch.py policy:=gate_map_main`으로 실행해도 map이 gate_map으로 선택된다.
카메라 각도 변경은 두 터미널에 같은 값을 준다. 정책 관측 각도는 45°로 유지한다.

## 기하와 런타임

- 방: X [-4,4], Y [-4,7], Z [0,3.5] m. 벽 네 면과 천장을 Gazebo에 생성하고
  기존 텍스처 바닥을 사용한다. 실내에 학습에 없던 기둥/상자를 추가하지 않는다.
- 학습 wall_clear=0.18에 따라 판정 경계는 [-3.82,-3.82,0.18]~[3.82,6.82,3.32].
  config의 z_max=8은 room 없는 맵에만 적용되므로 이 맵의 천장은 8 m가 아니다.
- 게이트 외경 1.5 / 내경 1.2 / 두께 0.05 m, inflate=0.18로 통과 반폭은 0.42 m.
- G2/G3는 XY가 같지만 높이와 통과 방향이 다르다. 정책의 목표는 순차 평면 통과로
  갱신하고 PnP 관측 ID로 정책의 목표를 덮어쓰지 않는다. 기존 PnP 투영 매칭을 유지한다.
- 현재 저장소의 비행 중 기하 이벤트 비중단 동작은 유지한다. room 이탈은 이벤트에
  기록되며, 벽/게이트는 Gazebo 충돌 형상을 갖는다. 추가 자동 hold 로직은 넣지 않았다.
- 정책 50 Hz, ENU/FLU 및 xyzw 규약, 추력 변환은 기존 경로를 사용한다.
- 선택 정책·해시·학습 설정·실효 맵은 기존 세션 메타데이터에 저장된다.

## ROS 없이 검증

저장소 루트에서:

```bash
PYTHONPATH=adr_ws/src/adr_control python3 -m adr_control.a2rl_runtime.verify --policy adr_ws/src/adr_control/policies/gate_map_slow --map adr_ws/src/adr_bringup/config/maps/gate_map.yaml
```

main은 경로의 gate_map_slow를 gate_map_main으로 바꾼다. 각각 17,600개 기록 상태의
관측·액션·추력·각속도·목표 게이트를 비교한다. 이는 재생 검증이며 실제 ROS/PX4
완주율 검증과 다르다. 전달된 98.4%/100%는 제공자의 학습 환경 평가 결과다.
