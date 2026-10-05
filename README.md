# autonomous-drone-racing

전방 카메라로 게이트를 인식해 통과하는 자율 드론 레이싱 검증용 ROS 2 워크스페이스.
시뮬(PX4 SITL + Gazebo Harmonic)에서 단계별로 검증한 뒤 250급 레이싱 기체 + 모션캡처로 실기체 검증한다.

|                   | step 1 (현재)                   | step 2               | step 3                  |
| ----------------- | ------------------------------- | -------------------- | ----------------------- |
| 게이트 인식       | (x) 맵 사용                     | (x)                  | Gatenet                 |
| 위치 추정         | motion capture                  | motion capture       | OpenVINS + gate map PnP |
| 경로 계획 및 제어 | min-snap + PX4 Position Control | RL + rate controller | RL + rate controller    |

## 구성

```
autonomous-drone-racing/
├── docs/step1_architecture.md    # 기술 문서 (설계·좌표계·설치·실행·트러블슈팅)
└── adr_ws/src/
    ├── adr_msgs/                 # msg: PolynomialTrajectory, GateDetectionArray(2D), GateArray(3D)
    ├── adr_sim/                  # 시뮬 일괄 launch(gz+PX4+Agent+rviz), assets/{models,worlds,px4}
    ├── adr_video/                # 카메라 입력 (sim: ros_gz_bridge / 실기체: 드라이버 relay)
    ├── adr_perception/           # 게이트 인식 (주황 HSV 기반 2D 검출; step3: Gatenet + PnP)
    ├── adr_planning/             # min-snap 궤적 생성
    ├── adr_control/              # PX4 offboard 제어 (step1: position control)
    ├── adr_bringup/              # 미션 launch(step1), 게이트 맵(config/maps/*.yaml), rviz 노드, mocap 브릿지
    ├── adr_vio/                  # (선택) OpenVINS VIO — 설정 생성·map 정렬·rviz 궤적. 제어 미반영
    ├── px4_msgs/                 # submodule (release/1.16)
    └── motion_capture_tracking/  # submodule (Qualisys 등 mocap → /poses, TF)
```

## 실행 환경

- Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic (8.x)
- PX4-Autopilot v1.18.x, Micro-XRCE-DDS-Agent — **이 레포 밖**에 설치 (`PX4_DIR`, 기본 `~/PX4-Autopilot`)
- 코드는 macOS 에서 작성, 빌드/실행은 Ubuntu VM 에서 (Mac 에서는 `pytest` 로 min-snap 만 검증)

> `px4_msgs` submodule 은 PX4 본체와 **같은 버전**이어야 한다. 어긋나면 빌드는 통과하지만
> DDS 타입이 안 맞아 `/fmu/out/*` 이 조용히 안 들어온다(arm/offboard 전환이 멈춘 것처럼 보인다).

## 설치

### ros_gz (Harmonic 용 소스 빌드, 최초 1회)

apt 의 `ros-humble-ros-gz-*` 는 Gazebo **Fortress**(`gz-msgs 8`/`ign-gazebo 6`) 기준으로 빌드돼 있어,
Harmonic 월드를 띄우면 Fortress 로 실행되며 렌더 스레드에서 죽는다. Harmonic 으로 직접 빌드해야 한다.

```bash
sudo apt install libgz-msgs10-dev libgz-transport13-dev -y
mkdir -p ~/ros_gz_harmonic_ws/src && cd ~/ros_gz_harmonic_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
cd ~/ros_gz_harmonic_ws && source /opt/ros/humble/setup.bash
GZ_VERSION=harmonic colcon build --packages-select ros_gz_interfaces ros_gz_bridge ros_gz_sim ros_gz_image
```

### 워크스페이스

메시(`*.stl`)는 git LFS 로 관리하므로 클론 전에 `git lfs install` 이 한 번 필요하다.
`--recursive` 를 빠뜨리면 `src/px4_msgs` 가 빈 디렉터리라 colcon 이 실패한다.
이미 clone 했다면 `git submodule update --init --recursive`.

```bash
sudo apt install git-lfs && git lfs install
git clone --recursive https://github.com/sungho2574/autonomous-drone-racing.git
cd autonomous-drone-racing/adr_ws
source ~/ros_gz_harmonic_ws/install/setup.bash   # apt 의 Fortress 판보다 먼저
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### PX4

`make px4_sitl` 만 끝나 있으면 된다(airframe 은 실행 시 자동 주입, PX4 소스 수정·재빌드 불필요):

```bash
export PX4_DIR=~/PX4-Autopilot      # 클론 위치가 다르면 그 경로로. 기본값이면 생략 가능
(cd $PX4_DIR && make px4_sitl)
```

### OpenVINS (VIO 를 쓸 때만, 최초 1회)

**`adr_vio`(VIO)는 설치를 따로 해야 돌아간다.** OpenVINS 는 레포에 넣지 않았다 — 그 레포의
`ov_data`(공개 데이터셋 groundtruth)가 376 MB 인데 우리는 안 쓴다. 아래 스크립트가 필요한 3개
패키지만 sparse·shallow 로 `adr_ws/src/open_vins` 에 받는다(~15 MB, `.gitignore` 대상이라
`git clone`/`git pull` 로는 안 따라온다. 워크스페이스를 밀었다면 다시 받아야 한다).

```bash
sudo apt install libeigen3-dev libboost-all-dev libopencv-dev libceres-dev
adr_ws/src/adr_vio/scripts/setup_openvins.sh
cd adr_ws && colcon build --symlink-install && source install/setup.bash
ros2 pkg list | grep ov_        # ov_core ov_init ov_msckf 세 개가 나와야 한다
```

안 하면 step1 launch 가 설치 안내와 함께 멈춘다(그냥 비행만 할 거면 `vio:=false` 로 끄면 된다).

## 실행 (시뮬, 터미널 2개)

두 터미널 모두 먼저:

```bash
source /opt/ros/humble/setup.bash && source ~/ros_gz_harmonic_ws/install/setup.bash && source ~/autonomous-drone-racing/adr_ws/install/setup.bash
```

```bash
# T1 — gz 월드 + MicroXRCEAgent + PX4 SITL + 브릿지 + rviz 를 한 번에
ros2 launch adr_sim sim.launch.py
```

```bash
# T2 — min-snap 계획 + position control (자동 arm → 이륙 → 2바퀴 → 착륙)
ros2 launch adr_bringup step1.launch.py laps:=2 time_scale:=1.0
```

위 명령은 VIO(OpenVINS) + drift 보정 KF 까지 **기본으로 같이 띄운다** — 터미널 수는 그대로 2개다.
**단, 위 "OpenVINS" 설치가 먼저 돼 있어야 한다** ([docs §13](docs/step1_architecture.md#13-vio-openvins-로-위치-추정-재-보기)).
설치가 안 됐거나 CPU 를 아끼고 싶으면 끌 수 있다:

```bash
ros2 launch adr_bringup step1.launch.py laps:=2 vio:=false
```

rviz 의 하늘색 `VioPath` 가 VIO 궤적, 빨간 `FlownPath` 가 실제다. 특징점 추적 영상은
`ros2 run rqt_image_view rqt_image_view /adr/vio/debug_image` — **볼 때만** 그려진다.

### 맵 (코스) 바꾸기

**맵 하나 = yaml 파일 하나**, 저장 위치는 여기다:

```
adr_ws/src/adr_bringup/config/maps/
├── cross.yaml          # 십자 4게이트 원형 (기본값)
├── figure8.yaml        # 평면 8자, 게이트 6개
├── inverted_loop.yaml  # SkyDreamer 논문 Fig 6  — 게이트 3
└── big_track.yaml      # SkyDreamer 논문 Fig 9  — 게이트 12, 약 23×13 m
```

이 파일이 코스의 단일 진실 원천이고 `gate_planner`(궤적) · `gate_markers`(rviz) · `gate_pnp`(게이트 위치) ·
`gen_world.py`(gz 월드) 가 전부 여기서 읽는다.

**맵 이름 = gz 월드 이름 = `adr_sim/assets/worlds/<맵>.sdf`** 로 맞춰 놨기 때문에, 기존 launch 3개 모두
**`map:=<맵 이름>` 인자 하나**로 고른다 (확장자·경로 없이 이름만, 기본값 `cross`):

```bash
ros2 launch adr_sim sim.launch.py       map:=big_track   # T1 — gz 월드 + PX4 + rviz
ros2 launch adr_bringup step1.launch.py map:=big_track   # T2 — 계획 + 제어
ros2 launch adr_bringup real.launch.py  map:=big_track   # 실기체(mocap) 쪽 rviz
```

`map:=` 하나가 gz 월드 · `PX4_GZ_WORLD` · 게이트 마커 · min-snap 웨이포인트 · PnP 게이트 · 컨트롤러의
`start` · VIO 의 gz IMU 토픽 경로까지 전부 따라간다. 배치만 보고 싶으면 T1 만 띄우면 된다
(`ros2 launch adr_sim sim.launch.py map:=big_track` → gz + rviz 에 게이트가 그려진다).

> ⚠️ **T1 과 T2 의 `map` 이 다르면 에러 없이** 월드와 게이트 좌표만 조용히 어긋난다.
> 각 터미널의 `[map]` 로그 줄을 비교할 것.

궤적만 ROS 없이 그려 보려면:

```bash
cd adr_ws/src/adr_planning
python3 -m adr_planning.plot_trajectory --gates ../adr_bringup/config/maps/big_track.yaml --laps 1
```

#### 맵 목록과 비행 가능 여부

| 맵 | 게이트 | 내용 | 기본 파라미터로 비행 |
|---|---|---|---|
| `cross` | 4 | 십자 배치 원형 코스 (기본값) | ✅ 최대 틸트 36° |
| `figure8` | 6 | 전 게이트 높이 1.5 m, 좌우 루프와 중앙 X 교차 | SITL 미검증 |
| `inverted_loop` | 3 | 논문 Fig 6 / Table IV "Loop" | ⚠️ 틸트 53°, 바닥 0.33 m |
| `big_track` | 12 | 논문 Fig 9, 약 23×13 m 타원형 | ⚠️ 틸트 45.2° (한계 45°) |

⚠️ 표시된 두 맵도 **뒤집어야 하는 것은 아니다.** min-snap 은 논문의 split-S 를 재현하지 않고 그 경유점들을
매끄러운 곡선으로 지나가므로, 궤적 전 구간에서 필요한 추력 벡터가 위를 향한다(f_z > 0). 걸리는 것은
**기울기**다 — PX4 `MPC_TILTMAX_AIR`(기본 45°)를 넘으면 세트포인트를 못 따라가고, `inverted_loop` 맵은 4.05 m
가상 게이트에서 1.35 m 실제 게이트로 내려오는 구간에서 바닥에 가까워진다. 궤적을 덜 공격적으로 만들면 해결된다:

```bash
ros2 launch adr_bringup step1.launch.py map:=big_track     v_avg:=2.5 a_max:=6.0   # 틸트 35°, 40.5 s
ros2 launch adr_bringup step1.launch.py map:=inverted_loop v_avg:=2.0 a_max:=5.0   # 틸트 24°, 바닥 0.40 m
```

논문 트랙 두 맵은 속도를 낮춰야 한다. 변경된 `figure8`은 SITL 비행 검증이 필요하다. 기본값은
[`planner.yaml`](adr_ws/src/adr_bringup/config/planner.yaml) 에 있고 위처럼 launch 인자로 덮어쓴다.

자세한 내용은 [docs §4](docs/step1_architecture.md#4-코스-정의--맵-configmapsyaml).

#### 새 맵 추가

```bash
cp adr_ws/src/adr_bringup/config/maps/cross.yaml adr_ws/src/adr_bringup/config/maps/mymap.yaml
# gates / start 를 고친다
python3 adr_ws/src/adr_sim/scripts/gen_world.py --map mymap   # gz 월드 생성 (전부: --all)
colcon build --symlink-install --packages-select adr_bringup adr_sim
```

그 다음부터는 `map:=mymap` 만 붙이면 된다. 배경 장애물은 게이트 배치에서 자동으로 코스 밖에 둘러지므로
맵마다 배치할 일은 없고, 월드를 안 만들었거나 맵 yaml 보다 낡았으면 `sim.launch.py` 가 실행할 명령을
알려주며 멈춘다.

`sim.launch.py` 옵션: `ev:=false`(진실값 주입 대신 GPS 시뮬, airframe 4030 — 이때 T2 에 `origin_mode:=start` 필요) · `gui:=false`(headless) · `rviz:=false` · `soft_gl:=0`(GPU 있는 머신) · `agent:=micro-xrce-dds-agent`(snap 설치본) · `px4_dir:=...`

PX4 셸이 필요하면 daemon 으로 떠 있는 PX4 에 클라이언트로 붙는다: `~/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander check`, `px4-param set MPC_XY_VEL_MAX 5`.

## 자주 막히는 곳

| 증상                                            | 원인 / 조치                                                                                                                                                |
| ----------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `colcon build` 가 px4_msgs 에서 실패            | submodule 미초기화. `git submodule update --init --recursive`                                                                                              |
| 노드는 뜨는데 arm/offboard 로 안 넘어감         | `/fmu/out/*` 미수신. px4_msgs 와 PX4 버전이 맞는지 확인. 토픽 이름에 `_v4`/`_v1` 같은 접미사가 붙으므로 `ros2 topic list \| grep fmu` 로 실제 이름을 볼 것 |
| gz 가 `Ogre::UnimplementedException` 으로 abort | GPU 없는 VM. `sim.launch.py` 기본값 `soft_gl:=1` 이 llvmpipe 를 강제한다. GPU 있으면 `soft_gl:=0`                                                          |
| gz 가 `ign gazebo --force-version 6` 으로 뜸    | apt 의 Fortress 용 ros_gz 가 잡힘. Harmonic 워크스페이스를 먼저 source                                                                                     |
| `Unknown message type [9]`                      | 위와 동일 (브릿지가 Fortress 판)                                                                                                                           |
| PX4 가 `no autostart file found (…/4030_*)`     | `px4_dir` 가 잘못됐거나 `make px4_sitl` 미완료. launch 로그의 `[px4] … airframes→` 줄 확인                                                                 |
| `ov_msckf 패키지를 찾을 수 없다` 며 launch 가 멈춤 | OpenVINS 미설치. 설치 절의 `setup_openvins.sh` → `colcon build` → `source install/setup.bash`. 워크스페이스를 밀면 매번 다시 받아야 한다 |
| VIO 궤적·디버그 이미지가 아예 안 보임 | `vio:=false` 로 껐는지 확인. `ros2 node list \| grep -E 'vio_align\|ov_msckf\|drift_corrector'` |
| PX4 가 `waiting for gz world` 에서 60 s 후 종료 | gz 서버가 안 떴거나 월드 이름 불일치. T1 로그 앞부분의 gz 에러 확인                                                                                        |
| `맵 "..." 의 월드가 아직 생성되지 않았다` | 맵 yaml 만 만들고 월드를 안 만들었다. `gen_world.py --map <맵>` → `colcon build` |
| 게이트가 rviz 와 gz 에서 다른 곳에 있음 | T1 과 T2 의 `map:=` 이 다르다. 두 터미널의 `[map]` 로그 줄을 비교할 것 |

자세한 내용은 [docs/step1_architecture.md](docs/step1_architecture.md).
