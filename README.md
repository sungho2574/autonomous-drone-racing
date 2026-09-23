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
    ├── adr_bringup/              # 미션 launch(step1), 게이트 맵(gates.yaml), rviz 노드, mocap 브릿지
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

PX4 는 `make px4_sitl` 만 끝나 있으면 된다(airframe 은 실행 시 자동 주입, PX4 소스 수정·재빌드 불필요):

```bash
export PX4_DIR=~/PX4-Autopilot      # 클론 위치가 다르면 그 경로로. 기본값이면 생략 가능
(cd $PX4_DIR && make px4_sitl)
```

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

VIO(OpenVINS) 로 위치 추정을 같이 재 보려면 터미널 하나를 더 쓴다 (최초 1회 설치 필요 —
[docs §13](docs/step1_architecture.md#13-vio-openvins-로-위치-추정-재-보기)):

```bash
adr_ws/src/adr_vio/scripts/setup_openvins.sh && (cd adr_ws && colcon build --symlink-install --packages-select ov_core ov_init ov_msckf adr_vio)
ros2 launch adr_vio vio.launch.py
```

rviz 의 하늘색 `VioPath` 가 VIO 궤적, 빨간 `FlownPath` 가 실제다. 특징점 추적 영상은
`ros2 run rqt_image_view rqt_image_view /adr/vio/debug_image` — **볼 때만** 그려진다.

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
| PX4 가 `waiting for gz world` 에서 60 s 후 종료 | gz 서버가 안 떴거나 월드 이름 불일치. T1 로그 앞부분의 gz 에러 확인                                                                                        |

자세한 내용은 [docs/step1_architecture.md](docs/step1_architecture.md).
