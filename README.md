# autonomous-drone-racing

전방 카메라로 게이트를 인식해 통과하는 자율 드론 레이싱 검증용 ROS 2 워크스페이스.
시뮬(PX4 SITL + Gazebo Harmonic)에서 단계별로 검증한 뒤 250급 레이싱 기체 + 모션캡처로 실기체 검증한다.

| | step 1 (현재) | step 2 | step 3 |
|---|---|---|---|
| 게이트 인식 | (x) 맵 사용 | (x) | Gatenet |
| 위치 추정 | motion capture | motion capture | OpenVINS + gate map PnP |
| 경로 계획 및 제어 | min-snap + PX4 Position Control | RL + rate controller | RL + rate controller |

## 구성

```
autonomous-drone-racing/
├── docs/step1_architecture.md    # 기술 문서 (설계·좌표계·설치·실행·트러블슈팅)
└── adr_ws/src/
    ├── adr_interfaces/           # msg: PolynomialTrajectory, GateArray
    ├── adr_sim/                  # gz 모델(게이트·레이싱 드론)·월드·PX4 airframe·실행 스크립트
    ├── adr_video/                # 카메라 입력 (sim: ros_gz_bridge / 실기체: 드라이버 relay)
    ├── adr_perception/           # 게이트 인식 (step1: stub)
    ├── adr_planning/             # min-snap 궤적 생성
    ├── adr_control/              # PX4 offboard 제어 (step1: position control)
    ├── adr_bringup/              # 통합 launch, 게이트 맵(gates.yaml), rviz, mocap 브릿지
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

PX4 쪽 준비(airframe 등록 후 재빌드, 최초 1회):

```bash
export PX4_DIR=~/PX4-Autopilot      # 클론 위치가 다르면 그 경로로
$PX4_DIR/../autonomous-drone-racing/adr_ws/src/adr_sim/scripts/install_px4_assets.sh
(cd $PX4_DIR && make px4_sitl)
```

## 실행 (시뮬, 터미널 4개)

모든 터미널에서 먼저:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_gz_harmonic_ws/install/setup.bash          # apt 의 Fortress 판보다 먼저
source adr_ws/install/setup.bash
source adr_ws/src/adr_sim/scripts/setup_env.sh          # PX4_DIR / gz 리소스 경로 / GL 설정
```

```bash
# T1 — gz 월드 + 브릿지 + rviz
ros2 launch adr_bringup sim.launch.py
```

```bash
# T2 — DDS 에이전트
MicroXRCEAgent udp4 -p 8888
```

```bash
# T3 — PX4 SITL (adr_racer 모델에 attach). --ev 면 GPS 대신 진실값(mocap 에뮬레이션)
adr_ws/src/adr_sim/scripts/run_px4_sitl.sh
```

```bash
# T4 — min-snap 계획 + position control (자동 arm → 이륙 → 2바퀴 → 착륙)
ros2 launch adr_bringup step1.launch.py laps:=2 time_scale:=1.0
```

## 자주 막히는 곳

| 증상 | 원인 / 조치 |
|---|---|
| `colcon build` 가 px4_msgs 에서 실패 | submodule 미초기화. `git submodule update --init --recursive` |
| 노드는 뜨는데 arm/offboard 로 안 넘어감 | `/fmu/out/*` 미수신. px4_msgs 와 PX4 버전이 맞는지 확인. 토픽 이름에 `_v4`/`_v1` 같은 접미사가 붙으므로 `ros2 topic list \| grep fmu` 로 실제 이름을 볼 것 |
| gz 가 `Ogre::UnimplementedException` 으로 abort | GPU 없는 VM. `setup_env.sh` 가 `ADR_SOFT_GL=1` 로 llvmpipe 를 강제한다 |
| gz 가 `ign gazebo --force-version 6` 으로 뜸 | apt 의 Fortress 용 ros_gz 가 잡힘. Harmonic 워크스페이스를 먼저 source |
| `Unknown message type [9]` | 위와 동일 (브릿지가 Fortress 판) |

자세한 내용은 [docs/step1_architecture.md](docs/step1_architecture.md).
