# Step 1 기술 문서 — min-snap + PX4 Position Control

## 목차

1. [목적과 범위](#1-목적과-범위)
2. [시스템 구성](#2-시스템-구성)
3. [좌표계와 규약](#3-좌표계와-규약)
4. [코스 정의 (gates.yaml)](#4-코스-정의-gatesyaml)
5. [min-snap 궤적 생성](#5-min-snap-궤적-생성)
6. [PX4 offboard 제어](#6-px4-offboard-제어)
7. [시뮬 자산: 드론 모델·게이트·월드·airframe](#7-시뮬-자산-드론-모델게이트월드airframe)
8. [환경 설치 (Ubuntu VM)](#8-환경-설치-ubuntu-vm)
9. [실행 절차](#9-실행-절차)
10. [실기체(mocap) 전환](#10-실기체mocap-전환)
11. [검증 체크리스트](#11-검증-체크리스트)
12. [트러블슈팅](#12-트러블슈팅)
13. [다음 단계(step 2/3)와의 접점](#13-다음-단계step-23와의-접점)

---

## 1. 목적과 범위

step 1 은 인식 없이 **알려진 게이트 맵**으로 한 바퀴를 도는 것까지다. 목적은 파이프라인 뼈대(토픽·프레임·launch·시뮬 자산)를 세워 이후 단계에서 **노드만 교체**할 수 있게 하는 것.

| 요소 | step 1 구현 | 비고 |
|---|---|---|
| 게이트 인식 | 없음 (`adr_perception/gate_detector_stub` 가 빈 `GateArray` 발행) | step 3 Gatenet 이 같은 토픽/메시지로 교체 |
| 위치 추정 | sim: PX4 EKF2(GPS 시뮬) 또는 진실값 주입 / 실기체: mocap → `vehicle_visual_odometry` | 실기체 파라미터 세트 = airframe 4031 |
| 경로 계획 | `adr_planning` min-snap (7차 piecewise polynomial) | 게이트 중심 + 전후 접근점을 웨이포인트로 |
| 제어 | `adr_control/px4_position_controller` → `TrajectorySetpoint`(pos+vel+acc feedforward) | step 2 RL 노드가 `OffboardBase` 를 상속해 rate setpoint 로 교체 |

## 2. 시스템 구성

```mermaid
flowchart LR
  subgraph gz["Gazebo Harmonic (adr_cross.sdf)"]
    racer[adr_racer 모델<br/>IMU/mag/baro/navsat + camera + OdometryPublisher]
    gates[adr_gate x4]
  end
  subgraph px4["PX4 SITL (standalone, airframe 4030/4031)"]
    gzb[gz_bridge] --> ekf[EKF2] --> mpc[Position/Attitude/Rate ctrl] --> gzb
    uxrce[uxrce_dds_client]
  end
  agent[MicroXRCEAgent udp4:8888]
  subgraph ros["ROS 2 Humble"]
    bridge[ros_gz_bridge<br/>clock, camera, ground truth]
    planner[adr_planning<br/>gate_planner]
    ctrl[adr_control<br/>px4_position_controller]
    markers[adr_bringup<br/>gate_markers]
    tf[adr_bringup<br/>px4_odom_to_tf]
    stub[adr_perception<br/>gate_detector_stub]
    rviz[rviz2]
  end
  yaml[(gates.yaml)] --> planner
  yaml --> markers
  yaml -. gen_world.py .-> gz
  racer -- gz transport --> gzb
  racer -- /adr_racer/camera --> bridge --> stub
  uxrce <--> agent <--> ros
  planner -- /adr/trajectory<br/>/adr/planned_path --> ctrl
  ctrl -- /fmu/in/offboard_control_mode<br/>/fmu/in/trajectory_setpoint<br/>/fmu/in/vehicle_command --> agent
  agent -- /fmu/out/vehicle_local_position<br/>/fmu/out/vehicle_status<br/>/fmu/out/vehicle_odometry --> ctrl
  agent --> tf --> rviz
  markers --> rviz
  planner --> rviz
```

### 토픽 계약

| 토픽 | 타입 | 발행 | 구독 | QoS |
|---|---|---|---|---|
| `/adr/trajectory` | `adr_interfaces/PolynomialTrajectory` | gate_planner | px4_position_controller | reliable, **transient_local** |
| `/adr/planned_path` | `nav_msgs/Path` | gate_planner | rviz | transient_local |
| `/adr/gates` | `adr_interfaces/GateArray` | gate_markers (맵) | (step 2 RL 관측) | transient_local |
| `/adr/detected_gates` | `adr_interfaces/GateArray` | gate_detector (stub) | (step 3 PnP/맵 정합) | default |
| `/adr/gate_markers` | `visualization_msgs/MarkerArray` | gate_markers | rviz | transient_local |
| `/adr/flown_path` | `nav_msgs/Path` | gate_markers (TF 누적) | rviz | default |
| `/adr/odom` + TF `map→base_link` | `nav_msgs/Odometry` | px4_odom_to_tf | rviz, gate_markers | default |
| `/adr/camera/image_raw`, `camera_info` | `sensor_msgs/Image`, `CameraInfo` | ros_gz_bridge (sim) / camera_relay (실기체) | gate_detector | best effort |
| `/adr/ground_truth/odom` | `nav_msgs/Odometry` | ros_gz_bridge | 평가 스크립트 | default |
| `/adr/controller_state` | `std_msgs/String` | px4_position_controller | 디버그 | default |
| `/fmu/in/*` | px4_msgs | adr_control, mocap_bridge | PX4 | best effort |
| `/fmu/out/*` | px4_msgs | PX4 | adr_control, px4_odom_to_tf | **best effort + volatile** (reliable 로 구독하면 안 옴) |

## 3. 좌표계와 규약

| | ROS 측 (`map`, 궤적, 게이트, mocap) | PX4 측 (`/fmu/*`) |
|---|---|---|
| 월드 | **ENU** (x 동, y 북, z 위) | **NED** (x 북, y 동, z 아래) |
| 기체 | FLU (x 앞, y 왼, z 위) | FRD (x 앞, y 오른, z 아래) |
| yaw | +x(East) 기준 **CCW** | +x(North) 기준 CW(위에서 봤을 때) |

- 변환은 전부 `adr_control/frames.py` 한 곳에서: `(e,n,u) ↔ (n,e,−u)`, `yaw_ned = π/2 − yaw_enu`, 쿼터니언은 `q_ENU→NED=(0,√½,√½,0)`, `q_FLU→FRD=(0,1,0,0)` (PX4 gz_bridge 와 동일 상수). `test/test_frames.py` 가 왕복·기수방향을 검증한다.
- 모든 노드의 public 인터페이스는 ENU 이고 **PX4 경계(publish/subscribe 콜백)에서만** NED 로 바꾼다.
- 게이트 `yaw_deg` = 통과 방향(개구부 법선)의 방위각, ENU CCW. ⚠️ crazyflie 레포 `gates.yaml` 은 CW 관례였으므로 값을 복사해 오면 안 된다.
- `map` 원점 = PX4 local origin = 기체 전원 인가(EKF 초기화) 위치. 실기체에서는 mocap 원점을 그대로 쓰므로(EKF2 EV 만 사용) mocap 원점 = `map` 원점이 된다. sim 에서 GPS 모드(4030)일 때는 스폰 위치가 local origin 이 되므로 `gates.yaml` 의 `start` 를 스폰 위치와 같게 둔다 (`gen_world.py` 가 자동으로 그렇게 배치).

## 4. 코스 정의 (gates.yaml)

[`adr_bringup/config/gates.yaml`](../adr_ws/src/adr_bringup/config/gates.yaml) 이 단일 진실 원천이다.

```
          G2 (0, 4) yaw 180
             ┌─┐
   G3 ─┐     │ │     ┌─ G1 (4, 0) yaw 90
 (-4,0)│     └─┘     │
  yaw  │      ↻ CCW  │
  270  └─   ┌─┐     ─┘
             │ │  ● start (2.83, -2.83)
             └─┘
          G4 (0,-4) yaw 0
```

- 반지름 4 m 원 위에 90° 간격, 게이트 중심 높이 1.5 m, 법선 = 원의 접선(CCW 진행).
- 게이트: 내부 1.5 m, 외부 2.1 m(프레임 폭 0.3 m), 두께 0.1 m, 주황색.
- `start` = 이륙 지점(바닥). G4→G1 사이 원호 위에 두어 이륙 후 첫 진입이 자연스럽다.
- 값을 바꾸면 **`adr_sim/scripts/gen_world.py`** 를 다시 돌려 월드를 갱신하고 커밋한다(planner/markers 는 yaml 을 직접 읽으므로 자동 반영).

## 5. min-snap 궤적 생성

`adr_planning/min_snap.py` (numpy 만 사용, ROS 비의존 → Mac 에서 `pytest` 가능)

- 세그먼트마다 7차 다항식, 비용 = ∫ snap² dt (Mellinger & Kumar 2011).
- 등식 제약: 웨이포인트 통과, 시작/끝 vel·acc·jerk = 0, 세그먼트 경계에서 1~4차 미분 연속.
- 풀이: 세그먼트별 정규화 시간 τ=t/T∈[0,1] 로 조건수를 낮춘 뒤, 제약을 SVD nullspace 로 소거한 무제약 QP 를 폐형식으로 푼다(`solve_1d`). x, y, z, yaw 를 독립으로 4번 푼다. 25 세그먼트 기준 ~70 ms.
- 시간 할당: 세그먼트 길이 / `v_avg`, 출발·정지 세그먼트에 `v_avg/a_max` 추가. 샘플링으로 `v_max`, `a_max` 초과 시 전체 시간을 균일 스케일(최대 5회).
- yaw 는 `np.unwrap` 으로 연속화해 풀고, 컨트롤러가 PX4 로 보낼 때 `[-π, π]` 로 감싼다.
- 웨이포인트(`course.py`): `[start 호버점] + ([게이트−d·n, 게이트 중심, 게이트+d·n] × 4게이트 × laps) + [start 호버점]`. `approach_dist` d=0.8 m 의 전후 접근점이 게이트를 **면에 수직으로** 통과하게 만든다(중심 통과 시 속도·법선 cos > 0.97).
- 오프라인 확인: `python3 -m adr_planning.plot_trajectory --gates adr_ws/src/adr_bringup/config/gates.yaml --laps 2`

기본 파라미터(`planner.yaml`: v_avg 3, v_max 6, a_max 8, laps 2)에서 2바퀴 17.7 s, 최고 3.8 m/s.

메시지 `PolynomialTrajectory` 는 세그먼트별 `duration` 과 축별 계수(오름차순, 실제 시간 τ∈[0,duration] 기준)를 담는다. 컨트롤러는 이를 `Trajectory.from_plain` 으로 복원해 50 Hz 로 샘플링한다.

## 6. PX4 offboard 제어

### `OffboardBase` (`adr_control/offboard_base.py`)

컨트롤러 종류와 무관한 부분: `/fmu/out/vehicle_local_position`, `/fmu/out/vehicle_status` 구독(best-effort), `offboard_control_mode` / `trajectory_setpoint` / `vehicle_command` 발행, `arm()`, `set_offboard_mode()`(DO_SET_MODE 176, param1=1, param2=6), `land()`, ENU 입력 → NED 변환. step 2 의 RL 노드는 이를 상속해 `step()` 만 구현하고 `VehicleRatesSetpoint` 를 발행하면 된다.

### `PositionController` 상태기계

```
WAIT_TRAJ ─(궤적+위치+상태 수신)→ WARMUP ─(세트포인트 20회)→ ARMING ─(offboard & armed)→ TAKEOFF
  ─(호버점 도달)→ TRACK ─(t ≥ T)→ HOLD ─(hold_time, land_after)→ LAND ─(disarm)→ DONE
```

- PX4 는 offboard 진입 전에 세트포인트 스트림(≥2 Hz)이 이미 흐르고 있어야 하므로 WARMUP 에서 현재 위치 hold 를 먼저 보낸다.
- TAKEOFF/HOLD 는 `position` 만, TRACK 은 `position+velocity+acceleration`(feedforward) 을 `OffboardControlMode` 에 켠다. 첫 비행에서 튀면 `controller.yaml` 의 `feedforward: false`, launch 인자 `time_scale:=0.5` 로 낮춘다.
- PX4 파라미터 상 `MPC_XY_VEL_MAX`, `MPC_ACC_HOR` 등이 궤적 속도보다 작으면 추종이 늦어진다(airframe 4030 에서 10 m/s, 8 m/s² 로 완화).

## 7. 시뮬 자산: 드론 모델·게이트·월드·airframe

### 드론 `adr_racer` — 250급 레이싱 쿼드

스펙은 [`adr_sim/config/racer_spec.yaml`](../adr_ws/src/adr_sim/config/racer_spec.yaml) 한 곳에 두고 `scripts/gen_racer_model.py` 가 **`model.sdf` 와 PX4 airframe `4030_gz_adr_racer` 를 함께 생성**한다(로터 위치·최대 회전수처럼 양쪽이 일치해야 하는 값을 한 번에 관리).

| 항목 | 값 | 근거 |
|---|---|---|
| 질량 | 0.75 kg (배터리 포함) | 5" 프리스타일/레이싱 AUW 650~850 g |
| 모터 대각 | 250 mm → 암 반경 0.125 m, 로터 (±0.088, ±0.088) | x-config |
| 관성 | Ixx=Iyy 2.5e-3, Izz 4.5e-3 kg·m² | 강체 근사 |
| 프롭 | 5" (x500 의 13.45" 메시를 `<scale>0.372</scale>` 로 축소) | 메시는 `meshes/` 에 복사(BSD-3) |
| 모터 모델 | `maxRotVelocity 3000 rad/s`, `motorConstant 1.4e-6` → 모터당 12.6 N, TWR ≈ 6.8 | 2306 2400KV @ 4S 부하 시 ~28k rpm |
| 카메라 | 640×480, HFOV 90°, 30 Hz, 30° 상향 틸트, gz 토픽 `adr_racer/camera` | 레이싱 카메라 각도 |
| 외관 | 박스 placeholder | STL 준비되면 `meshes/README.md` 절차로 교체 |

PX4 gz_bridge 규약 때문에 **링크 `base_link`, 센서 이름 `imu_sensor / magnetometer_sensor / air_pressure_sensor / navsat_sensor`, 모터 명령 토픽 `/<model>/command/motor_speed`** 는 x500 과 같아야 한다(`scripts/px4_sensors.sdf.inc` 에서 그대로 삽입).

airframe `4030_gz_adr_racer`: `4001_gz_x500` 기반. `CA_ROTORn_PX/PY = ±0.0884`(FRD), `SIM_GZ_EC_MIN/MAX = 300/3000`(= `maxRotVelocity`), `MPC_THR_HOVER 0.31`(ω_hover=√(mg/4k_f)=1146 rad/s 를 [300,3000] 에 선형 매핑), 레이싱용 `MPC_XY_VEL_MAX 10`, `MPC_ACC_HOR 8`, `MPC_JERK_AUTO 20`, `MPC_TILTMAX_AIR 60`. MC 자세/각속도 게인은 `rc.mc_defaults` 그대로이므로 **첫 호버에서 진동하면 `MC_ROLLRATE_P/PITCHRATE_P` 를 0.1 → 0.05 부터 낮춘다.**

airframe `4031_gz_adr_racer_ev`: 4030 + `EKF2_EV_CTRL 15`, `EKF2_HGT_REF 3`, `EKF2_GPS_CTRL 0`, `EKF2_BARO_CTRL 0`. 외부 위치(sim: OdometryPublisher 진실값, 실기체: mocap)만으로 EKF2 를 돌리는 세트.

### 게이트 `adr_gate`

box 링크 4개(좌·우·상·하)로 된 static 모델. 원점 = 개구부 중심, +x = 통과 방향. 색 `1.0 0.45 0.0`.

### 월드 `adr_cross.sdf`

`gen_world.py` 가 `gates.yaml` 에서 생성. PX4 가 gz 를 직접 띄우지 않는 **standalone 모드**이므로 PX4 `server.config` 가 넣어주던 시스템 플러그인(Physics, UserCommands, SceneBroadcaster, Contact, Imu, AirPressure, Magnetometer, NavSat, Sensors(ogre2))을 월드에 직접 포함하고, navsat 용 `<spherical_coordinates>` 도 넣는다. 게이트 4개와 `adr_racer` 를 `<include>` 로 배치(드론 이름 `adr_racer` = `PX4_GZ_MODEL_NAME`).

## 8. 환경 설치 (Ubuntu VM)

```bash
# 1) ROS 2 Humble + Gazebo Harmonic + ros_gz (non-default pairing; ros-humble-ros-gz* 와 동시 설치 금지)
sudo apt install ros-humble-desktop ros-dev-tools
sudo curl https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update && sudo apt install gz-harmonic ros-humble-ros-gzharmonic

# 2) PX4-Autopilot v1.16 (레포 밖, ~/PX4-Autopilot). ubuntu.sh 가 gz-harmonic 을 설치한다.
git clone -b release/1.16 --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh
cd ~/PX4-Autopilot && make px4_sitl        # 최초 빌드

# 3) Micro-XRCE-DDS-Agent (레포 밖)
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent && mkdir build && cd build && cmake .. && make -j$(nproc) && sudo make install && sudo ldconfig /usr/local/lib/

# 4) 이 레포 (메시는 git LFS)
sudo apt install git-lfs && git lfs install
git clone --recursive https://github.com/sungho2574/autonomous-drone-racing.git
cd autonomous-drone-racing/adr_ws
rosdep install --from-paths src --ignore-src -r -y     # motion_capture_tracking 의존성 포함
colcon build --symlink-install
source install/setup.bash

# 5) PX4 에 airframe 등록 후 재빌드 (최초 1회, 스펙 변경 시 재실행)
PX4_DIR=~/PX4-Autopilot src/adr_sim/scripts/install_px4_assets.sh
cd ~/PX4-Autopilot && make px4_sitl
```

- `install_px4_assets.sh` 는 `adr_sim/px4/airframes/*` 를 `ROMFS/px4fmu_common/init.d-posix/airframes/` 에 심볼릭 링크하고 그 디렉터리의 `CMakeLists.txt` `px4_add_romfs_files(...)` 에 항목을 넣는다(멱등).
- px4_msgs 는 PX4 버전과 메시지 정의가 맞아야 한다: PX4 `release/1.16` ↔ px4_msgs `release/1.16` (submodule 로 고정).

## 9. 실행 절차

```bash
# T1 — gz 월드 + ros_gz_bridge + gate_markers + odom→TF + rviz   (gui:=false 로 headless)
ros2 launch adr_bringup sim.launch.py

# T2 — DDS 에이전트
MicroXRCEAgent udp4 -p 8888

# T3 — PX4 SITL: 실행 중인 월드의 adr_racer 에 attach (PX4_DIR 기본 ~/PX4-Autopilot)
adr_ws/src/adr_sim/scripts/run_px4_sitl.sh          # airframe 4030 (GPS 시뮬)
adr_ws/src/adr_sim/scripts/run_px4_sitl.sh --ev     # airframe 4031 (진실값 = mocap 에뮬레이션)
#   pxh> 프롬프트에서 commander takeoff 로 호버 확인 가능

# T4 — step1 파이프라인
ros2 launch adr_bringup step1.launch.py laps:=2 time_scale:=1.0 land_after:=true
```

`run_px4_sitl.sh` 가 설정하는 환경변수: `PX4_GZ_STANDALONE=1 PX4_GZ_MODEL_NAME=adr_racer PX4_GZ_WORLD=adr_cross PX4_SYS_AUTOSTART=4030|4031`. gz 는 이미 떠 있어야 하며 월드 이름(`<world name="adr_cross">`)과 모델 이름이 일치해야 한다.

유용한 확인 명령:

```bash
ros2 topic echo /adr/controller_state
ros2 topic hz /fmu/out/vehicle_local_position
gz topic -l | grep adr_racer
ros2 run adr_planning plot_trajectory --gates adr_ws/src/adr_bringup/config/gates.yaml --laps 2
```

## 10. 실기체(mocap) 전환

1. 기체 PX4 파라미터에 airframe 4031 의 EKF2 세트를 적용(`EKF2_EV_CTRL 15`, `EKF2_HGT_REF 3`, `EKF2_GPS_CTRL 0`, `EKF2_BARO_CTRL 0`, `EKF2_EV_DELAY` 는 mocap 지연 실측값). uXRCE-DDS 클라이언트를 시리얼/UDP 로 설정하고 `MicroXRCEAgent` 를 그에 맞게 실행.
2. mocap rigid body 이름을 `adr_racer` 로 두고(또는 `rigid_body:=`), rigid body 의 x 축이 기체 앞을 보도록 정의한다(yaw 정렬). mocap 좌표계는 ENU(z-up).
3. `motion_capture_tracking` 실행 (crazyflie 레포의 `motion_capture.yaml` 과 같은 방식, `/poses` 발행).
4. `ros2 launch adr_bringup real.launch.py` → `mocap_bridge` 가 `/fmu/in/vehicle_visual_odometry` 로 주입. QGC/`ros2 topic echo /fmu/out/vehicle_local_position` 에서 위치가 mocap 과 일치하는지, 기체를 손으로 움직여 방향이 맞는지 확인.
5. 게이트를 `gates.yaml` 좌표(mocap 원점 기준)에 배치하고 이륙 지점 `start` 에 기체를 둔다(`WAIT_TRAJ` 에서 현재 위치를 hold 로 잡으므로 정확할 필요는 없지만 첫 세그먼트가 짧을수록 좋다).
6. `ros2 launch adr_bringup step1.launch.py use_sim_time:=false time_scale:=0.5 laps:=1` 로 저속 1바퀴부터.

sim 에서는 `--ev` 로 같은 EKF2 경로를 미리 검증할 수 있다(모델의 `OdometryPublisher` 진실값이 PX4 gz_bridge 를 통해 같은 uORB `vehicle_visual_odometry` 로 들어간다). 이때 ROS `mocap_bridge` 를 같이 띄우면 인스턴스가 2개가 되므로 띄우지 않는다.

## 11. 검증 체크리스트

macOS(작성 머신):
- [ ] `python -m pytest adr_ws/src/adr_planning` 와 `python -m pytest adr_ws/src/adr_control` (numpy, pyyaml, pytest)
- [ ] `xmllint --noout adr_ws/src/adr_sim/models/*/model.sdf adr_ws/src/adr_sim/worlds/*.sdf`
- [ ] `gen_world.py` / `gen_racer_model.py` 재실행 결과가 커밋본과 동일

Ubuntu VM:
- [ ] `colcon build --symlink-install` 무오류, `ros2 pkg list | grep adr_`
- [ ] `sim.launch.py`: 게이트 4개·드론 표시, `ros2 topic hz /adr/camera/image_raw` ≈ 30 Hz, rviz 에 게이트 마커
- [ ] PX4 SITL 이 `adr_racer` 에 붙어 `commander takeoff` 로 호버 (진동 없음)
- [ ] `step1.launch.py`: 자동 arm → 이륙 → 4게이트 × 2바퀴 → hold → 착륙. rviz 에서 planned(초록) vs flown(빨강) 경로 겹침, 게이트 통과 시 오차 < 0.3 m
- [ ] `--ev` 로 GPS 없이 동일 비행

## 12. 트러블슈팅

| 증상 | 원인/대응 |
|---|---|
| `/fmu/out/*` 가 아무것도 안 옴 | 구독 QoS 가 reliable. `PX4_SUB_QOS`(best effort, volatile) 사용. `MicroXRCEAgent` 가 떠 있는지, `ROS_DOMAIN_ID` 일치 확인 |
| PX4 가 `gz_bridge` 에서 멈춤 / 센서 없음 | 월드에 Imu/Magnetometer/AirPressure/NavSat/Sensors 시스템 플러그인이 없음(standalone). `adr_cross.sdf` 가 생성본인지 확인 |
| `PX4_GZ_MODEL_NAME` 모델을 못 찾음 | 월드의 `<include><name>adr_racer</name>` 와 이름 불일치, 또는 `PX4_GZ_WORLD` ≠ `<world name>` |
| offboard 전환 거부 (`REJECT OFFBOARD`) | 세트포인트 스트림이 없거나 2 Hz 미만. WARMUP 이 돌고 있는지, `/fmu/in/offboard_control_mode` 가 50 Hz 인지 확인 |
| arm 거부 | EKF 위치 미수렴(`xy_valid`), 또는 preflight 실패. `pxh> commander check` |
| 호버에서 진동/뒤집힘 | 추력/관성 대비 게인 과다. `MC_ROLLRATE_P`, `MC_PITCHRATE_P` 를 낮추거나 `motorConstant`/`MPC_THR_HOVER` 재확인 |
| 궤적 추종이 늦고 안쪽으로 잘림 | `MPC_XY_VEL_MAX`/`MPC_ACC_HOR` 한계. `time_scale` 을 낮추거나 planner `v_max`/`a_max` 를 줄임 |
| yaw 가 90° 틀어짐 | ENU↔NED yaw 변환 누락. `frames.yaw_enu_to_ned` 를 통했는지, 실기체는 rigid body x 축 정의 확인 |
| 프롭 메시가 안 보임 | `GZ_SIM_RESOURCE_PATH` 에 `adr_sim/models` 가 없음. `source install/setup.bash`(env hook) 또는 `scripts/setup_env.sh` |
| `ros-humble-ros-gzharmonic` 설치 충돌 | 기존 `ros-humble-ros-gz*`(Fortress) 제거 후 설치 |

## 13. 다음 단계(step 2/3)와의 접점

- **step 2 (RL + rate controller)**: `adr_control` 에 `OffboardBase` 상속 노드 추가. 관측 = `/adr/gates`(맵) + `/fmu/out/vehicle_odometry`, 출력 = `/fmu/in/vehicle_rates_setpoint` + `OffboardControlMode(body_rate=True)`. `step1.launch.py` 의 컨트롤러 노드만 교체. 학습 환경은 별도(예: gz 병렬 or 경량 시뮬)이며 airframe/모델 스펙(`racer_spec.yaml`)을 공유.
- **step 3 (Gatenet + OpenVINS)**: `adr_perception` 이 `/adr/camera/image_raw` → `/adr/detected_gates`(카메라 프레임 `GateArray`), 새 `adr_localization`(OpenVINS + 게이트 맵 PnP) 이 `map→base_link` 를 제공하고 mocap 대신 `vehicle_visual_odometry` 를 채운다. 인터페이스(`GateArray`, `frames.py`, `mocap_bridge` 의 주입 경로)는 그대로 재사용.
