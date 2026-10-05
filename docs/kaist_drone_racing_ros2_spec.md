# Monocular Drone Racing — ROS2 Implementation Spec

> 출처 논문: *Drift-Corrected Monocular VIO and Perception-Aware Planning for Autonomous Drone Racing* (Azhari et al., KAIST, A2RL×DCL 2025, arXiv:2512.20475v1)
>
> 이 문서는 위 논문의 시스템을 **ROS2 (Humble/Jazzy) + Python/C++** 로 재구현하기 위한 구현 명세다. 센서 세팅은 **단안 RGB 카메라 + 저품질 IMU 만** 주어지는 상황을 가정한다 (mono RGB + raw IMU).
>
> Claude Code 사용 지침: 각 모듈은 독립 ROS2 노드로 구현한다. `TODO(impl)` 태그가 붙은 곳이 실제 작성 대상이다. 수식·임계값은 논문 값을 기본값으로 넣되 전부 `ros2 param` 으로 노출한다.

---

## 0. 전체 아키텍처

파이프라인은 4개 모듈로 구성된다. 데이터 흐름:

```
[Camera] --image--> Perception ┬-> YOLOv8-Pose ─gate keypoints─┐
[IMU]    --imu----> Perception ┴-> VIO (OpenVINS) ─odom─┐      │
                                                        ▼      ▼
                                              State Estimation
                                  (Gate Association → PnP → Filtering → KF drift correction)
                                                        │ drift-corrected state
                                                        ▼
   [Gate Map (prior)] ──────────────> Planning (TOGT + perception-aware heading)
                                                        │ reference trajectory
                                                        ▼
                                              Control (MPC → PID/FC)
                                                        │ thrust + body-rate
                                                        ▼
                                                  Flight Controller
```

핵심 아이디어 3가지:
1. **VIO-centric**: OpenVINS 를 backbone 으로 쓰고, gate 검출을 global anchor 로 삼아 **translational drift 만** 보정한다 (회전은 VIO 를 신뢰).
2. **Kalman Filter 는 full state 가 아니라 drift state `[p_d, v_d]` 만 추정**한다.
3. **Perception-aware planning**: TOGT time-optimal 궤적 위에 heading 을 따로 얹어서, 다음 gate 가 항상 FOV 안에 들어오게 만든다.

### ROS2 패키지 레이아웃

```
drone_racing_ws/src/
├── dr_msgs/                 # 커스텀 메시지/서비스 정의
├── dr_perception/          # VIO 래핑 + YOLOv8-Pose gate detector
├── dr_state_estimation/    # gate association, PnP, filtering, KF drift correction
├── dr_planning/            # TOGT + perception-aware heading planner (offline pre-compute)
├── dr_control/             # MPC (acados) + low-level 인터페이스
├── dr_bringup/             # launch, params, tf static, gate map
└── dr_sim/                 # (선택) 시뮬레이터 브릿지
```

### 좌표계 규약

| Frame | 설명 |
|-------|------|
| `world` (W) | gate map 이 정의된 고정 관성 프레임 |
| `body` (B)  | 드론 body (FLU: x-forward, y-left, z-up 권장) |
| `cam` (C)   | 카메라 optical frame |

- `T_BC` (body→cam extrinsics) 는 **Kalibr** 로 캘리브레이션 후 static TF 로 발행.
- 논문 카메라 장착각: qualification 15°, final 45° pitch. → extrinsics 로 반영.

---

## 1. Perception 모듈 (`dr_perception`)

원시 image + IMU 를 (a) VIO odometry, (b) gate keypoint 검출로 변환.

### 1.1 카메라 / IMU 인터페이스

논문 세팅 (기본값, param 화):

| 항목 | 값 |
|------|-----|
| 카메라 센서 | Sony IMX219, rolling shutter, wide-angle |
| FOV | 155°(H) × 115°(V) × 175°(D) |
| 스트림 | 1640×1232 @ 30FPS → **downscale ×2 → 820×626 @ 30FPS** |
| 노출 | 고정 exposure/gain (auto 끔 — motion blur 및 아티팩트 억제) |
| IMU rate | 250Hz (평가시 500Hz) |
| control rate | 200Hz |

**타임스탬프 동기화가 VIO 성능의 핵심.** 논문 방식:
- **카메라**: 표준 API 대신 **커스텀 GStreamer 파이프라인**을 ROS 노드에 넣어 프레임 도착 시각이 아니라 *하드웨어 acquisition timestamp* 를 캡처.
- **IMU**: FC↔onboard clock drift 때문에 FC 타임스탬프 대신 **Jetson 도착 시각** 사용 (jitter 있지만 drift 없음).
- 카메라-IMU 간 constant time offset 은 **Kalibr** 로 추정.

```
TODO(impl): dr_perception/camera_node
  - GStreamer appsink 로 프레임 + PTS(하드웨어 timestamp) 획득
  - sensor_msgs/Image + 정확한 header.stamp 발행 (/cam/image_raw)
TODO(impl): dr_perception/imu_relay_node
  - raw IMU 수신 → 도착시각으로 stamp 재기입 → /imu/data 발행
  - 카메라-IMU time offset 파라미터 적용
```

### 1.2 VIO backbone (OpenVINS)

논문은 TII-RATM 데이터셋으로 6종 VIO(ROVIO, LARVIO, OpenVINS, SVO, VINS-Mono, DM-VIO)를 벤치마크 후 **OpenVINS 선택**. 선정 기준(중요도 순): 정확도 > 강건성 > 처리시간 > CPU > 메모리. 하드웨어 임계값: **≤33ms 처리시간, ≤200% CPU, ≤500MB 메모리**.

OpenVINS 선택 이유: 초기화 강건성. persistent *"SLAM features"* 덕분에 정지 상태에서도 drift-away 하지 않음 (LARVIO 의 ZUPT 보다 안정적).

구현: OpenVINS 를 **그대로 서브모듈로 래핑**한다. 재구현하지 말 것.

```
TODO(impl): dr_perception/vio_node (OpenVINS 래퍼)
  in : /cam/image_raw, /imu/data
  out: /vio/odometry (nav_msgs/Odometry) — pose + twist @ ~30Hz
       p_v (position), q_v (orientation), v_v (linear vel), ω_v (angular vel)
  주의: VIO 는 초기화에 motion 필요 → race 전 수동 excitation 절차
```

VIO 초기화가 drifted heading 을 만들 수 있음 → §2.5 initial heading alignment 에서 1회 보정.

### 1.3 Gate 검출 — YOLOv8-Pose

2-stage(segmentation→corner) 대신 **single-pass YOLOv8-Pose** 로 bbox + 내부 corner keypoint 동시 예측. 라벨링 부담이 segmentation mask 보다 훨씬 적음.

**네트워크 출력 (per gate)** — 21차원:

```
[ O, cx, cy, w, h, tl(x,y,v), tr(x,y,v), br(x,y,v), bl(x,y,v) ] ∈ R^21
```
- `O`: gate class
- `cx,cy,w,h ∈ [0,1]`: 정규화된 bbox center/size
- `tl,tr,br,bl`: top-left / top-right / bottom-right / bottom-left corner, 각 `(x,y)` 정규화 좌표 + visibility flag `v`

**모델/배포 스펙:**

| 항목 | 값 |
|------|-----|
| 채택 모델 | **YOLOv8s @ 640×640** |
| 학습 데이터 | 수동 라벨 2000장 (다양한 트랙/환경) + gate 없는 이미지(FP 억제) |
| init | COCO pretrained weights |
| loss | class + bbox reg + keypoint reg + keypoint confidence |
| augmentation | color(대비/밝기), motion blur, Gaussian blur, MixUp, Mosaic |
| 배포 | **TensorRT FP16** 변환, onboard 추론 ~16.1ms (62Hz) |
| params | 11.6M |

**모델 선택 근거 (Table 3, 640px):**

| Model | Bbox mAP | Kpts mAP | t_infer | t_total |
|-------|----------|----------|---------|---------|
| YOLOv8n | 0.865 | 0.959 | 7.0ms | 12.6ms |
| **YOLOv8s** | **0.877** | **0.971** | 10.5ms | **16.1ms** |
| YOLOv8m | 0.885 | 0.972 | 19.7ms | 25.2ms |

→ YOLOv8m 은 25.2ms 로 33ms 한계에 근접(여유 부족). YOLOv8s 가 정확도/속도 최적 trade-off.

```
TODO(impl): dr_perception/gate_detector_node
  in : /cam/image_raw
  out: /gates/detections (dr_msgs/GateDetectionArray)
  - 이미지 640×640 resize → TensorRT FP16 추론
  - NMS 후 각 검출을 21-dim 라벨로 파싱 → corner 4개 pixel 좌표 복원
  - visibility flag 낮은 corner 는 downstream 에서 제외 가능하도록 그대로 전달
```

---

## 2. State Estimation 모듈 (`dr_state_estimation`)

VIO odometry + gate 검출을 융합해 **drift-corrected state** 생성. 순서: association → PnP → filtering → KF → (초기 1회) heading alignment.

### 2.1 Gate Association

검출된 gate 를 알려진 gate map 과 매칭. 현재 추정 pose 로 3D gate map 을 image plane 에 투영해서 검출과 대응.

- 검출 집합 `S = {S_0,...,S_{M-1}}`, 각 `S_i = [p^I_{i,0..K-1}]`, `K=4` corner 의 rectified pixel 좌표.
- gate map `G = {G_0,...,G_{N-1}}`, 각 `G_j = [p^W_{j,0..K-1}]`, corner 의 3D world 좌표.

**3D corner → image 투영:**
```
p^C_{j,k} = R^{WC} (p^W_{j,k} − p^W_C)          # world→cam
p^I_{j,k} = (1/⌊p^C_{j,k}⌋_z) · K_intr · p^C_{j,k}
```
`K_intr = [[fx,0,cx],[0,fy,cy]]`, `⌊·⌋_z` 는 z 성분.

**Association cost (reprojection error):**
```
e_repj(G_j, S_i) = Σ_{k=0}^{K-1} || p^I_{j,k} − p^I_{i,k} ||²
```

**Hungarian algorithm** 으로 전체 최적 할당:
```
min Σ_{(i,j)} e_repj(G_j, S_i)
```

```
TODO(impl): dr_state_estimation/association
  - VIO pose 로 gate map projection
  - cost matrix 구성 → scipy.optimize.linear_sum_assignment (또는 C++ Hungarian)
  - 매칭된 (G_j, S_i) pair 만 다음 단계로
```

### 2.2 Gate-based Pose Estimation (PnP)

매칭된 각 `(G_j, S_i)` 에 대해 **PnP** 로 카메라 pose 추정. 3D-2D 대응: map gate 의 known 3D corner ↔ 검출된 2D corner.

- `cv2.solvePnP(..., flags=SOLVEPNP_IPPE_SQUARE)` 사용 — planar square gate 에 최적화.
```
(p^W_{C,Si}, R^{WC}_{C,Si}) := SolvePnP(G_j, S_i)
```

**중요:** 노이즈 있는 corner 검출 때문에 **회전 추정 `R^{WC}_{C,Si}` 는 버리고 위치 `p^W_{C,Si}` 만** KF measurement 로 사용 (회전은 VIO 가 더 신뢰도 높음). → translational drift 만 보정.

```
TODO(impl): dr_state_estimation/pnp
  - 각 매칭 pair 마다 SOLVEPNP_IPPE_SQUARE
  - position p^W_{C,Si} 만 추출, rotation 폐기
```

### 2.3 Detection Filtering

KF 에 넣기 전 신뢰 못할 검출 제거. 3종 필터 순차 적용.

**(a) Distance Filtering** — 너무 가까우면 perspective 왜곡, 너무 멀면 저해상도.
```
d_i = || p^W_{C,Si} − p^W_{G_j} ||     # 추정 카메라 위치 ↔ gate 중심
discard if d_i < τ_{d,min}  or  d_i > τ_{d,max}
τ_{d,min} = 1 m,  τ_{d,max} = 13 m
```

**(b) Aspect Ratio Filtering** — gate 가 심하게 기울면 corner 정확도 저하.
```
a_i = max(w/h, h/w)        # bbox skew ratio
discard if a_i > τ_ratio
τ_ratio = 2
```

**(c) Occlusion Filtering** — 가까운 gate 가 먼 gate 를 가려 오검출 유발. `S_i` 가 `S_j` 에 가려졌다고 판단하는 조건 (둘 다 만족):
```
visual proximity : Δp(S_i, S_j) < τ_bbox        # bbox 간 최소 pixel 거리
relative size    : A(S_j)/A(S_i) > τ_area       # S_j 가 훨씬 큼 = 더 가까움
τ_bbox = 20 px,  τ_area = 1.2
```
→ 둘 다 만족 시 `S_i` discard. (double-gate 같은 의도적 근접 배치는 relative size≈1 이라 걸러지지 않음.)

```
TODO(impl): dr_state_estimation/filtering
  - 위 3필터 순차 적용, 통과한 검출만 KF update 로
```

### 2.4 Drift Correction — Kalman Filter

고빈도 drift-prone VIO 를 저빈도 globally-accurate gate 위치로 보정. **drift state 만** 추정.

**State:** `x_d = [p_d^T, v_d^T]^T ∈ R^6` — `p_d` 위치 drift, `v_d` 속도 drift.

**Prediction (constant-velocity drift model):**
```
x⁻_{d,k+1} = F x⁺_{d,k}
P⁻_{k+1}   = F P⁺_k F^T + Q

F = [[ I₃  Δt·I₃ ],      Q = [[ σ_p·I₃    0    ],
     [ 0₃    I₃  ]]            [   0     σ_v·I₃ ]]

σ_p = 0.1,  σ_v = 0.2
```

**Update:** 유효한 gate PnP 위치 `z_k = p^W_{Si}` 도착 시.
```
measurement 관계: z_k = p_{v,k} + p⁻_{d,k}     # p_{v,k}: VIO 위치
residual        : y_k = z_k − p_{v,k}
H = [ I₃  0₃ ]

K_k     = P⁻_k H^T (H P⁻_k H^T + R)⁻¹
x⁺_{d,k} = x⁻_{d,k} + K_k (y_k − H x⁻_{d,k})
P⁺_k     = (I − K_k H) P⁻_k

R = λ_r · diag(σ_rx, σ_ry, σ_rz) ∈ R^{3×3}
```
- `R`: PnP 위치 추정 불확실성. **λ_r > 0** 는 검출 품질 기반 tuning factor.
- 불확실성 추정법: 하나의 gate 검출을 50회 perturb → pose 추정 분포로 σ 산출.

**최종 state 복원:** VIO 출력에 위치 drift 만 더함.
```
p_c = p_v + p_d
X_{c,k} = [ (p_v + p_d)^T,  q_v^T,  v_v^T,  ω_v^T ]^T
```
position 만 보정, orientation/velocity/angular-vel 는 VIO 그대로.

```
TODO(impl): dr_state_estimation/kf_node
  in : /vio/odometry, /gates/pnp_positions (filtered)
  out: /state/corrected (nav_msgs/Odometry) — drift-corrected full state
  - 6-state KF, VIO 프레임마다 predict, gate 도착시 update
  - λ_r 를 검출 품질(거리/skew)로 스케일하는 로직 포함
```

### 2.5 Initial Heading Alignment (1회)

VIO 초기화가 부정확한 heading 을 만들 수 있음 → race 시작 전 **gate 기반 orientation 으로 heading 1회 보정**. 시작 podium 에서 첫 gate 가 항상 잘 보이므로 신뢰 가능. (논문상 B/C 시퀀스 ATE_r 개선의 주요 원인.)

```
TODO(impl): dr_state_estimation/heading_align (one-shot)
  - race arm 직후 첫 gate 검출로 world heading offset 산출 → VIO yaw 보정
```

---

## 3. Planning 모듈 (`dr_planning`)

Race **전에 offline 으로** reference trajectory 생성 (실시간 재계획 아님). TOGT time-optimal 궤적 + perception-aware heading.

### 3.1 Trajectory Planning — TOGT

waypoint-following 을 넘어 **TOGT (Time-Optimal Gate-Traversing) planner** 사용. gate 를 single point 가 아니라 **공간 volume `G_i`** 로 모델링해 flyable area 전체를 활용하는 time-minimal 궤적을 찾음.

**최적화 문제:**
```
min_{p, T}  T
s.t.  p(0) = p_start,  p(T) = p_finish
      ∃ 0 < t_1 < t_2 < ... < t_L < T
      such that  p(t_i) ∈ G_i,   i = 1..L
```
- gate 내부에 waypoint 를 두고 path 를 세그먼트화 → change-of-variable 로 unconstrained 문제로 변환 → gradient 기반(L-BFGS)으로 효율적 해결.
- CPC 대비 계산이 훨씬 빨라 on-site 반복 튜닝에 유리.

**perception 을 위한 튜닝 파라미터** (TOGT 가 성급/지연 heading 을 만들어 gate 를 놓치는 것 방지):
- gate 의 크기/volume → flyable area 제한
- gate 의 방향/tilt → gate 가시성 유지하는 path 유도
- body-rate limit

```
TODO(impl): dr_planning/togt (offline)
  참고: 논문 [34] Qin et al. TOGT planner 구현체 활용
  in : gate map (위치/자세/크기), start/finish, dynamic limits
  out: p(t), v(t), a(t), body-rate ref (시간 파라미터화 궤적)
```

### 3.2 Perception-aware Heading Planning

궤적 생성 **후** heading(yaw) 을 **decouple** 해서 얹음. 핵심: 지연된 turn 으로 visual lock 을 잃는 것을 막는 **anticipatory heading**. (인간 파일럿이 이전 gate 통과 즉시 다음 gate 로 시선 고정하는 것에서 착안.)

현재 gate heading `ψ_{g,i}` 와 다음 gate heading `ψ_{g,i+1}` 을 거리 기반 weight `λ_i` 로 블렌딩:
```
        ⎧ 1                          if d_i < d_min
λ_i  =  ⎨ 0                          if d_i > d_max
        ⎩ (d_max − d_i)/(d_max − d_min)   otherwise
```
`d_i`: 현재 gate 까지 거리.

블렌딩된 gate heading `ψ_g` 를 궤적 forward-direction `ψ_c` 와 weight `λ_g` 로 결합해 최종 desired heading:
```
ψ_g   = (λ_i · ψ_{g,i} + λ_{i+1} · ψ_{g,i+1}) / (λ_i + λ_{i+1})
ψ_des = λ_g · ψ_g + (1 − λ_g) · ψ_c
```

**성능 (Table 5):** perception-aware 로 gate visibility 개선.

| Visibility @ FOV | TOGT baseline | Ours |
|------------------|---------------|------|
| @ 820×626px, 155°×115° | 71.36% | **80.24%** |
| @ 614×470px, 120°×90° | 51.82% | **60.19%** |

visibility 정의 (모든 corner 가 이미지 경계 안):
```
vis = (|u_{i,k} − u_0| ≤ W/2) ∧ (|v_{i,k} − v_0| ≤ H/2)
```

### 현재 step1 구현: `adr_planning/perception_heading.py`

`heading_mode:=perception_aware`가 기본이다. TOGT 도입 전 단계로, 현재 min-snap 위치 경로 위에
§3.2 heading을 offline으로 얹는다. `ψ_{g,i}`는 맵의 게이트 법선이 아니라 **예측 기체 위치에서
게이트 중심을 바라보는 방위각**으로 해석한다. 가속도로 추정한 기체 자세와 카메라 장착 각도도 반영한다.

- 현재/다음 게이트의 거리 가중치와 전진 방향을 최단 각도 차이로 혼합한다. 두 거리 가중치가
  모두 0이면 현재 목표 게이트 방향을 사용한다. 게이트 중심 통과 예정 시각에 다음 대상으로 바꾼다.
- 원시 목표 yaw는 게이트 중심 방향 ±10° 안으로 제한한다. 비인과 평활화(기본 0.3 s)로
  전환을 미리 시작하고 C2 연속 cubic yaw를 만든다. 평활화 이후의 시야 조건은 **soft objective**이며
  하드 보장이 아니다. 급격한 표적 전환과 수직 게이트는 중앙 정렬이 불가능할 수 있다.
- yaw 속도/가속도 상한은 기본 0(비활성)이다. 기존 위치 경로와 계획 시간을 그대로 유지한다.
  극값은 진단용으로 보고하며, 사용자가 상한을 양수로 설정할 때만 전체 시간 스케일을 적용한다.
- 기존 `PolynomialTrajectory`의 `cyaw`로 전송하므로 제어기는 같은 경로로 yaw/yaw-rate를 받는다.
  재생 배율 `time_scale`에는 v·yaw-rate를 배율만큼, a를 배율의 제곱만큼 적용한다.
  `time_scale>1`은 계획의 속도 제한을 다시 초과할 수 있으므로 제한 준수 시 1 이하를 사용한다.
- 계획 로그의 가시율은 기체 추력 방향으로 예측한 자세, 카메라 외부 파라미터와 수평/수직 화각을
  사용한 **개구부 네 꼭짓점의 투영 결과**다. 반환/호버 구간은 제외하며 가림은 모델링하지 않는다.
  실제 SITL 영상 검증과는 구분한다. yaw만으로 수직 FOV, 통과 직전 화면 잘림을 해결할 수 없다.
- 이 구현은 §3.3의 가상 게이트 삽입이나 TOGT를 추가하지 않는다. 기존 맵의 급반전은 yaw 평활화로
  처리한다(각속도/각가속도 제한은 기본 비활성). heading은 계획 시간 기준이며 실시간 게이트 추적기는 아니다.

설정: `adr_bringup/config/planner.yaml`의 `heading_*`. 카메라 설정은 `racer_spec.yaml` 또는
실기체 캘리브레이션과 일치시켜야 한다. 기존 yaw와 비교하려면 `heading_mode:=gate_normal`.

### 3.3 Split-S 특수 처리

split-s 는 급격한 **180° heading 반전** → 표준 planner 는 순간적 flip 을 명령해 컨트롤러 saturate. 2단 해법:
1. 두 물리 gate 사이에 **intermediate virtual gate** 추가 → 드론을 바깥으로 밀어 안전한 통로 확보.
2. heading 을 고정 step 으로 점진 회전:
```
ψ_{k+1} = ψ_k ± Δψ_step      # ± 는 turn 방향
```

```
TODO(impl): dr_planning/perception_aware_heading (offline post-process)
  - TOGT 궤적에 §3.2 heading 오버레이
  - split-s 구간 감지 → virtual gate 삽입 + incremental yaw
  out: /plan/reference (dr_msgs/ReferenceTrajectory) — pos/vel/att/rate + yaw
```

---

## 4. Control 모듈 (`dr_control`)

2-레벨 구조: **high-level MPC** (궤적 추종) → **low-level PID on FC** (자세 안정화).

### 4.1 드론 모델 (rigid body, 6-DOF)

state: 위치 `p∈R³`, 속도 `v∈R³`, 자세 `q∈SO(3)`, body angular vel `ω∈R³`.
control input: collective thrust `f∈R¹`, desired angular velocity `τ∈R³`.

**연속시간 동역학:**
```
ṗ = v
v̇ = (1/m) R(q) [0,0,f]^T − g
ω̇ = J⁻¹ (−ω × Jω + τ)
q̇ = ½ q ⊗ [0, ω]^T
```
`m`: 질량, `g`: 중력 벡터, `R(q)`: q 대응 회전행렬, `J = diag(Jxx,Jyy,Jzz)`: 관성, `⊗`: quaternion 곱.

### 4.2 Model Predictive Control (MPC)

qualification 은 differential-flatness 컨트롤러로 충분했으나 final 은 **nonlinear MPC** 로 전환 (aggressive 궤적 추종 우수, thrust/angular-vel constraint 명시적 처리).

**Cost:**
```
min_{u_0:N-1} Σ_{k=0}^{N-1} [ ||p_k − p_ref,k||²_Qp + ||v_k − v_ref,k||²_Qv
                            + ||q_k ⊖ q_ref,k||²_Qq + ||ω_k − ω_ref,k||²_Qω
                            + ||u_k||²_Ru ]
```
`u_k = [f, τ] ∈ R⁴`. `Qp,Qv,Qq,Qω,Ru` 는 tracking/입력 weight. `⊖` 는 quaternion error.

**구현 스펙:**

| 항목 | 값 |
|------|-----|
| 프레임워크 | **acados** + **qpOASES** solver |
| prediction horizon | 1 초 |
| discretization | **N = 20** |
| 출력 | optimal thrust + angular velocity → FC @ **200Hz** |

```
TODO(impl): dr_control/mpc_node
  in : /state/corrected, /plan/reference
  out: /control/ctbr (collective thrust + body-rate) @ 200Hz
  - acados OCP: §4.1 동역학 discretize, §4.2 cost, thrust/rate constraint
  - solver: qpOASES, RTI scheme 권장 (실시간)
```

### 4.3 Low-level 인터페이스

MPC 의 thrust + angular-velocity command 를 FC 의 onboard **PID** 가 실행 (fast·reliable attitude control). high-level MPC 는 궤적 최적화에만 집중.

```
TODO(impl): dr_control/fc_bridge
  - /control/ctbr → FC 프로토콜 변환 (MSP / MAVLink 등 대상 플랫폼 맞춤)
  - 논문 HW: Betaflight 4.4.0, UART+MSP. 대상 환경에 맞게 교체
```

---

## 5. 메시지 인터페이스 (`dr_msgs`)

```
# GateDetection.msg
int32   gate_class
float32 conf
float32[8] corners_px      # tl,tr,br,bl 의 (x,y) pixel
float32[4] corner_vis      # visibility flag
float32[4] bbox            # cx,cy,w,h (px)

# GateDetectionArray.msg
std_msgs/Header header
GateDetection[] detections

# GatePnP.msg
std_msgs/Header header
int32   gate_id            # 매칭된 map gate index
geometry_msgs/Point pos_w  # p^W_{C,Si} (position only, rotation 폐기)
float32 quality            # λ_r 산출용 검출 품질

# ReferenceTrajectory.msg
std_msgs/Header header
geometry_msgs/Point[]      pos
geometry_msgs/Vector3[]    vel
geometry_msgs/Quaternion[] att   # yaw = ψ_des 반영
geometry_msgs/Vector3[]    rate
float32[]                  time_from_start
```

### 토픽 그래프

| Topic | Type | Pub → Sub |
|-------|------|-----------|
| `/cam/image_raw` | sensor_msgs/Image | camera → vio, detector |
| `/imu/data` | sensor_msgs/Imu | imu_relay → vio |
| `/vio/odometry` | nav_msgs/Odometry | vio → association, kf |
| `/gates/detections` | dr_msgs/GateDetectionArray | detector → association |
| `/gates/pnp` | dr_msgs/GatePnP[] | pnp+filter → kf |
| `/state/corrected` | nav_msgs/Odometry | kf → mpc |
| `/plan/reference` | dr_msgs/ReferenceTrajectory | planner → mpc |
| `/control/ctbr` | (thrust+rate) | mpc → fc_bridge |

---

## 6. 파라미터 요약 (기본값 = 논문 값)

```yaml
perception:
  image:       { width: 820, height: 626, fps: 30 }
  yolo:        { model: yolov8s, infer_size: 640, precision: fp16 }
  cam_imu_time_offset: 0.0        # Kalibr 로 추정
state_estimation:
  filtering:
    tau_d_min: 1.0                # m
    tau_d_max: 13.0               # m
    tau_ratio: 2.0                # bbox skew
    tau_bbox:  20.0               # px
    tau_area:  1.2
  kalman:
    sigma_p:   0.1                # position drift process noise
    sigma_v:   0.2                # velocity drift process noise
    lambda_r:  1.0                # 검출 품질 기반 스케일
planning:
  heading:
    d_min:     <tune>             # λ_i lower
    d_max:     <tune>             # λ_i upper
    lambda_g:  <tune>             # gate vs forward blend
  split_s:
    dpsi_step: <tune>             # incremental yaw step
control:
  mpc:
    horizon_s: 1.0
    N:         20
    Qp: [..], Qv: [..], Qq: [..], Qw: [..], Ru: [..]
    rate_hz:   200
    solver:    qpOASES
```

---

## 7. 의존성 / 빌드

| 모듈 | 핵심 의존성 |
|------|-------------|
| VIO | `OpenVINS` (submodule) |
| Gate 검출 | `ultralytics` (YOLOv8), `TensorRT` FP16, `torch` |
| PnP/association | `OpenCV` (SOLVEPNP_IPPE_SQUARE), `scipy`(Hungarian) |
| Planning | TOGT planner ([34]), L-BFGS |
| MPC | `acados` + `qpOASES` |
| 캘리브레이션 | `Kalibr` (intrinsics/extrinsics + cam-imu time offset) |
| 평가/GT | `MAPLAB` (batch optimization ground-truth) — 개발용 |

```
TODO(impl): dr_bringup/launch/race.launch.py
  - camera_node, imu_relay, vio_node, gate_detector_node,
    state_estimation 파이프라인, mpc_node, fc_bridge 순차 기동
  - gate map YAML 로드, static TF(T_BC) 발행
```

---

## 8. 구현 순서 (권장 로드맵)

1. **인터페이스 뼈대**: `dr_msgs` + 토픽 배선 + 시뮬/rosbag 재생으로 end-to-end 더미 통과.
2. **Perception**: OpenVINS 래핑 → `/vio/odometry` 확인. 별도로 YOLOv8-Pose 학습/변환 → `/gates/detections`.
3. **State Estimation**: association → PnP → filtering → KF. rosbag 으로 drift 보정 효과 검증 (ATE 감소 확인).
4. **Planning**: gate map 으로 TOGT 궤적 생성 → perception-aware heading 오버레이 → visibility 지표 확인.
5. **Control**: acados MPC 로 `/plan/reference` 추종 → FC bridge 연결.
6. **통합 튜닝**: exposure↔detection trade-off, λ_r, MPC weight, split-s step 반복 조정.

### 개발 원칙 (논문 §III)
> rapid development · ease of deployment · iterative improvement. 각 비행을 데이터 수집 기회로 삼아 gate dataset 증강, state-estimation·planner·controller 를 점진 개선. 검증 안 된 config 는 실전 투입 전 반드시 테스트 (논문에서 14m/s 미검증 config 로 final 2번째 슬롯에서 12번 gate 충돌한 사례 있음).

---

## 9. 알려진 한계 / 주의 (논문 §VI)

- **Detection ↔ VIO trade-off**: motion blur 줄이려 노출을 낮추면 이미지가 어두워져 gate 검출 성능 저하. on-site dataset 증강 + 튜닝 필수.
- **드문 gate 구간 (perceptual desert)**: drag race 처럼 gate 간격이 멀면 오래 "blind" 비행 → drift 누적. 대응: 빠르게 날아 blind 시간 최소화 (단 model/control 정확도 요구 ↑).
- **Pre-computed 궤적의 경직성**: 실시간 재계획 불가. tracking/state 오류 회복 불가, 상대 드론에 전략적 대응 불가 (multi-drone).
- **모델 미스매치**: MPC 가 rigid-body 모델만 사용, 고속 공력효과(drag, ground effect, downwash) 미반영. crash 후 동역학 변화도 미대응. → adaptive/learning-based control 이 향후 방향.
- **회전 신뢰**: PnP 회전은 노이즈 커서 폐기, VIO 회전만 사용한다는 설계 결정을 반드시 지킬 것.
