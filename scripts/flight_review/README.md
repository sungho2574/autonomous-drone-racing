# Flight review

`step1.launch.py` 실행 한 번을 비행 하나로 기록한다. 기본값은 `record:=true`이며
저장소 루트 `flight_logs/`는 Git에서 제외된다. 웹 분석은 ROS 없이도 실행할 수 있다.

## 비행 기록

ROS 환경에서 OpenVINS 계측 패치를 적용하고 변경한 패키지를 빌드한다.
새 의존성은 `rosdep`으로 설치한다. `setup_openvins.sh`는 기존 checkout에도 아직 적용하지 않은
패치를 적용한다. 패치 실패 시 중단하므로 실패 상태에서 빌드를 진행하지 않는다.

```bash
adr_ws/src/adr_vio/scripts/setup_openvins.sh
cd adr_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ov_core ov_init ov_msckf adr_msgs adr_vio adr_bringup
source install/setup.bash
ros2 launch adr_bringup step1.launch.py map:=figure8 laps:=3 vio_profile:=slam_dense
```

시뮬레이터는 기존처럼 별도 터미널에서 같은 맵으로 실행한다.
`record:=false`로 기록을 끌 수 있다. 설치본만 배포해 저장소 루트를 찾을 수 없으면
`record_root:=/absolute/path/to/repo/flight_logs`를 지정한다.

```text
flight_logs/<UTC시각>_<트랙>_<ID>/
  session.json                 # 비행 노드 시작 전에 atomic write + fsync
  ready.json                   # bag writer 초기화 후 준비 완료
  bag/*.db3                    # SQLite ROS 2 bag, CDR messages
  bag/metadata.yaml            # 정상적으로 writer를 닫으면 생성
  openvins_timing.csv           # OpenVINS 각 업데이트의 원본 timing
  review/gate-passages.png      # 웹에서 해당 기록 분석 시 생성
```

`session.json`에는 트랙, 랩 수, 배속, 계획/heading 설정, VIO 프로필과 유효 예산,
맵·보정·제어·카메라/IMU 설정 파일 원문, custom message 정의, Git revision/dirty 상태,
시작 시각을 남긴다. 상태 전이도 즉시 갱신한다. `DONE` 이후 약 1초의 후속 데이터를
기록하고 닫는다. `land_after:=false`이면 HOLD 중에도 종료할 때까지 계속 기록한다.

writer 준비 전에는 비행 파이프라인을 시작하지 않는다. SQLite는 `resilient` 프리셋과
메모리 캐시 0을 사용한다. Ctrl+C는 가능한 경우 bag를 마무리하고 `interrupted`로 기록한다.
강제 kill에서는 `recording`이 남을 수 있으므로 뷰어가 **진행 중/비정상 종료 미확정**으로
알린다. `metadata.yaml` 없이도 SQLite에 commit된 메시지를 직접 읽는다. 전원 손실 시
마지막 미반영 메시지까지 보장하는 것은 아니다. `.db3-wal` 파일이 있으면 `.db3`와 함께
전체 세션 폴더를 복사해야 한다. 진행 중 bag를 옮기기보다 recorder 종료 후 복사를 권장한다.

기존 step1 재시작 정리 로직에는 이전 `vio_metrics`/`flight_recorder`도 포함된다.
이때 오래된 기록은 비정상 종료 기록으로 남아도 읽을 수 있다.

## 분석 화면 실행

저장소 루트에서:

```bash
uv run --with-requirements scripts/flight_review/requirements.txt python scripts/flight_review/app.py
```

또는 별도 venv에 `pip install -r scripts/flight_review/requirements.txt` 후 실행한다.
`http://127.0.0.1:5050`에 접속한다. 다른 컴퓨터에서 가져온 세션도 폴더 전체를 복사한 뒤
`--root /path/to/flight_logs`로 열 수 있다. 포트는 `--port 5051`로 변경 가능하다.
uPlot 1.6.32, Three.js 0.160.1/OrbitControls와 라이선스를 vendor로 포함해 CDN 연결이 필요 없다.
코드 업데이트 후 실행 중인 Flask 서버를 재시작하고 브라우저를 새로고침한다.

- 접을 수 있는 사이드바에서 기록 선택·검색. 새 비행은 새로고침으로 표시.
- 우상단 **보기 옵션**에서 그래프만 / **3D + 그래프 이분할** 선택. 선택은 브라우저에 저장.
- 이분할 왼쪽은 스크롤 중에도 유지되는 3D 뷰어, 오른쪽은 통과점 이미지와 그래프.
- 궤적별 Plan / Actual / Corrected VIO / Raw VIO 체크박스로 선과 위치 마커를 함께 표시/숨김.
- 3D에서 드래그 회전, 우클릭 이동, 휠 확대. 전체 보기/위에서 보기 버튼 제공. WebGL 지원 필요.
- 그래프 호버 시각에 각 궤적의 구·기체 3축·지면(z=0) 점선 표시. 마우스를 옮겨도 마지막 시각 유지.
- 위치는 선형 보간, 자세는 quaternion의 shortest-arc SLERP. 0.3초 초과 pose 공백이나 기록 범위 밖은 마커를 숨긴다.
- 계획 궤적의 축은 기록된 계획 yaw만 반영하며 roll/pitch는 추정하지 않는다. TRACK 시각이 없는 계획은 선만 표시.
- 상단 4열 × ceil(게이트 수/4) **흰 배경** 정면 통과점 PNG. 복사 버튼과 PNG 다운로드.
- 모든 그래프의 시간 커서가 연동됨. 드래그 확대, 더블클릭 복원.
- 체크박스+이름 버튼으로 시리즈 표시/숨김. 호버 시 각 값 표시.
- 현재 기록도 읽을 수 있지만 자동 갱신하지 않는다. **기록 다시 읽기** 사용.
- 이미지 복사는 브라우저 Clipboard API 지원이 필요하다. 미지원 시 PNG 저장 사용.

## 수치 해석

`/adr/vio/metrics` (`diagnostic_msgs/DiagnosticArray`, 10 Hz)에 아래 값들을 모은다.
누락/오래된 값은 0으로 채우지 않고 생략한다. 마지막 수신 이후 시간은 따로 표시한다.

| 항목 | 의미 |
|---|---|
| `klt_features` | 추적기의 현재 ID 전체(새로 검출된 점 포함), stereo 중복 ID는 한 번만 계산 |
| `klt_observations` | 카메라별 관측 수 합계. 현재 mono 환경에서는 klt_features와 같음 |
| `tracker_active_features`, `tracker_is_klt` | 사용 중인 추적기의 전체 ID 수와 KLT 활성 여부 |
| `slam_features` | `/ov_msckf/points_slam` 점 수: 필터 상태에 유지 중인 SLAM landmarks |
| `msckf_update_features` | `/ov_msckf/points_msckf`: 최근 MSCKF 업데이트에서 시각화하는 특징점 |
| `loop_triangulated_features` | `/ov_msckf/loop_feats`: loop 연동용 삼각측량 점. loop closure 횟수가 아님 |
| `slam_capacity`, `slam_utilization_pct` | 설정된 max_slam과 사용률 |
| `*_hz`, `*_lag_s`, `*_age_s` | 헤더 시각 기준 주파수, ROS 시간 지연, wall time 기준 수신 경과 |
| `timing_*_ms` | OpenVINS tracking/propagation/MSCKF/SLAM/총 처리 시간 |
| `vio_sigma_*` | pose covariance 대각선의 제곱근: 위치 m, 회전 rad; OpenVINS 좌표계 |
| `*_speed_mps`, `*_yaw_deg`, `*_x/y/z_m` | 실제/보정/raw odometry 수치 |
| `drift_*_m`, `pnp_*`, `gate_*` | PnP 보정량, 품질·재투영 오차·거리·게이트 ID, 검출 수 |

KLT는 `adr_vio/patches/0002-tracker-metrics.patch`로 계측한다. 추적기의 thread-safe
`get_last_ids()`를 사용해 삼각측량/SLAM 선정 전의 현재 ID를 세며, 새로 검출한 점도 포함한다.
따라서 수가 유지돼도 같은 점이 오래 유지됐다고 보장하지는 않는다. SLAM update 채택/기각 수와
track 수명은 아직 계측하지 않는다.

패치는 `/ov_msckf/tracking_metrics` (`diagnostic_msgs/DiagnosticArray`)를 각 처리 프레임의
카메라 timestamp로 발행한다. `/adr/vio/metrics`에 최신값을 모으고 원본 토픽도 bag에 기록한다.
뷰어는 원본이 있으면 10 Hz 집계 대신 원본 프레임별 KLT 값을 사용해 짧은 추적점 감소를 보존한다.
새 패치로 OpenVINS를 재빌드한 이후의 비행부터 기록된다. 이전 bag에 KLT가 없으면 누락 안내를
표시하며 0으로 채우거나 loop triangulated 수치로 대체하지 않는다. `use_klt: false`이면 KLT
수치는 생략하고 `tracker_active_features`만 보고한다.

`slam_dense` 검증은 **SLAM 특징점/사용률 → 처리시간 → VIO 주파수·지연 → 위치 오차**를 같은
시각에서 함께 본다. 특징점이 늘어도 처리 시간이 카메라 주기를 넘고 지연이 쌓이거나
오차가 개선되지 않으면 예산 증가가 도움이 안 된 것이다. `default`와 같은 코스·속도·랩 수로
비행해 비교한다. 원본 timing CSV가 있으면 뷰어는 10 Hz 최신값 대신 모든 업데이트를 읽어
짧은 처리시간 피크를 보존한다. 그래프는 최대 약 5,000점으로 min/max 다운샘플하지만
원본 bag/CSV는 그대로 보존한다. 긴 공백은 끊어 그린다.

## 궤적과 게이트 통과점

- Plan: `/adr/trajectory`의 실제 발행 다항식. 현재 소스로 재계획하지 않는다.
- Actual: `/adr/odom` (현재 시뮬 환경의 기준 궤적; 실기체에서는 이 토픽의 측정원에 따라 정확도가 달라짐).
- PnP-corrected VIO: `/adr/state/corrected`.
- Raw VIO: `/adr/vio/odom`, 최초 map 정렬 후 PnP drift 보정 전. OpenVINS `global` 좌표 자체가 아니다.

게이트 법선의 음→양 방향 평면 교차를 보간한다. 게이트 정면에서 오른쪽은
`(sin(yaw), -cos(yaw), 0)`, 세로는 +z이다. 개구부 밖의 교차도 표시해 빗나감이 드러난다.
게이트 순서와 계획 통과 시각 주변 구간으로 다른 게이트의 무한 평면 교차를 걸러낸다.
`TRACK t_traj`와 기록 시각에서 시작 시간을 복원하므로 제어 상태 발행/수신 지연 정도의
시간 오차는 있다. 아주 큰 지연이나 되돌아가는 비행에서는 대응이 빠질 수 있다.
교차가 없으면 최근접점을 통과점으로 꾸며내지 않는다. 샘플 간 0.3초 초과 공백은 제외하며,
0.75 m 초과 위치 점프를 가로지르는 교차는 ×로 표시한다.

위치 오차 그래프는 실제 pose를 VIO header 시각에 보간해 비교한다. 0.2초 초과 기준 데이터
공백에는 외삽하지 않는다. 온라인 `/adr/vio/error`와 별도로 계산한다.

bag에는 위 metrics·세 pose·계획·제어 상태·OpenVINS pose/odom·PnP·게이트 검출·drift·IMU·camera_info·clock을 저장한다.
원본 카메라 영상은 저장하지 않으므로 이 bag만으로 OpenVINS를 처음부터 재실행할 수는 없다.
이 기록은 추정 결과 진단용이다.

## 검증

```bash
uv run --with-requirements scripts/flight_review/requirements.txt --with pytest pytest -q scripts/flight_review/tests
node --test scripts/flight_review/tests/test_pose.mjs
# 실제 비행과 분리된 합성 데이터 미리보기
uv run --with-requirements scripts/flight_review/requirements.txt python scripts/flight_review/tests/fixture_bag.py --root /tmp/adr-review-demo --gates 12
uv run --with-requirements scripts/flight_review/requirements.txt python scripts/flight_review/app.py --root /tmp/adr-review-demo
```

ROS 환경에서 첫 비행 시 `ros2 topic echo /ov_msckf/tracking_metrics --once`,
`ros2 topic echo /adr/vio/metrics --once`, `ros2 bag info <세션>/bag`,
`session.json`의 message_counts를 확인한다. VIO가 활성화된 정상 비행에서는 metrics와 세 pose,
trajectory에 데이터가 있어야 한다. 실제 ROS/DDS 및 rosbag2 writer 실행은 ROS 환경에서 확인해야 한다.

근거: [OpenVINS v2.7 ROS2Visualizer](https://github.com/rpng/open_vins/blob/v2.7/ov_msckf/src/ros/ROS2Visualizer.cpp),
[rosbag2 SQLite crash resilience](https://github.com/ros2/rosbag2/blob/rolling/rosbag2_storage_sqlite3/README.md).

### 궤적 오차 통계

오차 표에서 전체 비행(TRACK 구간) 또는 계획 랩을 선택합니다. Raw VIO와
PnP 보정 VIO는 actual과 비교하고, actual과 plan의 차이는 추종 오차로 구분합니다.
모든 값은 위치 단위 m이며 자세 오차를 포함하지 않습니다.

- APE(t) = 같은 시각의 두 위치 사이 3D 거리. 그래프에서 시간별로 확인합니다.
- ATE / RMSE = sqrt(mean(APE²)). 동일한 통계이므로 중복 열을 만들지 않습니다.
- Mean APE, P95, Max 및 X/Y/Z RMSE를 함께 표시합니다.
- 기존 map 좌표계에서 계산하며 추가 SE(3)/Sim(3) 정렬을 적용하지 않습니다.
  Raw VIO에는 시스템의 최초 map 정렬이 이미 적용되어 있습니다.
- TRACK 시작 기준 20 Hz 공통 평가 시각에 선형 보간합니다. 외삽 및 측정 간격
  0.2초 초과 구간은 제외하고, 유효 샘플 수/예정 샘플 수를 표시합니다.
  Max는 이 평가 시각들에서의 최대값이며 원본 모든 프레임의 최대값은 아닙니다.
- 첫 랩은 TRACK 시작부터 계획의 마지막 게이트 통과 시각까지, 다음 랩은
  직전 경계부터 다음 마지막 게이트 통과까지입니다. 반개구간으로 경계 중복을
  방지합니다. 마지막 랩 이후 복귀 구간은 전체 통계에만 포함됩니다.
  계획의 게이트 순서를 확인할 수 없으면 해당 랩부터 통계를 생략합니다.
- 미완료/미기록 랩은 해당 상태를 표시하고 누락 구간을 유효 비율에 반영합니다.
  랩 구간이 기록되었다는 표시는 실제 게이트 통과 성공 판정이 아닙니다.
  TRACK 시작 시각이 없는 기록은 시간 대응 통계를 계산하지 않습니다.

정의 참고: [evo — APE / ATE metrics](https://github.com/MichaelGrupp/evo/wiki/Metrics).
