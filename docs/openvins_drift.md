# OpenVINS SLAM 특징점과 반복 코스 드리프트

현재 사용 대상은 OpenVINS v2.7 (`setup_openvins.sh`). `ov_msckf`의 일반 SLAM 특징점은
EKF 안에 유지되는 장기 추적 랜드마크이며, 한 바퀴 전의 장소를 다시 찾아 pose graph를
보정하는 loop closure와 다르다. RViz의 `points_slam`이 보인다고 재방문 인식이 켜진 것은 아니다.

## 특징점 승격/제거

- 일반 KLT 특징점이 clone window보다 오래 관측되면 SLAM 후보가 된다.
- 초기화 뒤 `dt_slam_delay` 경과, `max_slam` 여유, 삼각측량 및 필터 검증이 필요하다.
- 일반 SLAM 특징점은 추적이 끊기거나 갱신을 반복해서 실패하면 marginalize된다.
  같은 장소로 돌아와 새로 검출한 점을 이전 점에 자동으로 재연결하는 전역 검색은 없다.
- `max_slam_in_update`는 순차 갱신의 **배치 크기**다. 25라서 한 프레임에 25개만
  사용한다는 뜻이 아니다. v2.7은 남은 특징점을 while 루프로 나누어 모두 갱신한다.
- `dt_slam_delay`는 시작 직후의 지연이며, 각 특징점의 승격 대기시간이 아니다.
  `max_clones`를 키우면 관측창은 길어지지만 승격에 필요한 트랙도 길어진다.

현재 설정: clones 15, SLAM 상한 50, 배치 25, 추적 특징점 300, tracking 상한 35 Hz.
카메라가 30 Hz라면 처리 상한을 35 Hz로 올려도 영상이 35장 생성되지는 않는다.

## 비교 가능한 SLAM 예산 프로필

`slam_dense`는 max_slam=100, max_slam_in_update=50을 ROS 파라미터로 덮어쓴다.
나머지 파라미터, 센서 캘리브레이션, 노이즈, 초기화 설정은 원본 YAML을 유지한다.
기본은 `default`이며 이 프로필의 비행 성능은 아직 검증되지 않았다.

```bash
# T1: 기존 sim 실행. T2는 아래 둘 중 하나씩 실행한다.
ros2 launch adr_bringup step1.launch.py map:=figure8 laps:=5 vio_profile:=default vio_timing_path:=/tmp/vio-default-run1.csv
ros2 launch adr_bringup step1.launch.py map:=figure8 laps:=5 vio_profile:=slam_dense vio_timing_path:=/tmp/vio-slam-dense-run1.csv
```

동일 맵, 비행속도, heading, 초기 정지 조건으로 비교한다. 이전 비행을 종료하고 새로 시작한다.
CSV는 upstream에서 덮어쓰기하므로 launch는 이미 존재하는 경로를 거부한다. 매 실행 새 이름을 준다.
독립 VIO launch는 `profile:=slam_dense timing_path:=...`를 쓴다.

확인할 것:

- `/ov_msckf/points_slam`: PointCloud2의 width×height로 현재 SLAM 점 수 확인.
  점 수가 상한 50보다 계속 훨씬 적으면 예산을 늘려도 해결되지 않는다. 추적 수명/삼각측량이 먼저다.
- `/adr/camera/image_raw`, `/adr/vio/odom`의 실제 stamp 주기와 출력 지연, timing CSV의 처리 시간.
  상태 수를 늘려 처리 지연/프레임 손실이 늘면 오히려 악화될 수 있다.
- `/adr/vio/error` 및 `vio_align` RMSE 로그로 raw VIO 비교. 그림의 진한 파랑은 PnP 보정 출력이므로
  raw VIO와 구분한다. 현재 오차 로거는 최신 기준 포즈와 비교하므로 정밀 평가는 bag에서 timestamp를
  맞춰 ATE/RPE와 랩별 위치·yaw 오차를 따로 계산해야 한다.

## 누적 오차를 되돌리는 구조

반복 주행을 활용하려면 키프레임/descriptor 저장 → 재방문 검색 → 기하 검증 → 위치·yaw 보정이 필요하다.
공식 `ov_secondary`가 이 역할의 예제지만 ROS1/catkin 코드이며 현재 ROS2 launch에는 연결되어 있지 않다.
또한 보정 결과를 별도 출력하며 OpenVINS EKF에 되먹임하지 않는다. 이를 연결해도 raw VIO 궤적은
계속 드리프트할 수 있다. 잘못된 loop closure를 거르면 성능 향상이 제한되고, 잘못 수용하면 맵을 망칠 수 있다.

게이트 위치가 알려진 이 프로젝트에서는 게이트 ID를 신뢰성 있게 연관시키고 재투영/시간 정합 검증을 거쳐
map↔VIO의 위치+heading을 보정하는 접근도 가능하다. 현재 DriftKF는 위치만 보정한다.
PnP 회전값을 그대로 덮어쓰거나 실제 궤적을 기준으로 강제 맞추는 것은 이 문제의 해결로 취급하지 않는다.

## 근거

- [v2.7 VioManager: 승격·marginalization·순차 갱신](https://github.com/rpng/open_vins/blob/v2.7/ov_msckf/src/core/VioManager.cpp)
- [v2.7 YAML/ROS 파라미터 우선순위](https://github.com/rpng/open_vins/blob/v2.7/ov_core/src/utils/opencv_yaml_parse.h)
- [공식 ov_secondary: 적용 범위, ROS1 설치, loop closure 한계](https://github.com/rpng/ov_secondary)
