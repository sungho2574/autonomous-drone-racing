# PnP 관측 게이트 ID 연결

주행의 다음 게이트와 카메라에 보이는 게이트를 분리한다. 모든 유효 검출을 맵의
가시 게이트와 비교하며, 선택된 관측 ID의 맵 좌표로 단일 게이트 PnP를 푼다.
경로/목표 순서는 바꾸지 않는다. MonoRace 문서 3-5의 prior matching을 참고한
구현으로, Adaptive Cropping, LSD, descriptor, affine RANSAC은 이번에 넣지 않았다.

## Prior와 시간

`/adr/state/corrected`의 위치와 자세(VIO 자세 유지)를 사용한다. 이미지 timestamp를
양쪽에서 감싸는 pose의 위치를 선형 보간하고 자세를 SLERP한다. 최대 0.25초 동안
pose를 기다리며, pose 간격 0.2초 초과/외삽/다른 frame_id는 허용하지 않는다.
prior가 없으면 해당 프레임은 보정하지 않는다. **따라서 기본 설정은 vio:=true가
필요하다.** simulator의 `/adr/odom`은 오차 평가/주행 순서 계측에만 사용한다.

카메라 extrinsic과 CameraInfo의 K/D로 개구부 코너를 투영한다. 카메라 뒤,
게이트 뒷면, 화면 밖 코너, 퇴화된 사각형은 제외한다. 투영 코너와 검출 코너의
시계방향 순환 이동을 비교해 물리 코너 순서를 보존한 뒤 PnP에 전달한다.

## 초기 임계값

`adr_perception/config/gate_pnp.yaml`에서 조정한다.

- 예상 사각형 평균 변 길이 × 0.15를 RMSE 상한으로 사용한다. 10–45 px로 제한.
- 가장 나쁜 코너의 오차도 RMSE 상한 × 1.5 이하여야 한다.
- 최선 RMSE ≤ 차선 RMSE × 0.7, 차선 − 최선 ≥ 5 px를 모두 만족해야 한다.
- 차선은 절대 임계값 탈락 여부와 무관하게 비교한다. 하나의 후보만 보이면 절대
  임계값으로 판단한다. 여러 검출이 같은 ID를 주장하면 그 ID는 사용하지 않는다.
- 유효 매칭이 여러 개면 RMSE/허용값이 가장 작은 하나로 PnP를 푼다.
- 기존 재투영 3 px 제한과 위치 차이 5 m 제한을 유지한다. 위치 차이는 이제
  실제 위치가 아닌 이미지 시각의 VIO prior를 기준으로 한다.

이 값들은 초기 튜닝값이다. prior 드리프트가 커서 매칭 범위를 벗어나면 보정이
끊길 수 있다. 이런 경우 자동으로 임계값을 풀거나 다음 게이트 ID를 강제하지 않는다.

## 진단 및 비행 후 검증

`/adr/pnp/association` DiagnosticArray는 매 검출 프레임의 이미지 timestamp로
선택 ID(0=없음), 최선/차선 ID 및 RMSE, 최대 코너 오차, 허용값, 점수 차이,
채택 여부와 판정 코드를 발행한다. 문자열 사유는 DiagnosticStatus.message에 있다.
이 토픽은 비행 bag에 저장되고 `/adr/vio/metrics`에도 최신 값이 집계된다.
리뷰 화면은 원본 프레임 진단을 우선 사용하여 한 프레임 기각도 보존한다.

| 코드 | 사유 |
|---|---|
| 0 | 채택 |
| 1 | prior 없음/지연/공백 |
| 2 | 유효 코너 없음 |
| 3 | 가시 게이트 후보 없음 |
| 4 | 절대 코너 오차 초과 |
| 5 | 최선/차선 모호 |
| 6 | 여러 검출이 동일 ID 주장 |
| 7 | PnP 재투영 실패 |
| 8 | PnP 위치가 prior에서 과하게 벗어남 |
| 9 | CameraInfo 없음 |

ROS 환경에서 `colcon build --packages-select adr_perception adr_vio adr_bringup` 후
환경을 다시 source하고 big_track을 VIO와 함께 실행한다. 비행 후 리뷰의
‘게이트 ID 매칭 오차/결과/판정’ 그래프에서 G3→G4 구간의 G6 채택 여부, 기각 비율,
PnP 위치 점프 및 보정 VIO 오차를 함께 비교한다. 실제 비행 개선 여부는 이 재비행으로
검증해야 한다. 합성 투영 테스트는 실제 big_track 맵에서 G4와 G6가 동시에 보일 때
G6 검출이 G6로 연결되고 원래 기체 위치로 복원되는지 검증한다.
