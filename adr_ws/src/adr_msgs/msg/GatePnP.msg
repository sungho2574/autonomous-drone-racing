# 게이트 개구부 4 꼭짓점 PnP 로 얻은 **기체 위치** 1건. drift 보정 KF 의 측정값.
#
# 논문(KAIST A2RL 2025) §2.2 규약: PnP 의 회전 추정은 코너 노이즈에 취약하므로 **버리고**
# 위치만 쓴다. 회전은 VIO 쪽이 더 신뢰도가 높다 → 병진 drift 만 보정한다.
std_msgs/Header header      # frame_id = map, stamp = 원본 이미지 stamp
int32 gate_id               # 매칭된 맵 게이트 id (gates.yaml 의 id)
geometry_msgs/Point pos_w   # map 기준 기체 위치 p^W
float32 quality             # 0~1, 클수록 신뢰. KF 의 R 스케일(λ_r)에 쓴다
float32 reproj_px           # 재투영 오차 [px] — quality 산출 근거
float32 distance_m          # 추정 위치 ↔ 게이트 중심 거리 [m] — 거리 필터·quality 근거
