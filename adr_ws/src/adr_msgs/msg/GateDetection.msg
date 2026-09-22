# 이미지 평면에서 검출된 게이트 1개 (2D, 픽셀 단위). 색상 기반 검출기(step1~2) 출력.
# 3D 포즈(PnP 이후)는 Gate/GateArray 를 쓴다.
int32 id            # 프레임 내 순번 (면적 내림차순, 0 = 가장 큼/가까움)
float32 u           # 개구부(구멍) 중심 x [px]. 구멍이 안 보이면 bbox 중심
float32 v           # 개구부 중심 y [px]
float32 x           # bbox 좌상단 x [px]
float32 y           # bbox 좌상단 y [px]
float32 w           # bbox 폭 [px]
float32 h           # bbox 높이 [px]
float32 area        # 주황색 프레임 픽셀 면적 [px^2]
bool has_hole       # 개구부(내부 컨투어)가 보였는지 — true 면 통과 가능한 정면 시야
float32 confidence  # 0~1. 프레임 면적 / bbox 면적 등으로 만든 간이 신뢰도
