# 레이싱 게이트 1개. pose 의 원점 = 개구부 중심, +x = 통과 방향(법선), ENU(map).
int32 id
geometry_msgs/Pose pose
float64 inner_size    # 개구부 한 변 [m]
float64 outer_size    # 프레임 바깥 한 변 [m]
