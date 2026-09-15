# piecewise polynomial 궤적 (min-snap 출력). frame 은 ENU(map).
# 전체 시간 = sum(segments[i].duration). 첫 세그먼트 시작이 t=0.
std_msgs/Header header
PolynomialSegment[] segments
