# 다항식 궤적 한 구간. 각 축은 세그먼트 로컬 시간 tau in [0, duration] 에 대한
# 다항식 p(tau) = sum_k c[k] * tau^k  (계수 오름차순, 길이 = 차수+1)
float64 duration      # [s]
float64[] cx          # x [m]   (ENU)
float64[] cy          # y [m]
float64[] cz          # z [m]
float64[] cyaw        # yaw [rad], ENU +x 기준 CCW, unwrap 된 연속값
