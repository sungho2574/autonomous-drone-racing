#!/usr/bin/env python3
"""adr_sim/config/racer_spec.yaml → OpenVINS 캘리브레이션 설정(kalibr 형식) 생성.

    python3 scripts/gen_ov_config.py            # adr_vio 패키지 루트에서

sim 은 캘리브레이션이 '정확히' 알려져 있으므로 기체 스펙에서 바로 뽑는다.
카메라 위치/틸트를 racer_spec.yaml 에서 바꾸면 이 스크립트를 다시 돌려야 VIO 가 안 기운다
(adr_perception/config/gate_pnp.yaml 의 cam_tilt_deg 도 같이 맞출 것).

실기체는 이 파일들을 Kalibr 결과로 교체한다 — 파일 형식이 Kalibr 출력과 같다.
"""
import os

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SPEC = os.path.normpath(os.path.join(PKG, '..', 'adr_sim', 'config', 'racer_spec.yaml'))

# optical(x 오른쪽, y 아래, z 앞) → camera_link(x 앞, y 왼쪽, z 위). adr_perception/pnp.py 와 같은 규약.
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


def Ry(deg: float) -> np.ndarray:
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def T_imu_cam(xyz, tilt_deg: float) -> np.ndarray:
    """IMU(=base_link) ← camera(optical) 4x4. OpenVINS 의 T_imu_cam 규약 그대로.

    IMU 센서는 SDF 에서 base_link 원점에 달려 있으므로 IMU 프레임 == base_link.
    """
    T = np.eye(4)
    T[:3, :3] = Ry(-float(tilt_deg)) @ R_LINK_OPTICAL
    T[:3, 3] = np.asarray(xyz, dtype=float)
    return T


def rows(T: np.ndarray) -> str:
    return '\n'.join('    - [' + ', '.join(f'{v: .10f}' for v in row) + ']' for row in T)


def main():
    with open(SPEC) as f:
        spec = yaml.safe_load(f)
    cam = spec['camera']
    w, h = int(cam['width']), int(cam['height'])
    hfov = float(cam['hfov'])
    # gz 카메라는 왜곡 없는 이상적 핀홀. fx = (w/2) / tan(hfov/2), 주점은 영상 중심.
    # 실제 값은 ros2 topic echo /adr/camera/camera_info 로 확인할 수 있다(0.5 px 차이는 무시 가능).
    fx = fy = (w / 2.0) / np.tan(hfov / 2.0)
    cx, cy = w / 2.0, h / 2.0
    T = T_imu_cam(cam['pose_xyz'], cam['tilt_deg'])

    imucam = f'''%YAML:1.0
# 자동 생성: adr_vio/scripts/gen_ov_config.py (입력: adr_sim/config/racer_spec.yaml). 직접 수정하지 말 것.
# 실기체에서는 Kalibr 결과로 이 파일을 통째로 바꾼다 (형식 동일).
#   T_imu_cam : 카메라(optical) → IMU(=base_link) 변환. 카메라 {cam['pose_xyz']} m, 위로 {cam['tilt_deg']}° 틸트
cam0:
  T_imu_cam:
{rows(T)}
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]   # gz 카메라는 왜곡 없음
  distortion_model: radtan
  intrinsics: [{fx:.4f}, {fy:.4f}, {cx:.4f}, {cy:.4f}]   # fx, fy, cx, cy (hfov {np.degrees(hfov):.1f}°)
  resolution: [{w}, {h}]
  rostopic: /adr/camera/image_raw
  timeshift_cam_imu: 0.0
'''

    # gz IMU 노이즈는 샘플당 stddev(이산). OpenVINS 는 연속시간 밀도를 받는다: σ_c = σ_d / sqrt(rate).
    # 모델 SDF 값(250 Hz): gyro 8.727e-4 rad/s → 5.5e-5,  accel 6.37e-3 m/s² → 4.0e-4.
    # 그대로 쓰면 필터가 IMU 를 과신해 발산하기 쉬워 3배로 부풀렸다(시뮬 미모델링분 + 여유).
    # 바이어스 random walk 는 SDF 에 아예 없지만(=0) 프로세스 노이즈가 0 이면 안 되므로 작은 값을 준다.
    imu = '''%YAML:1.0
# 자동 생성: adr_vio/scripts/gen_ov_config.py. 직접 수정하지 말 것.
imu0:
  T_i_b:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  accelerometer_noise_density: 1.2e-03    # 계산값 4.0e-04 의 3배
  accelerometer_random_walk: 1.0e-04      # SDF 에 바이어스 RW 없음 → 작은 값
  gyroscope_noise_density: 1.7e-04        # 계산값 5.5e-05 의 3배
  gyroscope_random_walk: 1.0e-05
  rostopic: /adr/imu
  time_offset: 0.0
  update_rate: 250.0
  model: "kalibr"
  Tw: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  R_IMUtoGYRO: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  Ta: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  R_IMUtoACC: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
  Tg: [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
'''

    for name, text in (('kalibr_imucam_chain.yaml', imucam), ('kalibr_imu_chain.yaml', imu)):
        path = os.path.join(PKG, 'config', name)
        with open(path, 'w') as f:
            f.write(text)
        print('wrote', path)
    print(f'  intrinsics fx=fy={fx:.2f} cx={cx:.1f} cy={cy:.1f}, '
          f'camera {cam["pose_xyz"]} tilt {cam["tilt_deg"]}°')


if __name__ == '__main__':
    main()
