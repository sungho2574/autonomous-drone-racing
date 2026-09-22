"""PnP 왕복 검증: 알려진 기체 포즈로 게이트 꼭짓점을 투영 → 다시 PnP → 원래 포즈가 나오는가.

좌표계 부호 실수는 rviz 로는 잘 안 보인다. 여기서 잡는다.
"""
import numpy as np
import pytest


from adr_perception.pnp import (camera_extrinsic, drone_pose_in_map, gate_object_points,
                                gate_world_transform, order_quad)

W, H, HFOV = 640, 480, np.pi / 2
FX = (W / 2) / np.tan(HFOV / 2)
K = np.array([[FX, 0, W / 2], [0, FX, H / 2], [0, 0, 1.0]])
DIST = np.zeros(5)
INNER = 1.5
CAM_XYZ, CAM_TILT = (0.045, 0.0, 0.022), 20.0
T_BO = camera_extrinsic(CAM_XYZ, CAM_TILT)


def drone_T(x, y, z, yaw, pitch=0.0):
    """map ← base_link (FLU). yaw 는 ENU CCW, pitch 는 기수 숙임(+) [rad]."""
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    cp, sp = np.cos(pitch), np.sin(pitch)
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    T = np.eye(4)
    T[:3, :3] = Rz @ Ry
    T[:3, 3] = (x, y, z)
    return T


def project(T_map_base, gate_center, gate_yaw):
    """기체 포즈에서 본 개구부 꼭짓점 픽셀 (4,2). 시야 밖이면 None."""
    T_map_opt = T_map_base @ T_BO
    T_opt_gate = np.linalg.inv(T_map_opt) @ gate_world_transform(gate_center, gate_yaw)
    p = (T_opt_gate @ np.hstack([gate_object_points(INNER), np.ones((4, 1))]).T).T[:, :3]
    if (p[:, 2] <= 0.05).any():
        return None
    uv = (K @ p.T).T
    uv = uv[:, :2] / uv[:, 2:3]
    if (uv[:, 0] < 0).any() or (uv[:, 0] > W).any() or (uv[:, 1] < 0).any() or (uv[:, 1] > H).any():
        return None
    return uv


# (기체 x, y, z, yaw, pitch) — 게이트 1 (4,0,1.5, ψ=90°) 을 바라보는 여러 자세.
# 카메라가 20° 위로 틸트돼 있어 가까운 거리에서는 개구부 아래 꼭짓점이 화면 밖으로 나간다.
# 실제 비행 범위 안의 자세를 쓴다 (visible_range 테스트 참고).
GATE_C, GATE_YAW = np.array([4.0, 0.0, 1.5]), np.pi / 2
POSES = [
    (4.0, -3.0, 1.0, np.pi / 2, 0.0),                       # 게이트보다 낮게, 수평
    (4.0, -4.0, 1.5, np.pi / 2, np.radians(12)),            # 같은 높이, 기수 숙임
    (3.4, -3.0, 1.4, np.pi / 2 - 0.15, np.radians(15)),     # 옆에서 비스듬히
    (4.6, -2.5, 1.7, np.pi / 2 + 0.2, np.radians(20)),      # 반대쪽 위에서
]


@pytest.mark.parametrize('pose', POSES)
def test_roundtrip(pose):
    T_true = drone_T(*pose)
    uv = project(T_true, GATE_C, GATE_YAW)
    assert uv is not None, f'{pose} 에서 게이트가 시야 밖 — 테스트 포즈를 고칠 것'
    T_est, err = drone_pose_in_map(uv, INNER, K, DIST, GATE_C, GATE_YAW, T_BO)
    assert T_est is not None
    assert err < 1e-6, f'재투영오차 {err}'
    assert np.allclose(T_est[:3, 3], T_true[:3, 3], atol=1e-6), \
        f'위치 {T_est[:3, 3]} != {T_true[:3, 3]}'
    assert np.allclose(T_est[:3, :3], T_true[:3, :3], atol=1e-6), '자세 불일치'


@pytest.mark.parametrize('pose', POSES)
def test_order_quad_matches_projection_order(pose):
    """검출기의 꼭짓점 정렬이 투영 순서(TL→TR→BR→BL)와 같아야 PnP 대응이 맞는다."""
    uv = project(drone_T(*pose), GATE_C, GATE_YAW)
    assert uv is not None
    for roll in range(4):                       # 어떤 순서로 들어와도
        shuffled = np.roll(uv, roll, axis=0)
        assert np.allclose(order_quad(shuffled), uv, atol=1e-3), f'roll={roll} 정렬 실패'


def test_visible_range():
    """카메라 상향 틸트(20°)의 실제 제약을 박아 둔다 — 틸트를 바꾸면 여기가 먼저 깨진다.

    비행 고도(=게이트 높이 1.5 m) 정면 접근 기준 실측:
      수평 자세: 3 m 이상에서 4점이 다 보인다 (2.5 m 이하는 아래 꼭짓점이 화면 밖)
      기수 5° 숙임: 2 m 부터 보인다
    틸트 30° 일 때는 수평 자세로 8 m 가 필요했다 — 20° 로 낮춘 이유.
    """
    assert project(drone_T(4.0, -3.0, 1.5, np.pi / 2), GATE_C, GATE_YAW) is not None, \
        '수평 3 m 에서 안 보이면 틸트가 너무 크다'
    assert project(drone_T(4.0, -2.0, 1.5, np.pi / 2), GATE_C, GATE_YAW) is None, \
        '수평 2 m 에서 보이면 틸트/FOV 가 바뀐 것'
    for d in (2.0, 3.0, 4.0, 6.0):
        assert project(drone_T(4.0, -d, 1.5, np.pi / 2, np.radians(5)), GATE_C, GATE_YAW) is not None, \
            f'{d} m 에서 5° 숙였는데 안 보임'


def test_noise_error_structure():
    """단일 평면 게이트 PnP 의 오차 구조 — 깊이가 가장 정확하고 횡방향이 약하다.

    3 m / 1 px 기준 실측(중앙값): 깊이 0.009 m, 횡방향 0.088 m, 자세 1.6°.

    이 한계값은 회귀 감지용이다. solve_gate_in_camera 가 solver 를 전부 돌려 보지 않고
    IPPE_SQUARE 해만 쓰면 횡방향이 0.43 m 로 5배 나빠지므로 여기서 걸린다.
    objectPoints 순서나 대응이 틀어져도 마찬가지.
    """
    T_true = drone_T(4.0, -3.0, 1.5, np.pi / 2, np.radians(15))
    uv = project(T_true, GATE_C, GATE_YAW)
    n = np.array([0.0, 1.0, 0.0])            # 게이트 법선 = 깊이 방향
    rng = np.random.default_rng(0)
    depth, lateral = [], []
    for _ in range(200):
        T_est, _ = drone_pose_in_map(uv + rng.normal(0, 1.0, uv.shape), INNER, K, DIST,
                                     GATE_C, GATE_YAW, T_BO)
        assert T_est is not None
        e = T_est[:3, 3] - T_true[:3, 3]
        depth.append(abs(e @ n))
        lateral.append(np.linalg.norm(e - (e @ n) * n))
    assert np.median(depth) < 0.03, f'깊이 오차 {np.median(depth):.3f} m — 스케일이 틀어졌다'
    assert np.median(lateral) < 0.20, \
        f'횡방향 오차 {np.median(lateral):.3f} m — solver 를 전부 돌리고 있는지 확인할 것'
