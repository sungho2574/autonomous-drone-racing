"""게이트 개구부 4 꼭짓점 + 게이트 맵 → 카메라/기체 포즈 (ROS 비의존, cv2+numpy 만).

좌표계
  map        : ENU (x East, y North, z Up)
  base_link  : FLU (x 전방, y 좌, z 상) — ROS 기체 표준
  camera_link: base_link 에 (xyz, pitch=-tilt) 로 고정. +x 가 광축, tilt>0 이면 위를 본다
  optical    : OpenCV 카메라 (x 우, y 하, z 전방). cv2 의 rvec/tvec 은 이 프레임 기준
  gate(G)    : 개구부 중심 원점. x_G = 이미지상 오른쪽, y_G = 위(=map +z), z_G = -법선(카메라 쪽)
               → ArUco/IPPE_SQUARE 관례와 같아서 objectPoints 를 그대로 쓸 수 있다

게이트 yaw ψ (gates.yaml, ENU CCW, 통과 방향 법선 n = (cosψ, sinψ, 0)) 로부터
  x_G = ( sinψ, -cosψ, 0)      # 통과 방향을 바라볼 때의 오른쪽
  y_G = (0, 0, 1)
  z_G = x_G × y_G = (-cosψ, -sinψ, 0) = -n
"""
from __future__ import annotations

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# optical(x우 y하 z전) → camera_link(FLU). p_link = R @ p_optical
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])

# 전부 돌려서 재투영오차가 가장 작은 해를 쓴다. IPPE_SQUARE 는 정사각 마커 전용이라 보통 제일
# 정확하지만, 완전 정면·대칭에서 nan 을 내거나 멀쩡한 입력에 엉뚱한 해를 주는 자세가 있다.
_PNP_FLAGS = (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_ITERATIVE)


def _T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def order_quad(pts) -> np.ndarray:
    """4점을 이미지 기준 시계방향 TL→TR→BR→BL 로 정렬 — objectPoints 순서와 짝을 이룬다.

    이미지 좌표는 y 가 아래로 커지므로 atan2 각도 증가 = 화면상 시계방향이다.
    중심 기준 각도 정렬 후, x+y 가 가장 작은 점(좌상단)이 앞에 오도록 회전시킨다.
    기체가 대체로 수평이라는 전제 — 롤이 45°를 넘으면 TL 판정이 흔들린다.
    """
    p = np.asarray(pts, dtype=np.float64).reshape(4, 2)
    c = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - c[1], p[:, 0] - c[0]))]
    return np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0)


def gate_object_points(inner_size: float) -> np.ndarray:
    """게이트 로컬 프레임의 개구부 꼭짓점 (4,3). 순서 TL→TR→BR→BL (IPPE_SQUARE 규약)."""
    s = float(inner_size) / 2.0
    return np.array([[-s, +s, 0.0],
                     [+s, +s, 0.0],
                     [+s, -s, 0.0],
                     [-s, -s, 0.0]], dtype=np.float64)


def gate_world_transform(center, yaw: float) -> np.ndarray:
    """map ← gate 변환 (4,4). center = 개구부 중심 [m], yaw = ENU CCW 법선 방위각 [rad].

    게이트 로컬축(x=이미지 오른쪽, y=위, z=-법선)을 map 기준으로 세운 것 = 회전행렬의 열.
    """
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.column_stack(([s, -c, 0.0],      # x_G
                         [0.0, 0.0, 1.0],   # y_G
                         [-c, -s, 0.0]))    # z_G
    return _T(R, center)


def camera_extrinsic(xyz, tilt_deg: float) -> np.ndarray:
    """base_link ← optical 변환 (4,4).

    xyz: camera_link 원점의 base_link 기준 위치 [m]
    tilt_deg: 위쪽 틸트 각 [deg] (모델 SDF 의 rpy pitch = -tilt)
    """
    Ry = Rotation.from_euler('y', -float(tilt_deg), degrees=True).as_matrix()
    return _T(Ry @ R_LINK_OPTICAL, xyz)


def reprojection_error(obj_pts, img_pts, rvec, tvec, K, dist) -> float:
    """평균 재투영 오차 [px]."""
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    return float(np.linalg.norm(proj.reshape(-1, 2) - np.asarray(img_pts).reshape(-1, 2), axis=1).mean())


def solve_gate_in_camera(corners_px, inner_size: float, K, dist=None):
    """개구부 꼭짓점(TL→TR→BR→BL) → (optical ← gate 변환 4x4, 재투영오차).

    평면 4점이라 해가 둘 나올 수 있다(planar pose ambiguity). 재투영 오차가 작은 쪽을 고른다.
    오도메트리로 고르면 더 안정적이지만, 그러면 PnP 성능 평가가 아니게 되므로 쓰지 않는다.
    실패하면 (None, inf).
    """
    img = np.ascontiguousarray(np.asarray(corners_px, dtype=np.float64).reshape(4, 1, 2))
    obj = gate_object_points(inner_size).reshape(4, 1, 3)
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64).reshape(-1)
    best, best_e = None, float('inf')
    for flag in _PNP_FLAGS:
        try:
            # 반환값 4개: (해의 개수, rvecs, tvecs, 재투영오차). 오차는 직접 다시 계산한다.
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, K, dist, flags=flag)
        except cv2.error:
            continue
        for rvec, tvec in zip(rvecs or (), tvecs or ()):
            # IPPE_SQUARE 는 완전 정면·대칭인 사각형에서 rvec=nan 을 내놓는 경우가 있다.
            # (꼭짓점을 1e-9 흔들면 멀쩡해진다 — 순수 수치 예외) → 유한한 해만 받는다.
            if not (np.isfinite(rvec).all() and np.isfinite(tvec).all()):
                continue
            e = reprojection_error(obj, img, rvec, tvec, K, dist)
            if np.isfinite(e) and e < best_e:
                best, best_e = (rvec, tvec), e
        # 먼저 성공한 solver 에서 멈추지 않는다. IPPE_SQUARE 는 노이즈 없는 입력에도
        # 재투영오차 10 px 대의 엉뚱한 해를 내놓는 자세가 있다(SQPNP 는 같은 입력을 정확히 푼다).
        # 전부 돌려 보고 재투영오차가 가장 작은 해를 고른다 — 4점 PnP 라 비용은 무시할 만하다.
    if best is None:
        return None, float('inf')
    R, _ = cv2.Rodrigues(best[0])
    return _T(R, best[1].reshape(3)), best_e


def drone_pose_in_map(corners_px, inner_size, K, dist, gate_center, gate_yaw, T_base_optical):
    """개구부 꼭짓점 → map 기준 기체 포즈 (4,4) 와 재투영오차. 실패하면 (None, inf)."""
    T_opt_gate, err = solve_gate_in_camera(corners_px, inner_size, K, dist)
    if T_opt_gate is None:
        return None, err
    T_map_gate = gate_world_transform(gate_center, gate_yaw)
    T_map_opt = T_map_gate @ np.linalg.inv(T_opt_gate)
    return T_map_opt @ np.linalg.inv(T_base_optical), err


def quat_from_R(R: np.ndarray):
    """회전행렬 → 쿼터니언 (x, y, z, w) — ROS geometry_msgs 순서."""
    return Rotation.from_matrix(np.asarray(R, dtype=float)).as_quat()
