"""맵 기하 + 21 차원 관측 생성 (numpy 전용).

학습 env `racing_env/racer_env.py::_raw_obs` 와 **같은 식, 같은 순서**다.
동등성은 `racing_env/export_policy.py` 의 테스트 (B) 로 증명한다
(실제 롤아웃 상태 1000+ 개, float32 환산 후 일치).

관측 레이아웃 (전부 ground truth, 잡음·지연 없음)

    idx  0: 2   p_gi     목표 게이트 로컬 좌표계에서 본 기체 위치      [m]
    idx  3: 5   v_gi     같은 좌표계에서 본 속도                      [m/s]
    idx  6: 8   Φ        [roll, pitch, wrap(yaw - gate_yaw)]          [rad]
    idx  9:11   Ω        동체 각속도 (p, q, r) FLU                    [rad/s]
    idx 12:15   u_last   직전 **클립된** 액션 4 개                    [-1,1]
    idx 16:18   p_next   목표→다음 게이트 벡터 (목표 로컬)            [m]
    idx 19   :  ψ_next   wrap(next_yaw - gate_yaw)                    [rad]
    idx 20   :  θ_uptilt 카메라 업틸트 (학습 중 에피소드마다 40~50°)  [rad]

좌표계: 맵 = ENU(x 동, y 북, z 상), 동체 = FLU(x 전, y 좌, z 상),
쿼터니언 = **(x, y, z, w) 스칼라 마지막** — ROS `geometry_msgs/Quaternion` 과 같은 순서다.
게이트 로컬: +x = 정방향 통과 방향(= yaw 방향), +y = 좌, +z = 상.
"""
from __future__ import annotations

import os

import numpy as np
import yaml

OBS_DIM = 21
ACT_DIM = 4
MAX_HOLE = 2


def wrap_to_pi(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def rpy_from_quat(q):
    """단위 쿼터니언 (x,y,z,w) → ZYX 오일러 (roll, pitch, yaw). (...,4) → (...,3).

    학습 env `rpy_from_quat` 와 글자 그대로 같은 식이다.
    """
    q = np.asarray(q, dtype=np.float64)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.stack([roll, pitch, yaw], axis=-1)


def rotmat_from_quat(q):
    """(x,y,z,w) → 동체→월드 회전행렬 R. (...,4) → (...,3,3).

    roma `unitquat_to_rotmat` 과 같은 값 (roma 도 xyzw 다).
    """
    q = np.asarray(q, dtype=np.float64)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.empty(q.shape[:-1] + (3, 3), dtype=np.float64)
    R[..., 0, 0] = 1 - 2 * (y * y + z * z)
    R[..., 0, 1] = 2 * (x * y - z * w)
    R[..., 0, 2] = 2 * (x * z + y * w)
    R[..., 1, 0] = 2 * (x * y + z * w)
    R[..., 1, 1] = 1 - 2 * (x * x + z * z)
    R[..., 1, 2] = 2 * (y * z - x * w)
    R[..., 2, 0] = 2 * (x * z - y * w)
    R[..., 2, 1] = 2 * (y * z + x * w)
    R[..., 2, 2] = 1 - 2 * (x * x + y * y)
    return R


# --------------------------------------------------------------------- 맵 기하

class MapGeom:
    """맵 yaml 하나의 게이트 기하. `racing_env/track.py::RacingTrack` 의 numpy 판.

    판정에 쓰는 보정값(pass_half, panel_*, hole_*)까지 같은 식으로 만든다.
    """

    def __init__(self, path, *, virtual_gate_ids=(5, 6, 10, 12), inflate=0.134,
                 bounds_margin=3.0, z_min=0.0, z_max=6.0, loop=True, extra_xy=(),
                 wall_clear=0.0, start_xyz=None, gate_thickness=None):
        """`start_xyz` / `gate_thickness` / `wall_clear` 는 **설정에서 오는 값**이다.

        `maps/gate_map.yaml` 은 성호의 OpenVINS+PnP 쪽과 **글자 하나까지 같은 파일**
        이라 학습 전용 키(start, thickness)를 넣지 않았다. 맵에 그 블록이 있으면
        (옛 big_track / A2RL 맵) 맵 값을 쓰고, 없으면 여기 인자로 받는다.
        `load_map_from_config()` 가 `config_asrun.yaml` 의 `track:` 에서 자동으로
        채워 주므로 보통은 신경 쓸 필요가 없다.
        """
        with open(path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
        gates = raw["gates"]
        if not gates:
            raise ValueError(f"게이트가 없는 맵: {path}")

        self.path = str(path)
        self.gate_ids = [int(g["id"]) for g in gates]
        G = self.num_gates = len(gates)

        pos = np.array([[float(g["x"]), float(g["y"]), float(g["z"])] for g in gates])
        yaw = np.deg2rad(np.array([float(g["yaw_deg"]) for g in gates]))
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.zeros((G, 3, 3))
        R[:, 0, 0] = c
        R[:, 0, 1] = -s
        R[:, 1, 0] = s
        R[:, 1, 1] = c
        R[:, 2, 2] = 1.0

        self.gate_pos = pos                       # (G,3) 월드 ENU
        self.gate_yaw = yaw                       # (G,)  rad
        self.gate_R = R                           # (G,3,3) 로컬→월드
        self.gate_RT = np.transpose(R, (0, 2, 1))  # (G,3,3) 월드→로컬

        g = raw["gate"]
        self.inner = float(g["inner_size"])
        self.outer = float(g["outer_size"])
        # 맵의 thickness > 설정의 gate_thickness > 기본 0.05 (학습 track.py 와 같은 순서)
        self.thickness = float(
            g["thickness"] if "thickness" in g
            else (gate_thickness if gate_thickness is not None else 0.05))
        self.inflate = float(inflate)
        self.pass_half = 0.5 * self.inner - self.inflate
        self.frame_half = 0.5 * self.outer + self.inflate
        if self.pass_half <= 0.0:
            raise ValueError("inflate 가 게이트 내경 절반 이상이다")

        panels = raw.get("panels") or {}
        p_hy = np.full(G, 0.5 * self.outer)
        p_lo = np.full(G, -0.5 * self.outer)
        p_hi = np.full(G, +0.5 * self.outer)
        h_z = np.zeros((G, MAX_HOLE))
        h_hf = np.full((G, MAX_HOLE), -1.0)
        h_hf[:, 0] = 0.5 * self.inner
        self.panel_of = [None] * G
        for i, gt in enumerate(gates):
            name = gt.get("panel")
            if name is None:
                continue
            if name not in panels:
                raise ValueError(f"게이트 {gt['id']} 가 없는 panel '{name}' 을 쓴다")
            sp = panels[name]
            self.panel_of[i] = str(name)
            p_hy[i] = 0.5 * float(sp["width"])
            p_lo[i] = float(sp["z_lo"]) - pos[i, 2]
            p_hi[i] = float(sp["z_hi"]) - pos[i, 2]
            holes = sp["holes"]
            if len(holes) > MAX_HOLE:
                raise ValueError(f"panel '{name}': 구멍은 최대 {MAX_HOLE} 개")
            h_hf[i, :] = -1.0
            for k, h in enumerate(holes):
                h_z[i, k] = float(h["z"]) - pos[i, 2]
                h_hf[i, k] = 0.5 * float(h["size"])

        # 기체 반경 보정: 판은 넓히고 구멍은 좁힌다
        self.panel_half_y = p_hy + self.inflate
        self.panel_lo = p_lo - self.inflate
        self.panel_hi = p_hi + self.inflate
        self.hole_z = h_z
        self.hole_half = h_hf - self.inflate

        virt = {int(v) for v in virtual_gate_ids}
        self.virtual_mask = np.array([gid in virt for gid in self.gate_ids], dtype=bool)
        self.solid_idx = np.nonzero(~self.virtual_mask)[0]

        # 이륙 지점. 맵에 start: 가 있으면 그것, 없으면 설정의 track.start.
        st = raw.get("start")
        if st is None:
            st = start_xyz
        if st is None:
            raise ValueError(
                f"이륙 지점이 없다: {path} 에 start: 블록이 없고 설정에도 "
                f"track.start 가 없다. 둘 중 하나는 있어야 한다.")
        if isinstance(st, dict):
            sx, sy, sz = float(st["x"]), float(st["y"]), float(st["takeoff_z"])
        else:
            sx, sy, sz = (float(v) for v in st)
        self.start_pos = np.array([sx, sy, sz])

        # 코스 바운딩 박스. 학습 `racing_env/track.py` 와 같은 두 모드다.
        #
        # (a) `room:` 이 있는 맵 (gate_map = 실내 실비행 트랙) — 방이 **물리적인
        #     벽**이다. 벽 4 면 / 천장 / 바닥을 그대로 경계로 쓰고 `wall_clear`
        #     (기체 반폭) 만큼 안으로 줄인다. 질점 판정을 반경 r 구 판정으로
        #     만드는 것이고, 게이트 쪽 `inflate` 와 같은 규칙이다.
        #     bounds_margin / z_min / z_max 는 쓰지 않는다.
        # (b) `room:` 이 없는 맵 (big_track, A2RL) — 게이트 + 이륙 지점의
        #     바운딩 박스 + bounds_margin 이 가상 울타리다 (종전과 동일).
        self.wall_clear = float(wall_clear)
        room = raw.get("room")
        self.room = None
        if room is not None:
            ox, oy = float(room["origin_x"]), float(room["origin_y"])
            lo = np.array([ox, oy, 0.0])
            hi = np.array([ox + float(room["width"]),
                           oy + float(room["length"]),
                           float(room["height"])])
            c = self.wall_clear
            if np.any(hi - lo <= 2.0 * c):
                raise ValueError(f"wall_clear({c}) 가 방 크기의 절반 이상이다")
            self.room = dict(lo=lo, hi=hi)
            self.room_lo, self.room_hi = lo, hi
            self.bounds_lo, self.bounds_hi = lo + c, hi - c
        else:
            all_xy = [pos[:, 0:2], self.start_pos[None, 0:2]]
            if len(extra_xy):
                all_xy.append(np.asarray(extra_xy, dtype=float).reshape(-1, 2))
            all_xy = np.vstack(all_xy)
            self.bounds_lo = np.array([all_xy[:, 0].min() - bounds_margin,
                                       all_xy[:, 1].min() - bounds_margin, z_min])
            self.bounds_hi = np.array([all_xy[:, 0].max() + bounds_margin,
                                       all_xy[:, 1].max() + bounds_margin, z_max])
            self.room_lo, self.room_hi = self.bounds_lo, self.bounds_hi

        self.loop = bool(loop)
        self.num_segments = G if self.loop else G - 1
        nxt = [(i + 1) % G for i in range(G)]
        if not self.loop:
            nxt[-1] = -1
        self.next_idx = np.array(nxt, dtype=np.int64)

    # ---- 편의
    def index_of(self, gate_id):
        """맵 yaml 의 게이트 id → 내부 인덱스 (관측/판정은 전부 인덱스를 쓴다)."""
        return self.gate_ids.index(int(gate_id))

    def describe(self):
        L = [f"map {self.path}",
             f"게이트 {self.num_gates} ids={self.gate_ids}",
             f"inner/outer {self.inner}/{self.outer} inflate {self.inflate} "
             f"pass_half {self.pass_half:.3f}",
             f"start {np.round(self.start_pos, 3).tolist()}",
             f"bounds {np.round(self.bounds_lo,2).tolist()} ~ "
             f"{np.round(self.bounds_hi,2).tolist()}"]
        if self.room is not None:
            L.append(f"room(벽) {np.round(self.room['lo'],2).tolist()} ~ "
                     f"{np.round(self.room['hi'],2).tolist()}  "
                     f"wall_clear={self.wall_clear}")
        pn = [(self.gate_ids[i], p) for i, p in enumerate(self.panel_of) if p]
        L.append(f"panels {pn}")
        return "\n".join(L)


def load_map(path, **kw):
    """맵 yaml → MapGeom. 기본값은 학습 config 의 track 기본값과 같다."""
    return MapGeom(path, **kw)


def load_map_from_config(config_yaml, map_yaml=None, repo_root=None):
    """`config_asrun.yaml` 의 `track:` 블록을 그대로 써서 MapGeom 을 만든다.

    학습과 **완전히 같은** 판정 경계(inflate, bounds, virtual gate)를 쓰려면 이걸 쓴다.
    `map_yaml` 을 주면 config 의 경로 대신 그것을 읽는다 (맵 파일 위치가 다른 환경).
    """
    with open(config_yaml, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    t = cfg["track"]
    p = map_yaml or t["map_yaml"]
    if not os.path.isabs(p) and repo_root:
        p = os.path.join(repo_root, p)
    return MapGeom(p,
                   virtual_gate_ids=tuple(t.get("virtual_gate_ids") or ()),
                   inflate=float(t["inflate"]),
                   bounds_margin=float(t["bounds_margin"]),
                   z_min=float(t["z_min"]), z_max=float(t["z_max"]),
                   loop=bool(t["loop"]),
                   extra_xy=[tuple(float(v) for v in q)
                             for q in (t.get("extra_xy") or ())],
                   # gate_map 전용 — 맵이 성호 쪽과 공유 파일이라 학습 전용 키가
                   # 설정에만 있다. 옛 설정(A2RL/big_track)에는 없으므로 기본값.
                   wall_clear=float(t.get("wall_clear", 0.0)),
                   start_xyz=t.get("start"),
                   gate_thickness=t.get("gate_thickness"))


# ------------------------------------------------------------------- 관측 생성

class ObsBuilder:
    """상태 + 목표 게이트 → 21 차원 관측."""

    def __init__(self, mp: MapGeom):
        self.mp = mp

    def single(self, x, v, q, w, prev_action, gate_idx, uptilt):
        """(3,), (3,), (4,)xyzw, (3,), (4,), int, float[rad] → (21,) float32."""
        o = self.batch(np.asarray(x, dtype=np.float64)[None, :],
                       np.asarray(v, dtype=np.float64)[None, :],
                       np.asarray(q, dtype=np.float64)[None, :],
                       np.asarray(w, dtype=np.float64)[None, :],
                       np.asarray(prev_action, dtype=np.float64)[None, :],
                       np.array([int(gate_idx)]),
                       np.array([float(uptilt)]))
        return o[0]

    def batch(self, x, v, q, w, prev_action, gate_idx, uptilt):
        """위와 같은 입력의 (N,...) 판. → (N,21) float32."""
        mp = self.mp
        x = np.asarray(x, dtype=np.float64)
        v = np.asarray(v, dtype=np.float64)
        q = np.asarray(q, dtype=np.float64)
        w = np.asarray(w, dtype=np.float64)
        pa = np.asarray(prev_action, dtype=np.float64)
        gi = np.asarray(gate_idx, dtype=np.int64)
        ut = np.asarray(uptilt, dtype=np.float64)

        RT = mp.gate_RT[gi]                                  # (N,3,3)
        p_gi = np.einsum("nij,nj->ni", RT, x - mp.gate_pos[gi])
        v_gi = np.einsum("nij,nj->ni", RT, v)

        rpy = rpy_from_quat(q)
        phi = np.stack([rpy[:, 0], rpy[:, 1],
                        wrap_to_pi(rpy[:, 2] - mp.gate_yaw[gi])], axis=-1)

        ni = mp.next_idx[gi]
        valid = ni >= 0
        ns = np.clip(ni, 0, None)
        p_next = np.einsum("nij,nj->ni", RT, mp.gate_pos[ns] - mp.gate_pos[gi])
        psi_next = wrap_to_pi(mp.gate_yaw[ns] - mp.gate_yaw[gi])
        p_next = np.where(valid[:, None], p_next, 0.0)
        psi_next = np.where(valid, psi_next, 0.0)

        obs = np.concatenate([p_gi, v_gi, phi, w, pa, p_next,
                              psi_next[:, None], ut[:, None]], axis=-1)
        return obs.astype(np.float32)
