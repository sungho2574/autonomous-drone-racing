"""정책 1 스텝 실행기 — 상태 → PX4 VehicleRatesSetpoint 에 넣을 값 (numpy + onnxruntime).

ROS 에 의존하지 않는다. ROS 노드는 성호가 만들고, 이 클래스를 50 Hz 타이머에서
한 번 호출하면 된다.

    from racing_runtime import PolicyRunner, load_map_from_config
    mp = load_map_from_config("config_asrun.yaml", map_yaml="big_track.yaml")
    runner = PolicyRunner("policy.onnx", mp, uptilt_deg=45.0,
                          throttle=dict(mode="hover", thr_hover=0.31, mass_kg=0.80))
    runner.reset(pos_enu)                       # 이륙 직후 1 회
    out = runner.step(dict(pos=p, vel=v, quat=q, rates=w))   # 매 20 ms
    msg.roll = out["roll_rate"]; msg.pitch = out["pitch_rate"]; msg.yaw = out["yaw_rate"]
    msg.thrust_body = [0.0, 0.0, -out["thrust_norm"]]        # FRD, 위로 = -z

입력 상태는 전부 **맵 ENU / 동체 FLU, 쿼터니언 xyzw**다:
    pos   (3,) [m]      맵 ENU
    vel   (3,) [m/s]    맵 ENU (월드 속도. 동체 속도를 넣으면 안 된다)
    quat  (4,) xyzw     동체(FLU)→맵(ENU) 회전
    rates (3,) [rad/s]  동체 FLU (p, q, r)

제어 주기는 50 Hz (0.02 s) 가정이다. 학습 물리는 0.01 s × 2 서브스텝이었다.
주기가 달라지면 (a) 관측의 u_last 간격, (b) 게이트 통과 판정 구간 길이가 달라지므로
50 Hz 를 유지하는 게 좋다.
"""
from __future__ import annotations

import numpy as np

from .action_mapper import ActionMapper
from .gate_tracker import GateTracker
from .obs_builder import OBS_DIM, ObsBuilder
from .px4_convert import body_rates_flu_to_frd, throttle

DEFAULT_UPTILT_DEG = 45.0      # 학습 중에는 에피소드마다 40~50° 를 뽑아 관측에 넣었다


class PolicyRunner:
    def __init__(self, onnx_path, mp, *, uptilt_deg=DEFAULT_UPTILT_DEG,
                 throttle=None, action_mapper=None, providers=None,
                 control_dt=0.02):
        import onnxruntime as ort                      # 여기서만 필요

        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(
            str(onnx_path), sess_options=options,
            providers=providers or ["CPUExecutionProvider"])
        names = [o.name for o in self.sess.get_outputs()]
        if not {"action", "action_raw"}.issubset(names):
            raise ValueError(f"policy.onnx requires action/action_raw outputs: {names}")
        inputs = self.sess.get_inputs()
        if len(inputs) != 1 or inputs[0].type != 'tensor(float)' or inputs[0].shape[-1] != OBS_DIM:
            raise ValueError('policy.onnx requires one float32 [batch,21] input')
        for output in self.sess.get_outputs():
            if output.name in ('action', 'action_raw') and output.shape[-1] != 4:
                raise ValueError('policy.onnx requires [batch,4] actions')
        self.in_name = inputs[0].name
        self.mp = mp
        self.obs_builder = ObsBuilder(mp)
        self.tracker = GateTracker(mp)
        self.mapper = action_mapper or ActionMapper()
        self.uptilt = np.deg2rad(float(uptilt_deg))
        self.throttle_cfg = dict(throttle or {"mode": "sitl"})
        self.control_dt = float(control_dt)
        self.reset()

    # ------------------------------------------------------------------ 상태
    def reset(self, pos=None, target_gate=0):
        """목표 게이트와 직전 액션을 초기화한다. 이륙 직후 1 회 부른다.

        학습에서 에피소드 시작의 u_last 는 **0** 이다 (`prev_action[idx] = 0`).
        """
        self.prev_action = np.zeros(4, dtype=np.float64)
        self.prev_pos = None if pos is None else np.asarray(pos, dtype=np.float64).copy()
        self.tracker.reset(int(target_gate))
        self.n_step = 0

    # ------------------------------------------------------------------ 1 스텝
    def step(self, state, mp=None):
        """상태 dict → 제어 명령 dict. 50 Hz 로 호출한다.

        `mp` 를 주면 그 맵으로 바꿔 쓴다 (보통 생성 때 준 맵을 그대로 쓴다).

        반환 dict
            roll_rate/pitch_rate/yaw_rate  [rad/s] **FRD** — VehicleRatesSetpoint 필드
            thrust_norm                    [0,1]  정규화 스로틀 (thrust_body = [0,0,-u])
            thrust_n                       [N]    집합추력 (공칭 0.80 kg 기준)
            rates_flu                      [rad/s] 변환 전 동체(FLU) 명령
            action / action_raw            정책 출력 (클립 / 클립 전)
            obs                            정책에 들어간 21 차원
            target_gate / target_gate_id   갱신된 목표 게이트
            events                         gate_tracker 판정 dict
            debug                          그 외 (게이트까지 거리, 통과 수, 랩 등)
        """
        if mp is not None and mp is not self.mp:
            self.mp = mp
            self.obs_builder = ObsBuilder(mp)
            self.tracker.mp = mp

        p = np.asarray(state["pos"], dtype=np.float64).reshape(3)
        v = np.asarray(state["vel"], dtype=np.float64).reshape(3)
        q = np.asarray(state["quat"], dtype=np.float64).reshape(4)
        w = np.asarray(state["rates"], dtype=np.float64).reshape(3)

        # ---- 게이트 판정 먼저. 직전 위치 → 현재 위치 구간으로 본다.
        if self.prev_pos is None:
            self.prev_pos = p.copy()
        ev = self.tracker.update(self.prev_pos, p)
        self.prev_pos = p.copy()

        # ---- 관측 → 정책
        obs = self.obs_builder.single(p, v, q, w, self.prev_action,
                                      self.tracker.target, self.uptilt)
        x = obs.reshape(1, OBS_DIM).astype(np.float32)
        raw, act = self.sess.run(["action_raw", "action"], {self.in_name: x})
        act = np.clip(act.reshape(4).astype(np.float64), -1.0, 1.0)
        self.prev_action = act                       # 다음 관측의 u_last (클립된 값)

        # ---- 액션 → 명령
        thrust_n, rates_flu = self.mapper.map(act)
        rates_frd = body_rates_flu_to_frd(rates_flu)
        u = float(throttle(thrust_n, **self.throttle_cfg))
        self.n_step += 1

        gi = self.tracker.target
        d = self.mp.gate_pos[gi] - p
        return {
            "roll_rate": float(rates_frd[0]),
            "pitch_rate": float(rates_frd[1]),
            "yaw_rate": float(rates_frd[2]),
            "thrust_norm": u,
            "thrust_n": float(thrust_n),
            "rates_flu": rates_flu,
            "action": act,
            "action_raw": raw.reshape(4).astype(np.float64),
            "obs": obs,
            "target_gate": int(gi),
            "target_gate_id": int(self.mp.gate_ids[gi]),
            "events": ev,
            "debug": {
                "step": self.n_step,
                "t": self.n_step * self.control_dt,
                "dist_to_gate": float(np.linalg.norm(d)),
                "gates_passed": self.tracker.gates_passed,
                "laps": self.tracker.laps,
                "speed": float(np.linalg.norm(v)),
                "uptilt_deg": float(np.rad2deg(self.uptilt)),
            },
        }
