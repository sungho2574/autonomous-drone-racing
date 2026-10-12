"""racing_runtime — RotorPy·SB3·torch 없이 정책을 돌리는 최소 모듈.

의존성: numpy, onnxruntime, PyYAML (맵/설정 yaml 파싱) — 이 셋뿐이다.

    pip install numpy onnxruntime pyyaml

구성
    obs_builder.py   맵 기하(MapGeom) + 21 차원 관측 생성
    gate_tracker.py  목표 게이트 추적, 통과/역통과/프레임충돌/이탈 판정
    action_mapper.py 액션 4 개 → 집합추력 [N] + 동체 각속도 [rad/s] (FLU)
    px4_convert.py   ENU/FLU ↔ NED/FRD, 추력 [N] → 정규화 스로틀
    policy_runner.py 위 넷 + policy.onnx 를 묶은 1 스텝 실행기

출처: handoff_sungho_a2rl_20261012/racing_runtime.
ROS 연결 및 검증 방법은 레포의 docs/a2rl_rl.md 를 본다.
"""
from .action_mapper import ActionMapper, thrust_from_action
from .gate_tracker import GateTracker
from .obs_builder import (ACT_DIM, OBS_DIM, MapGeom, ObsBuilder, load_map,
                          load_map_from_config, rpy_from_quat, wrap_to_pi)
from .policy_runner import PolicyRunner

__all__ = ["ActionMapper", "thrust_from_action", "GateTracker", "MapGeom",
           "ObsBuilder", "load_map", "load_map_from_config", "PolicyRunner",
           "rpy_from_quat", "wrap_to_pi", "OBS_DIM", "ACT_DIM"]
