"""gates.yaml 코스 정의 로드 및 웨이포인트 생성 (ROS 비의존).

gates.yaml 스키마 (adr_bringup/config/gates.yaml 참고):
  course: {radius, height, direction}      # 참고용 메타
  gate:   {inner_size, outer_size, thickness}
  start:  {x, y, takeoff_z}
  gates:  [{id, x, y, z, yaw_deg}, ...]     # yaw_deg: ENU, +x 기준 CCW, 통과 방향(법선)
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin, atan2

import numpy as np
import yaml


@dataclass
class Gate:
    id: int
    x: float
    y: float
    z: float
    yaw: float          # [rad], ENU CCW

    @property
    def center(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def normal(self) -> np.ndarray:
        return np.array([cos(self.yaw), sin(self.yaw), 0.0])


@dataclass
class Course:
    gates: list[Gate]
    start: np.ndarray           # (3,) 이륙 후 호버점
    inner_size: float
    outer_size: float
    thickness: float
    raw: dict


def load_course(path: str) -> Course:
    with open(path) as f:
        d = yaml.safe_load(f)
    gates = [Gate(int(g['id']), float(g['x']), float(g['y']), float(g['z']),
                  radians(float(g['yaw_deg']))) for g in d['gates']]
    s = d['start']
    start = np.array([float(s['x']), float(s['y']), float(s['takeoff_z'])])
    g = d.get('gate', {})
    return Course(gates, start,
                  float(g.get('inner_size', 1.5)), float(g.get('outer_size', 2.1)),
                  float(g.get('thickness', 0.1)), d)


def course_waypoints(course: Course, laps: int = 1, approach_dist: float = 0.0,
                     end_at_start: bool = True):
    """호버점 → (게이트 전·중심·후) × laps → 호버점 순서의 웨이포인트와 yaw.

    approach_dist > 0 이면 각 게이트 앞뒤에 법선 방향 오프셋 점을 넣어
    궤적이 게이트 면에 수직으로 들어가도록 유도한다.
    yaw 는 게이트에서는 통과 방향, 호버점에서는 다음/이전 점을 향한 방향.
    """
    pts: list[np.ndarray] = [course.start.copy()]
    yaws: list[float] = []
    for _ in range(max(1, laps)):
        for g in course.gates:
            if approach_dist > 0:
                pts.append(g.center - approach_dist * g.normal); yaws.append(g.yaw)
            pts.append(g.center); yaws.append(g.yaw)
            if approach_dist > 0:
                pts.append(g.center + approach_dist * g.normal); yaws.append(g.yaw)
    if end_at_start:
        pts.append(course.start.copy())
    # 호버점 yaw: 시작은 첫 점을 향해, 끝은 마지막 게이트 진행 방향 유지
    d0 = pts[1] - pts[0]
    yaw0 = atan2(d0[1], d0[0]) if np.linalg.norm(d0[:2]) > 1e-6 else yaws[0]
    yaws = [yaw0] + yaws
    if end_at_start:
        yaws.append(yaws[-1])
    return np.vstack(pts), np.asarray(yaws)
