#!/usr/bin/env python3
"""SkyDreamer track.py(NED) → adr_bringup/config/maps/*.yaml(ENU) 변환.

    python3 scripts/import_skydreamer_tracks.py     # 어디서 실행해도 된다

track.py 의 게이트 좌표를 여기에 그대로 복사해 두고 변환만 한다. 원본이 바뀌면 아래
상수/리스트를 갱신해 다시 돌린다. 생성된 맵을 gz 월드로 만드는 것은 gen_world.py --all.

NED(x=North, y=East, z=Down) → ENU(x=East, y=North, z=Up):
    x_enu = y_ned,  y_enu = x_ned,  z_enu = -z_ned
yaw: NED 는 North 에서 East 로, ENU 는 East 에서 North 로 재므로 yaw_enu = 90° - yaw_ned.
    검산: yaw_ned=90(=+Y_ned=East) → yaw_enu=0(=+x_enu=East) ✓
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, '..', '..', 'adr_bringup', 'config', 'maps'))
GATE_1_X, GATE_2_X, GATE_Z, BIG_Z, LOOP_GAP = 3.0, -2.0, -1.35, -1.25, 2.7

INVERTED_LOOP = [
    (GATE_1_X, 0.0, GATE_Z, 90.0),
    (GATE_2_X, 0.0, GATE_Z - LOOP_GAP, 270.0, False),
    (GATE_2_X, 0.0, GATE_Z, 90.0),
]
LADDER_INVERTED_LOOP = [
    (GATE_1_X, 0.0, GATE_Z, 90.0),
    (GATE_1_X + 1.0, 1.0, GATE_Z - 1.2, 270.0, False),
    (GATE_1_X - 1.0, 0.0, GATE_Z - 0.6, 90.0, False),
    (GATE_2_X, 0.0, GATE_Z - LOOP_GAP, 270.0, False),
    (GATE_2_X, 0.0, GATE_Z, 90.0),
]
BIG_TRACK = [
    (3.7, -9.5, BIG_Z, 90.0),
    (6.0, -3.4, BIG_Z, 90.0),
    (6.0, 2.8, BIG_Z, 90.0),
    (2.6, 8.5, BIG_Z, 135.0),
    (5.5, 11.5, BIG_Z - 2.2, 250.0, False),
    (0.5, 10.5, BIG_Z - 1.0, 120.0, False),
    (-3.0, 8.9, BIG_Z, 215.0),
    (-6.0, 2.8, BIG_Z, 270.0),
    (-6.0, -3.4, BIG_Z, 270.0),
    (-3.0, -7.5, BIG_Z, 330.0, False),
    (0.0, -0.3, BIG_Z, 0.0),
    (3.7, -9.5, BIG_Z - LOOP_GAP, 270.0, False),
]

NOTES = {  # 게이트별 한 줄 설명 (track.py 주석 요약)
 'inverted_loop': ['게이트 1 — +x 로 통과',
                   '(논문 가상) 게이트 2 위 2.7 m, 반대 방향. "두 번째 게이트 위로 넘어간다"',
                   '게이트 2 — split-S: 뒤집어 당겨 내려오며 진행 방향이 다시 +x 로'],
 'ladder_loop': ['게이트 1',
                 '(논문 가상) ladder — 360° 좌선회를 좁게 묶는 옆·위 점',
                 '(논문 가상) ladder — 게이트 1 위로 되돌아오는 점',
                 '(논문 가상) 게이트 2 위 2.7 m (inverted loop 진입)',
                 '게이트 2 — split-S'],
 'big_track': ['출발 게이트 (논문 "top-left gate")', '완만한 좌선회 후 첫 직선', '직선 두 번째',
               '우선회 진입 — 45° 기울어진 게이트', '(논문 가상) ladder 위로', '(논문 가상) ladder 되돌아오기',
               '급강하로 진입', '반대쪽 직선', '직선 두 번째',
               '(논문 가상) 급제동 후 샤프 우선회', '가운데 게이트 (나머지와 90° 틀어짐)',
               '(논문 가상) 첫 게이트 위 split-S 로 마무리'],
}

def to_enu(g):
    x, y, z, yaw = g[0], g[1], g[2], g[3]
    vis = bool(g[4]) if len(g) > 4 else True
    return y, x, -z, (90.0 - yaw) % 360.0, vis

def emit(name, gates, header, inner, outer, start_back=3.0):
    rows = [to_enu(g) for g in gates]
    x0, y0, z0, yaw0, _ = rows[0]
    sx = x0 - start_back * math.cos(math.radians(yaw0))
    sy = y0 - start_back * math.sin(math.radians(yaw0))
    lines = []
    for i, ((x, y, z, yaw, vis), note) in enumerate(zip(rows, NOTES[name]), 1):
        lines.append(f'  # {i}. {note}')
        lines.append(f'  - {{ id: {i}, x: {x:.2f}, y: {y:.2f}, z: {z:.2f}, yaw_deg: {yaw:g} }}')
    env = max(max(math.hypot(x, y) + outer / 2 for x, y, *_ in rows), math.hypot(sx, sy))
    body = f'''{header}
course:
  source: SkyDreamer track.py      # 참고용 메타 (코드는 gates/start/gate 만 읽는다)
  frame_note: NED→ENU 변환됨 (x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned, yaw_enu = 90° - yaw_ned)
gate:
  inner_size: {inner}
  outer_size: {outer}              # 논문의 주황 게이트 (프레임 폭 {(outer - inner) / 2:g} m)
  thickness: 0.05                  # 물리 프레임 두께. 논문의 t_g(보상용 충돌 볼륨)와는 다른 값이다
# 이륙 지점 = 게이트 1 앞 {start_back:g} m (논문 Table III: "게이트 앞 2~4 m 에서 출발")
start:
  x: {sx:.2f}
  y: {sy:.2f}
  takeoff_z: {z0:.2f}
gates:
{chr(10).join(lines)}
'''
    p = os.path.join(OUT, f'{name}.yaml')
    open(p, 'w').write(body)
    n_vis = sum(1 for r in rows if r[4])
    print(f'wrote {p}  게이트 {len(rows)}개 (실제 {n_vis}, 가상 {len(rows)-n_vis}), envelope {env:.2f} m')

emit('inverted_loop', INVERTED_LOOP, '''# 맵: inverted loop (map:=inverted_loop) — SkyDreamer 논문 Figure 6 / Table IV "Loop"
#
# 출처: track.py 의 INVERTED_LOOP 를 NED→ENU 로 변환한 것. 원본 좌표는 논문 그림에서
# 측정한 재구성이라 ±0.05 m(게이트 1 은 ±0.2 m) 오차가 있다.
# 실제 게이트 2개가 y 축으로 5 m 떨어져 있고 둘 다 +x 로 통과한다.
#
# ⚠️ **step1(min-snap + PX4 위치제어)로는 비행 불가**하다 — split-S(뒤집기)가 들어간다.
#    step2 의 RL/CTBR 용 코스 기하이고, 지금은 rviz/gz 로 배치를 확인하는 용도다.
#
# 논문은 일부 게이트를 "가상 게이트"(기동을 강제하는 경유점, 렌더링 안 함) 로 두는데,
# 여기서는 구분 없이 전부 실제 게이트로 넣었다 — 어차피 기체가 그 개구부를 지나가는 점이라
# 기하가 같고, 아래 주석에 논문에서 가상이던 게이트를 표시해 뒀다.''', 1.5, 2.7)

emit('ladder_loop', LADDER_INVERTED_LOOP, '''# 맵: ladder + inverted loop (map:=ladder_loop) — 논문 Figure 4·5 / Table IV "Ladder loop"
#
# 논문의 시뮬레이션 결과가 나온 코스. track.py 의 LADDER_INVERTED_LOOP 를 NED→ENU 변환.
# inverted_loop 와 실제 게이트는 같고, 게이트 1 에 ladder(360° 좌선회 후 되돌아오기)가 붙는다.
#
# ⚠️ **step1 으로는 비행 불가** (ladder + split-S). 자세한 내용은 inverted_loop.yaml 주석 참고.''', 1.5, 2.7)

emit('big_track', BIG_TRACK, '''# 맵: big oval (map:=big_track) — 논문 Figure 9, zero-shot 일반화 테스트용 큰 코스
#
# track.py 의 BIG_TRACK 을 NED→ENU 변환. 약 23 x 13 m 타원형에 게이트 12개(실제 8, 가상 4).
# 논문이 좌표를 공개하지 않아 Figure 9 에서 측정한 재구성이다.
#
# ⚠️ **step1 으로는 비행 불가** (ladder + 급강하 + split-S).
#    다만 코스가 커서 VIO/인식 테스트용 큰 월드로는 쓸 만하다.''', 1.5, 2.7)
