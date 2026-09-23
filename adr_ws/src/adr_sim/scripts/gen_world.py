#!/usr/bin/env python3
"""코스·배경 설정 → gz 월드와 모델 생성.

  adr_bringup/config/gates.yaml  → assets/worlds/adr_cross.sdf, assets/models/adr_gate/model.sdf
  adr_sim/config/scene.yaml      → assets/models/adr_ground/ (텍스처 바닥) + 월드 안의 기둥·상자

    python3 scripts/gen_world.py [gates.yaml] [--out worlds/adr_cross.sdf] [--name adr_cross]

월드는 PX4 standalone(gz 를 ROS launch 가 직접 띄우는) 모드용이라 PX4 server.config 가
넣어주는 시스템 플러그인(IMU/자기/기압/NavSat/Sensors 등)을 직접 포함한다.
"""
import argparse
import os
import random
import struct
import zlib
from math import cos, radians, sin

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
DEFAULT_GATES = os.path.normpath(os.path.join(PKG, '..', 'adr_bringup', 'config', 'gates.yaml'))

WORLD_HEAD = '''<?xml version="1.0" encoding="UTF-8"?>
<!-- 자동 생성: adr_sim/scripts/gen_world.py (입력: adr_bringup/config/gates.yaml). 직접 수정하지 말 것. -->
<sdf version="1.9">
  <world name="{name}">
    <!-- PX4 standalone 모드용 시스템 플러그인 (PX4 gz_bridge/server.config 와 동일 구성) -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>488</elevation>
    </spherical_coordinates>

    <light name="sun" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation><range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic></attenuation>
    </light>

    <!-- 먼 배경용 무한 평면. 텍스처 바닥(adr_ground)이 그 위 z=0 에 깔린다 -->
    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.02 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.55 0.6 0.55 1</ambient>
            <diffuse>0.55 0.6 0.55 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>
'''

WORLD_TAIL = '''  </world>
</sdf>
'''

GATE_MAT = '''        <material>
          <ambient>1.0 0.45 0.0 1</ambient>
          <diffuse>1.0 0.45 0.0 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>'''


def gen_gate_model(gate: dict) -> str:
    """gates.yaml 의 gate: {inner_size, outer_size, thickness} → adr_gate model.sdf (box 4개, static)."""
    inner, outer, thick = float(gate['inner_size']), float(gate['outer_size']), float(gate['thickness'])
    w = (outer - inner) / 2
    c = inner / 2 + w / 2

    def bar(name, pose, size):
        return f'''    <link name="{name}">
      <pose>{pose}</pose>
      <visual name="{name}_visual">
        <geometry><box><size>{size}</size></box></geometry>
{GATE_MAT}
      </visual>
      <collision name="{name}_collision">
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
    </link>
'''
    body = ''.join([
        bar('left', f'0 {c:g} 0 0 0 0', f'{thick:g} {w:g} {outer:g}'),
        bar('right', f'0 {-c:g} 0 0 0 0', f'{thick:g} {w:g} {outer:g}'),
        bar('top', f'0 0 {c:g} 0 0 0', f'{thick:g} {inner:g} {w:g}'),
        bar('bottom', f'0 0 {-c:g} 0 0 0', f'{thick:g} {inner:g} {w:g}'),
    ])
    return f'''<?xml version="1.0"?>
<!-- 자동 생성: adr_sim/scripts/gen_world.py (입력: adr_bringup/config/gates.yaml 의 gate:). 직접 수정하지 말 것.
     정사각 레이싱 게이트. 내부 {inner:g} m, 외부 {outer:g} m (프레임 폭 {w:g} m), 두께 {thick:g} m.
     원점 = 개구부 중심, +x = 통과 방향(법선). static 이라 공중에 고정된다(다리 없음). -->
<sdf version="1.9">
  <model name="adr_gate">
    <static>true</static>
{body}  </model>
</sdf>
'''


# ===================== 배경: 텍스처 바닥 + 장애물 (scene.yaml) =====================
# 목적은 VIO(KLT)가 추적할 코너를 만드는 것. 무늬 없는 바닥에서는 특징점이 안 잡혀 바로 발산한다.

def write_png(path: str, rows: list):
    """의존성 없이 RGB8 PNG 쓰기. rows[y] = bytes(길이 3*width)."""
    w, h = len(rows[0]) // 3, len(rows)
    raw = b''.join(b'\x00' + r for r in rows)     # 각 행 앞에 filter type 0

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack('>I', len(data)) + tag + data
                + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff))

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n'
                + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 9))
                + chunk(b'IEND', b''))


def gen_ground_texture(path: str, g: dict, rng: random.Random):
    """랜덤 밝기 타일 + 경계선. 평평한 타일이라 PNG 가 잘 압축되고 코너는 많다."""
    n, t = int(g['texture_px']), int(g['tile_px'])
    lo, hi = g['gray']
    tint, grout = int(g['tint']), float(g['grout'])
    def tile_color():
        base = rng.randint(lo, hi)      # 타일마다 밝기 하나를 뽑고
        return tuple(min(255, max(0, base + rng.randint(-tint, tint))) for _ in range(3))   # 채널별로 살짝만 틀어 준다

    tiles = [[tile_color() for _ in range(n // t + 1)] for _ in range(n // t + 1)]
    rows = []
    for y in range(n):
        row = bytearray()
        edge_y = (y % t == 0)
        for x in range(n):
            c = tiles[y // t][x // t]
            if edge_y or x % t == 0:
                c = tuple(int(v * grout) for v in c)
            row += bytes(c)
        rows.append(bytes(row))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    write_png(path, rows)
    return n, t


GROUND_SDF = '''<?xml version="1.0"?>
<!-- 자동 생성: adr_sim/scripts/gen_world.py (입력: adr_sim/config/scene.yaml). 직접 수정하지 말 것.
     VIO 특징점용 텍스처 바닥. {size:g} x {size:g} m, 윗면 z=0. -->
<sdf version="1.9">
  <model name="adr_ground">
    <static>true</static>
    <pose>0 0 {zoff:g} 0 0 0</pose>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{size:g} {size:g} {th:g}</size></box></geometry>
        <material>
          <ambient>0.8 0.8 0.8 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://adr_ground/materials/textures/ground.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>0.9</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>{size:g} {size:g} {th:g}</size></box></geometry>
        <surface><friction><ode/></friction><contact/></surface>
      </collision>
    </link>
  </model>
</sdf>
'''

GROUND_CONFIG = '''<?xml version="1.0"?>
<model>
  <name>adr_ground</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>sungho</name><email>sungho2574@gmail.com</email></author>
  <description>VIO 특징점용 텍스처 바닥 (자동 생성)</description>
</model>
'''

OBSTACLE_SDF = '''    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
            <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>{geom}</geometry>
        </collision>
      </link>
    </model>
'''


def gen_ground_model(g: dict) -> str:
    """텍스처를 입힌 바닥 상자. 윗면이 정확히 z=0 이라 기체가 그 위에 선다."""
    size, th = float(g['size']), float(g['thickness'])
    return GROUND_SDF.format(size=size, th=th, zoff=-th / 2)


def gen_obstacles(scene: dict, rng: random.Random) -> str:
    """코스 바깥에 기둥·상자를 두른다. 세로 구조물이 VIO 의 시차(parallax)에 제일 좋다."""
    tau = 2 * 3.141592653589793
    out = ['\n    <!-- ===== 배경 장애물 (scene.yaml) — VIO 특징점용. 코스 바깥에만 둔다 ===== -->\n']
    p = scene['poles']
    n = int(p['count'])
    rad_p = p['thickness'] / 2
    for i in range(n):
        ang = tau * i / n + rng.uniform(-0.15, 0.15)
        rad = rng.uniform(*p['radius'])
        hgt = rng.uniform(*p['height'])
        shade = rng.uniform(0.25, 0.75)
        out.append(OBSTACLE_SDF.format(
            name=f'pole_{i}',
            pose=f'{rad * cos(ang):.3f} {rad * sin(ang):.3f} {hgt / 2:.3f} 0 0 0',
            geom=f'<cylinder><radius>{rad_p:g}</radius><length>{hgt:.3f}</length></cylinder>',
            r=shade, g=shade * rng.uniform(0.8, 1.2), b=shade * rng.uniform(0.8, 1.2)))
    b = scene['boxes']
    for i in range(int(b['count'])):
        ang = rng.uniform(0, tau)
        rad = rng.uniform(*b['radius'])
        sx, sy = rng.uniform(*b['size']), rng.uniform(*b['size'])
        hgt = rng.uniform(*b['height'])
        shade = rng.uniform(0.2, 0.8)
        out.append(OBSTACLE_SDF.format(
            name=f'box_{i}',
            pose=f'{rad * cos(ang):.3f} {rad * sin(ang):.3f} {hgt / 2:.3f} 0 0 {rng.uniform(0, 1.57):.3f}',
            geom=f'<box><size>{sx:.3f} {sy:.3f} {hgt:.3f}</size></box>',
            r=shade * rng.uniform(0.8, 1.2), g=shade, b=shade * rng.uniform(0.8, 1.2)))
    out.append('''
    <!-- 텍스처 바닥 (adr_ground): 윗면이 z=0 -->
    <include>
      <uri>model://adr_ground</uri>
      <name>adr_ground</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
''')
    return ''.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gates', nargs='?', default=DEFAULT_GATES)
    ap.add_argument('--out', default=os.path.join(PKG, 'assets', 'worlds', 'adr_cross.sdf'))
    ap.add_argument('--name', default='adr_cross')
    ap.add_argument('--racer-model', default='adr_racer')
    ap.add_argument('--scene', default=os.path.join(PKG, 'config', 'scene.yaml'))
    args = ap.parse_args()

    with open(args.gates) as f:
        d = yaml.safe_load(f)
    with open(args.scene) as f:
        scene = yaml.safe_load(f)
    rng = random.Random(scene.get('seed', 0))

    body = [gen_obstacles(scene, rng)]
    body += [f'\n    <!-- ===== 게이트 {len(d["gates"])}개 (gates.yaml) — 원점 = 개구부 중심, yaw = 통과 방향 ===== -->\n']
    for g in d['gates']:
        body.append(f'''    <include>
      <uri>model://adr_gate</uri>
      <name>gate_{g['id']}</name>
      <pose>{g['x']} {g['y']} {g['z']} 0 0 {radians(g['yaw_deg']):.6f}</pose>
    </include>
''')
    s = d['start']
    body.append(f'''
    <!-- ===== 드론 (start 지점 바닥, PX4 는 PX4_GZ_MODEL_NAME={args.racer_model} 로 붙는다) ===== -->
    <include>
      <uri>model://{args.racer_model}</uri>
      <name>{args.racer_model}</name>
      <pose>{s['x']} {s['y']} 0.05 0 0 0</pose>
    </include>
''')
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(WORLD_HEAD.format(name=args.name) + ''.join(body) + WORLD_TAIL)
    print('wrote', args.out)

    gate_sdf = os.path.join(PKG, 'assets', 'models', 'adr_gate', 'model.sdf')
    with open(gate_sdf, 'w') as f:
        f.write(gen_gate_model(d['gate']))
    print('wrote', gate_sdf)

    ground_dir = os.path.join(PKG, 'assets', 'models', 'adr_ground')
    tex = os.path.join(ground_dir, 'materials', 'textures', 'ground.png')
    n, t = gen_ground_texture(tex, scene['ground'], rng)
    grid = float(scene['ground']['size']) / (n / t)
    print(f'wrote {tex}  ({n}x{n} px, 타일 {t} px = 바닥 {grid:.2f} m 격자, '
          f'{os.path.getsize(tex) // 1024} KB)')
    with open(os.path.join(ground_dir, 'model.sdf'), 'w') as f:
        f.write(gen_ground_model(scene['ground']))
    with open(os.path.join(ground_dir, 'model.config'), 'w') as f:
        f.write(GROUND_CONFIG)
    print('wrote', os.path.join(ground_dir, 'model.sdf'))
    print(f"  배경 장애물: 기둥 {scene['poles']['count']}개, 상자 {scene['boxes']['count']}개 "
          f"(seed {scene.get('seed', 0)})")


if __name__ == '__main__':
    main()
