#!/usr/bin/env python3
"""adr_bringup/config/gates.yaml → worlds/adr_cross.sdf 생성.

    python3 scripts/gen_world.py [gates.yaml] [--out worlds/adr_cross.sdf] [--name adr_cross]

월드는 PX4 standalone(gz 를 ROS launch 가 직접 띄우는) 모드용이라 PX4 server.config 가
넣어주는 시스템 플러그인(IMU/자기/기압/NavSat/Sensors 등)을 직접 포함한다.
"""
import argparse
import os
from math import radians

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

    <model name="ground_plane">
      <static>true</static>
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('gates', nargs='?', default=DEFAULT_GATES)
    ap.add_argument('--out', default=os.path.join(PKG, 'worlds', 'adr_cross.sdf'))
    ap.add_argument('--name', default='adr_cross')
    ap.add_argument('--racer-model', default='adr_racer')
    args = ap.parse_args()

    with open(args.gates) as f:
        d = yaml.safe_load(f)

    body = [f'\n    <!-- ===== 게이트 {len(d["gates"])}개 (gates.yaml) — 원점 = 개구부 중심, yaw = 통과 방향 ===== -->\n']
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


if __name__ == '__main__':
    main()
