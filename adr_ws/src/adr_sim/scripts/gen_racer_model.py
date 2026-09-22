#!/usr/bin/env python3
"""racer_spec.yaml → assets/models/adr_racer/model.sdf + assets/px4/airframes/4030_gz_adr_racer 생성.

    python3 scripts/gen_racer_model.py            # adr_sim 패키지 루트에서
"""
import os
import sys
from math import radians

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)


def fmt(x):
    return f'{x:g}'


def gen_sdf(s):
    L = s['arm_length'] / 2 ** 0.5           # x-config: 각 모터의 |x| = |y| = L/√2
    rz = s['rotor_z']
    scale = s['prop']['diameter_in'] / s['prop']['x500_mesh_in']
    # x500_base 메시 원점 오프셋(스케일 0.846 기준 -0.022,-0.1464,-0.016) 을 새 스케일로 환산
    off = [v * scale / 0.8461538461538461 for v in (-0.022, -0.14638461538461536, -0.016)]
    prop_len = 0.0254 * s['prop']['diameter_in']
    m = s['motor']
    cam = s['camera']
    I = s['inertia']

    # x500 과 동일한 순서/방향: 0 전우 ccw, 1 후좌 ccw, 2 전좌 cw, 3 후우 cw  (FLU)
    rotors = [
        (0, L, -L, 'ccw'), (1, -L, L, 'ccw'), (2, L, L, 'cw'), (3, -L, -L, 'cw'),
    ]

    fm = s.get('frame_mesh')
    links, joints, plugins, arms = [], [], [], []
    for i, x, y, d in rotors:
        if not fm:   # 메시가 있으면 팔은 메시에 포함, 모터 실린더만 추가
            arms.append(f'''      <visual name="arm_{i}_visual">
        <pose>{fmt(x/2)} {fmt(y/2)} 0 0 0 {fmt(radians(45 if x*y > 0 else -45))}</pose>
        <geometry><box><size>{fmt(s['arm_length'])} 0.012 0.006</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>''')
        arms.append(f'''      <visual name="motor_{i}_visual">
        <pose>{fmt(x)} {fmt(y)} {fmt(rz/2)} 0 0 0</pose>
        <geometry><cylinder><radius>0.014</radius><length>{fmt(rz)}</length></cylinder></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>''')
        links.append(f'''    <link name="rotor_{i}">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <velocity_decay/>
      <pose>{fmt(x)} {fmt(y)} {fmt(rz)} 0 0 0</pose>
      <inertial>
        <mass>{fmt(s['prop']['mass'])}</mass>
        <inertia>
          <ixx>1.0e-07</ixx>
          <iyy>{fmt(s['prop']['mass'] * prop_len**2 / 12)}</iyy>
          <izz>{fmt(s['prop']['mass'] * prop_len**2 / 12)}</izz>
        </inertia>
      </inertial>
      <visual name="rotor_{i}_visual">
        <!-- PX4 x500_base 의 13.45" 프롭 메시(meshes/, BSD-3) 를 5" 로 축소 -->
        <pose>{fmt(off[0])} {fmt(off[1])} {fmt(off[2])} 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>{fmt(scale)} {fmt(scale)} {fmt(scale)}</scale>
            <uri>model://{s['name']}/meshes/1345_prop_{d}.stl</uri>
          </mesh>
        </geometry>
        <!-- 검정: 주황 계열이면 게이트와 같은 HSV 대역이라 gate_detector 에 프롭이 잡힌다 -->
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
      </visual>
      <collision name="rotor_{i}_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>{fmt(prop_len)} 0.0063 0.00085</size></box></geometry>
        <surface>
          <contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact>
          <friction><ode/></friction>
        </surface>
      </collision>
    </link>''')
        joints.append(f'''    <joint name="rotor_{i}_joint" type="revolute">
      <parent>base_link</parent>
      <child>rotor_{i}</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit>
        <dynamics><spring_reference>0</spring_reference><spring_stiffness>0</spring_stiffness></dynamics>
      </axis>
    </joint>''')
        plugins.append(f'''    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_{i}_joint</jointName>
      <linkName>rotor_{i}</linkName>
      <turningDirection>{d}</turningDirection>
      <timeConstantUp>{fmt(m['time_constant_up'])}</timeConstantUp>
      <timeConstantDown>{fmt(m['time_constant_down'])}</timeConstantDown>
      <maxRotVelocity>{fmt(m['max_rot_velocity'])}</maxRotVelocity>
      <motorConstant>{fmt(m['motor_constant'])}</motorConstant>
      <momentConstant>{fmt(m['moment_constant'])}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <actuator_number>{i}</actuator_number>
      <rotorDragCoefficient>{fmt(m['rotor_drag_coefficient'])}</rotorDragCoefficient>
      <rollingMomentCoefficient>{fmt(m['rolling_moment_coefficient'])}</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>{fmt(m['rotor_velocity_slowdown_sim'])}</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>''')

    cx, cy, cz = cam['pose_xyz']
    sensors = open(os.path.join(HERE, 'px4_sensors.sdf.inc')).read()
    if fm:
        r, p_, y_ = fm.get('rpy', [0, 0, 0])
        csz = fm.get('collision_size', [0.2, 0.2, 0.05])
        body = f'''      <!-- ===== 외관: meshes/{fm['file']} (scale {fm['scale']}, rpy {r} {p_} {y_}) ===== -->
      <visual name="frame_visual">
        <pose>0 0 0 {fmt(r)} {fmt(p_)} {fmt(y_)}</pose>
        <geometry>
          <mesh>
            <scale>{fmt(fm['scale'])} {fmt(fm['scale'])} {fmt(fm['scale'])}</scale>
            <uri>model://{s['name']}/meshes/{fm['file']}</uri>
          </mesh>
        </geometry>
        <material><ambient>0.12 0.12 0.12 1</ambient><diffuse>0.12 0.12 0.12 1</diffuse><specular>0.3 0.3 0.3 1</specular></material>
      </visual>'''
    else:
        csz = [0.2, 0.2, 0.05]
        body = '''      <!-- ===== 외관 (placeholder). racer_spec.yaml 에 frame_mesh 를 주면 메시로 교체된다 ===== -->
      <visual name="body_visual">
        <pose>0 0 0.012 0 0 0</pose>
        <geometry><box><size>0.11 0.04 0.024</size></box></geometry>
        <material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
      </visual>
      <visual name="battery_visual">
        <pose>0 0 0.036 0 0 0</pose>
        <geometry><box><size>0.075 0.035 0.024</size></box></geometry>
        <material><ambient>0.05 0.05 0.3 1</ambient><diffuse>0.05 0.05 0.3 1</diffuse></material>
      </visual>'''
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<!-- 자동 생성: scripts/gen_racer_model.py (스펙: config/racer_spec.yaml). 직접 수정하지 말 것.
     250급 레이싱 쿼드. 질량 {s['mass']} kg, 모터 대각 {2*s['arm_length']*1000:.0f} mm, 5" 프롭.
     PX4 gz_bridge 규약: 링크 base_link, 센서 imu_sensor/magnetometer_sensor/air_pressure_sensor/navsat_sensor,
     모터 명령 토픽 /<model>/command/motor_speed. 외관 메시는 config/racer_spec.yaml 의 frame_mesh 참고. -->
<sdf version="1.9">
  <model name="{s['name']}">
    <pose>0 0 {fmt(s['spawn_z'])} 0 0 0</pose>
    <self_collide>false</self_collide>
    <static>false</static>
    <link name="base_link">
      <inertial>
        <mass>{fmt(s['mass'])}</mass>
        <inertia>
          <ixx>{fmt(I['ixx'])}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{fmt(I['iyy'])}</iyy><iyz>0</iyz>
          <izz>{fmt(I['izz'])}</izz>
        </inertia>
      </inertial>
      <gravity>true</gravity>
      <velocity_decay/>
{body}
{chr(10).join(arms)}
      <collision name="base_link_collision">
        <pose>0 0 {fmt(csz[2]/2)} 0 0 0</pose>
        <geometry><box><size>{fmt(csz[0])} {fmt(csz[1])} {fmt(csz[2])}</size></box></geometry>
        <surface>
          <contact><ode><min_depth>0.001</min_depth><max_vel>0</max_vel></ode></contact>
          <friction><ode/></friction>
        </surface>
      </collision>
{sensors}    </link>
    <!-- ===== 전방 카메라 (HFOV {cam['hfov']:.2f} rad, {cam['width']}x{cam['height']} @ {cam['rate']} Hz, 상향 {cam['tilt_deg']}°) ===== -->
    <link name="camera_link">
      <pose>{fmt(cx)} {fmt(cy)} {fmt(cz)} 0 {fmt(-radians(cam['tilt_deg']))} 0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia>
      </inertial>
      <visual name="camera_visual">
        <geometry><box><size>0.015 0.02 0.02</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <sensor name="camera" type="camera">
        <gz_frame_id>camera_link</gz_frame_id>
        <topic>{cam['topic']}</topic>
        <camera>
          <horizontal_fov>{fmt(cam['hfov'])}</horizontal_fov>
          <image><width>{cam['width']}</width><height>{cam['height']}</height></image>
          <clip><near>0.05</near><far>100</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>{cam['rate']}</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
{chr(10).join(links)}
{chr(10).join(joints)}
{chr(10).join(plugins)}
    <!-- 진실값 odometry → PX4 gz_bridge 가 vehicle_visual_odometry 로 주입 (mocap 에뮬레이션).
         airframe 4031(EKF2_EV_CTRL=15) 에서만 EKF2 가 사용하고, 4030(GPS) 에서는 무시된다. -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <dimensions>3</dimensions>
      <odom_publish_frequency>100</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
'''


def gen_airframe(s):
    L = s['arm_length'] / 2 ** 0.5
    p = s['px4']
    m = s['motor']
    # FRD 기준 로터 위치 (gz FLU 의 y 부호 반전). 순서는 x500 과 동일.
    rot = [(L, L, 0.05), (-L, -L, 0.05), (L, -L, -0.05), (-L, L, -0.05)]
    ca = '\n'.join(
        f'param set-default CA_ROTOR{i}_PX {fmt(x)}\nparam set-default CA_ROTOR{i}_PY {fmt(y)}\n'
        f'param set-default CA_ROTOR{i}_KM {fmt(km)}' for i, (x, y, km) in enumerate(rot))
    ec = '\n'.join(
        f'param set-default SIM_GZ_EC_FUNC{i+1} {101+i}\nparam set-default SIM_GZ_EC_MIN{i+1} {p["ec_min"]}\n'
        f'param set-default SIM_GZ_EC_MAX{i+1} {fmt(m["max_rot_velocity"])}' for i in range(4))
    return f'''#!/bin/sh
#
# @name Gazebo adr_racer (250-class racing quad)
# @type Quadrotor
#
# 자동 생성: adr_sim/scripts/gen_racer_model.py (스펙: config/racer_spec.yaml). 직접 수정하지 말 것.
# 설치: adr_sim/launch/sim.launch.py 가 실행 때마다 PX4 build/px4_sitl_default/etc/init.d-posix/airframes/ 에 복사한다.

. ${{R}}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${{PX4_SIMULATOR:=gz}}
PX4_GZ_WORLD=${{PX4_GZ_WORLD:=adr_cross}}
PX4_SIM_MODEL=${{PX4_SIM_MODEL:={s['name']}}}

param set-default SIM_GZ_EN 1

# ---- 제어 할당: x-config, 로터 위치(FRD) = 스펙 arm_length/√2 ----
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 4
{ca}

# ---- gz 모터 명령(rad/s). MAX 는 model.sdf 의 maxRotVelocity 와 반드시 동일 ----
{ec}

# ---- 레이싱용 position controller 한계 ----
# 호버 추력(정규화): ω_hover = sqrt(mg/4/kf) 를 [EC_MIN, EC_MAX] 에 선형 매핑한 값
param set-default MPC_THR_HOVER {fmt(p['thr_hover'])}
param set-default MPC_XY_VEL_MAX {fmt(p['xy_vel_max'])}
param set-default MPC_Z_VEL_MAX_UP {fmt(p['z_vel_max_up'])}
param set-default MPC_Z_VEL_MAX_DN {fmt(p['z_vel_max_dn'])}
param set-default MPC_ACC_HOR {fmt(p['acc_hor'])}
param set-default MPC_ACC_UP_MAX {fmt(p['acc_up_max'])}
param set-default MPC_ACC_DOWN_MAX {fmt(p['acc_down_max'])}
param set-default MPC_JERK_AUTO {fmt(p['jerk_auto'])}
param set-default MPC_YAWRAUTO_MAX {fmt(p['yawrate_max'])}
param set-default MPC_TILTMAX_AIR 60
param set-default MC_PITCHRATE_MAX 800
param set-default MC_ROLLRATE_MAX 800
param set-default MC_YAWRATE_MAX 400

# ---- rate 루프 게인 ----
# rc.mc_defaults 의 P=0.15/D=0.003 은 x500 급(~1.5 kg) 기준이다. 이 기체는 관성 2.5e-3 에
# 모터 토크가 커서 각가속도 권한이 훨씬 크고, 그 게인이면 피치축이 리밋사이클에 빠진다
# (호버 정지 상태에서 피치 각속도 p90 0.45 rad/s, 모터가 1673/300 으로 포화).
# SITL 측정: P=0.15 에서 D=0.002 도 발진, D=0.0008 이면 정상. P<=0.12 면 D=0.0024 까지 정상.
# 발진 시작점 P=0.15 의 절반을 취함. 게인을 바꾸면 호버 각속도 p90 부터 확인할 것.
param set-default MC_ROLLRATE_P {fmt(p['rollrate_p'])}
param set-default MC_PITCHRATE_P {fmt(p['pitchrate_p'])}
param set-default MC_ROLLRATE_D {fmt(p['rollrate_d'])}
param set-default MC_PITCHRATE_D {fmt(p['pitchrate_d'])}

# ---- GCS(QGC)/RC 없이 offboard 만으로 운용 ----
param set-default COM_RC_IN_MODE 4     # 스틱 입력 비활성 (SITL 기본 1 = joystick only → manual control lost 실패안전 유발)
param set-default NAV_RCL_ACT 0        # RC loss 실패안전 끔
param set-default COM_RCL_EXCEPT 4     # bit2: offboard 중 RC loss 무시
param set-default NAV_DLL_ACT 0        # 데이터링크(GCS) loss 실패안전 끔 (x500 은 2=hold)
param set-default COM_OBL_ACT 0        # offboard 스트림 끊기면 position hold
param set-default COM_OF_LOSS_T 1.0    # offboard 끊김 판정 시간 [s]
param set-default COM_ARM_WO_GPS 1     # GPS 없이 arm 허용 (EV/mocap 모드)

param set-default EKF2_BCOEF_X 0.0
param set-default EKF2_BCOEF_Y 0.0
param set-default EKF2_MCOEF 0.12
'''


def main():
    spec_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(PKG, 'config', 'racer_spec.yaml')
    with open(spec_path) as f:
        s = yaml.safe_load(f)
    sdf_path = os.path.join(PKG, 'assets', 'models', s['name'], 'model.sdf')
    af_path = os.path.join(PKG, 'assets', 'px4', 'airframes', f"{s['px4']['sys_autostart']}_gz_{s['name']}")
    os.makedirs(os.path.dirname(sdf_path), exist_ok=True)
    os.makedirs(os.path.dirname(af_path), exist_ok=True)
    with open(sdf_path, 'w') as f:
        f.write(gen_sdf(s))
    with open(af_path, 'w') as f:
        f.write(gen_airframe(s))
    os.chmod(af_path, 0o755)
    print('wrote', sdf_path)
    print('wrote', af_path)


if __name__ == '__main__':
    main()
