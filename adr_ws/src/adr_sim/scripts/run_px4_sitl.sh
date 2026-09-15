#!/usr/bin/env bash
# PX4 SITL 을 standalone 모드로 띄워, 이미 실행 중인 gz 월드(adr_cross)의 adr_racer 모델에 붙인다.
#   ./run_px4_sitl.sh          # airframe 4030 (GPS/기압 시뮬 EKF2)
#   ./run_px4_sitl.sh --ev     # airframe 4031 (외부 비전 = OdometryPublisher 진실값, mocap 에뮬레이션)
# 선행: ros2 launch adr_bringup sim.launch.py  (gz 서버 + adr_cross 월드)
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/setup_env.sh" >/dev/null
AUTOSTART=4030
[ "${1:-}" = "--ev" ] && AUTOSTART=4031
BUILD="$PX4_DIR/build/px4_sitl_default"
[ -x "$BUILD/bin/px4" ] || { echo "px4 바이너리 없음: $BUILD/bin/px4  (cd $PX4_DIR && make px4_sitl)"; exit 1; }

cd "$BUILD"   # rootfs 로그/eeprom 이 여기에 생김
export PX4_GZ_STANDALONE=1
export PX4_GZ_MODEL_NAME=adr_racer
export PX4_GZ_WORLD=adr_cross
export PX4_SYS_AUTOSTART=$AUTOSTART
export PX4_SIM_MODEL=gz_adr_racer
export PX4_UXRCE_DDS_PORT=${PX4_UXRCE_DDS_PORT:-8888}
echo "PX4 SITL: autostart=$AUTOSTART model=$PX4_GZ_MODEL_NAME world=$PX4_GZ_WORLD"
exec ./bin/px4 ./etc -s etc/init.d-posix/rcS
