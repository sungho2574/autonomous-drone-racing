#!/usr/bin/env bash
# gz / PX4 실행에 필요한 환경변수. `source adr_ws/src/adr_sim/scripts/setup_env.sh`
#   PX4_DIR : PX4-Autopilot 클론 위치 (기본 ~/PX4-Autopilot). 레포 외부에 두는 것을 전제로 한다.
ADR_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export GZ_SIM_RESOURCE_PATH="${ADR_SIM_DIR}/models:${ADR_SIM_DIR}/worlds:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "PX4_DIR=$PX4_DIR"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
