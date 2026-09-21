#!/usr/bin/env bash
# gz / PX4 실행에 필요한 환경변수. `source adr_ws/src/adr_sim/scripts/setup_env.sh`
#   PX4_DIR   : PX4-Autopilot 클론 위치 (기본 ~/PX4-Autopilot). 레포 외부에 두는 것을 전제로 한다.
#   ADR_SOFT_GL : 1 이면 소프트웨어 렌더링 강제. GPU 가 없는 VM(특히 arm64)에서 필요하다.
ADR_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
export GZ_SIM_RESOURCE_PATH="${ADR_SIM_DIR}/models:${ADR_SIM_DIR}/worlds:${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# 기체에 카메라 센서가 있어 headless(-s) 여도 gz 가 렌더러를 띄운다. GPU 패스스루가 없는 VM 에서는
# ogre2 가 GL3Plus 경로에서 Ogre::UnimplementedException 으로 죽으므로 llvmpipe 로 돌린다.
# GPU 가 정상인 환경에서는 ADR_SOFT_GL=0 으로 꺼서 하드웨어 가속을 쓰는 게 훨씬 빠르다.
if [ "${ADR_SOFT_GL:-1}" = "1" ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
fi

echo "PX4_DIR=$PX4_DIR"
echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-<unset>}"
