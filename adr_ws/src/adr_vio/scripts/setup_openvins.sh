#!/usr/bin/env bash
# OpenVINS(v2.7) 를 adr_ws/src/open_vins 에 받아 온다. 레포에는 넣지 않는다(.gitignore).
#
# 왜 submodule 이 아닌가: open_vins 레포의 ov_data(공개 데이터셋 groundtruth CSV)가 376 MB 인데
# 우리는 안 쓴다. 여기서는 필요한 3개 패키지(ov_core, ov_init, ov_msckf)만 sparse·shallow 로
# 받아 ~15 MB 로 끝낸다. 받고 나면 adr_ws 의 colcon build 가 같이 빌드하므로
# 워크스페이스는 하나, source 도 하나로 유지된다.
#
#   adr_ws/src/adr_vio/scripts/setup_openvins.sh        # 받기 (이미 있으면 그대로 둠)
#   OV_REF=master setup_openvins.sh                     # 다른 버전
set -euo pipefail
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # adr_ws/src
DST="$SRC_DIR/open_vins"
OV_REF="${OV_REF:-v2.7}"

if [ -d "$DST/.git" ]; then
  echo "[setup_openvins] 이미 있음: $DST ($(git -C "$DST" describe --tags --always 2>/dev/null))"
else
  echo "[setup_openvins] clone $OV_REF → $DST (ov_core, ov_init, ov_msckf 만)"
  git clone --depth 1 --branch "$OV_REF" --filter=blob:none --sparse \
    https://github.com/rpng/open_vins.git "$DST"
  git -C "$DST" sparse-checkout set ov_core ov_init ov_msckf
fi

# ov_data / ov_eval 가 어쩌다 받아졌으면 colcon 이 빌드하지 않게 막는다 (ceres 없으면 ov_eval 이 깨진다)
for p in ov_data ov_eval docs; do
  [ -d "$DST/$p" ] && touch "$DST/$p/COLCON_IGNORE"
done

echo "[setup_openvins] 의존 패키지:"
echo "  sudo apt install libeigen3-dev libboost-all-dev libopencv-dev libceres-dev"
echo "[setup_openvins] 빌드:"
echo "  cd $(dirname "$SRC_DIR") && colcon build --symlink-install --packages-select ov_core ov_init ov_msckf adr_vio"
