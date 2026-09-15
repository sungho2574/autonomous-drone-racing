#!/usr/bin/env bash
# adr_sim/px4/airframes/* 를 PX4 ROMFS 에 심볼릭 링크하고 CMakeLists 에 등록한다.
# 이후 PX4 를 다시 빌드해야 적용된다:  cd $PX4_DIR && make px4_sitl
set -euo pipefail
ADR_SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKE="$AF_DIR/CMakeLists.txt"

[ -d "$AF_DIR" ] || { echo "PX4 airframes 디렉터리를 찾을 수 없음: $AF_DIR (PX4_DIR 확인)"; exit 1; }

for f in "$ADR_SIM_DIR"/px4/airframes/*; do
  name="$(basename "$f")"
  ln -sfn "$f" "$AF_DIR/$name"
  echo "link  $AF_DIR/$name -> $f"
  if ! grep -q "^\s*$name\s*$" "$CMAKE"; then
    # px4_add_romfs_files( 목록의 마지막 gz 항목 뒤에 삽입
    python3 - "$CMAKE" "$name" <<'PY'
import re, sys
path, name = sys.argv[1], sys.argv[2]
s = open(path).read()
start = s.find('px4_add_romfs_files(')
end = s.find('\n)', start)
if start < 0 or end < 0:
    sys.exit(f'{path}: px4_add_romfs_files( 블록을 찾지 못함')
s = s[:end] + f'\n\t{name}' + s[end:]
open(path, 'w').write(s)
PY
    echo "cmake $name 등록"
  fi
done
echo "완료. PX4 재빌드:  cd $PX4_DIR && make px4_sitl"
