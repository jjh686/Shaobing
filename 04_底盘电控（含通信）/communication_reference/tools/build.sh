#!/usr/bin/env bash
set -euo pipefail
gateway_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$gateway_root"
python3 tools/sync_common.py
mkdir -p build/gimbal artifacts
make -C gimbal_firmware BUILD_DIR=../build/gimbal -j"${JOBS:-4}" > build-gimbal.log 2>&1
cmake -S chassis_firmware -B build/chassis -G 'Unix Makefiles' > build-chassis-configure.log 2>&1
cmake --build build/chassis -j"${JOBS:-4}" > build-chassis.log 2>&1
for ext in elf bin hex; do
  cp "build/gimbal/basic_framework.$ext" "artifacts/gimbal_nav_converter.$ext"
  cp "build/chassis/basic_framework.$ext" "artifacts/chassis_nav_can.$ext"
done
(cd artifacts && sha256sum ./*.bin ./*.hex ./*.elf > SHA256SUMS)
python3 tools/write_build_config.py
bash tools/package_gimbal.sh
arm-none-eabi-size artifacts/*.elf
