#!/usr/bin/env bash
# Build only; no probe, serial device or flashing command is invoked.
set -euo pipefail
gimbal_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$gimbal_root/artifacts" "$gimbal_root/logs"
make -C "$gimbal_root" all -j"${JOBS:-4}" > "$gimbal_root/logs/gimbal-build.log" 2>&1 || {
  tail -n 60 "$gimbal_root/logs/gimbal-build.log" >&2
  exit 1
}
for ext in elf hex bin; do
  cp "$gimbal_root/build/basic_framework.$ext" "$gimbal_root/artifacts/Gimbal_ChassisLink.$ext"
done
(cd "$gimbal_root/artifacts" && sha256sum Gimbal_ChassisLink.{elf,hex,bin} > GIMBAL_SHA256SUMS)
python3 "$gimbal_root/tools/record_build.py"
arm-none-eabi-size "$gimbal_root/artifacts/Gimbal_ChassisLink.elf"
echo "Built gimbal firmware: $gimbal_root/artifacts/Gimbal_ChassisLink.elf (not flashed)"
