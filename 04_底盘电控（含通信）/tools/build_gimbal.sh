#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$root/build/gimbal" "$root/artifacts" "$root/logs"
# Use the Makefile: the bundled CMake file contains a linker option unsupported
# by Ubuntu 22.04's Arm toolchain. Keep BUILD_DIR relative to the firmware root.
make -C "$root/communication_reference/gimbal_firmware" \
  BUILD_DIR=../../build/gimbal -j"${JOBS:-4}" > "$root/logs/gimbal-build.log" 2>&1 || {
  tail -n 60 "$root/logs/gimbal-build.log" >&2
  exit 1
}
for ext in elf hex bin; do
  cp "$root/build/gimbal/basic_framework.$ext" "$root/artifacts/Gimbal_USB_CAN1.$ext"
done
(cd "$root/artifacts" && sha256sum Gimbal_USB_CAN1.{elf,hex,bin} > GIMBAL_SHA256SUMS)
python3 "$root/tools/record_build.py" gimbal
arm-none-eabi-size "$root/artifacts/Gimbal_USB_CAN1.elf"
echo "Built gimbal firmware: $root/artifacts/Gimbal_USB_CAN1.elf (not flashed)"
