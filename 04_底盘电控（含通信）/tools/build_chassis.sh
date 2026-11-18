#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$root/build/chassis" "$root/artifacts" "$root/logs"
cmake -S "$root/chassis_driver/basic_framework" -B "$root/build/chassis" -G 'Unix Makefiles' > "$root/logs/chassis-configure.log" 2>&1
cmake --build "$root/build/chassis" -j"${JOBS:-4}" > "$root/logs/chassis-build.log" 2>&1 || {
  tail -n 60 "$root/logs/chassis-build.log" >&2
  exit 1
}
for ext in elf hex bin; do
  cp "$root/build/chassis/basic_framework.$ext" "$root/artifacts/DiPan_Board.$ext"
done
(cd "$root/artifacts" && sha256sum DiPan_Board.{elf,hex,bin} > SHA256SUMS)
python3 "$root/tools/record_build.py" chassis
arm-none-eabi-size "$root/artifacts/DiPan_Board.elf"
echo "Built chassis firmware: $root/artifacts/DiPan_Board.elf (not flashed)"
