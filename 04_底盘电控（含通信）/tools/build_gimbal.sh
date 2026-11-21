#!/usr/bin/env bash
# Compatibility entry: all gimbal sources and outputs belong to 05_云台.
set -euo pipefail
chassis_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec bash "$chassis_root/../05_云台/tools/build.sh" "$@"
