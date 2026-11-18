#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
if [[ $# -lt 1 || -z "$1" ]]; then
  echo '用法：bash tools/zero_test.sh /dev/serial/by-id/实际云台板应用USB设备名' >&2
  exit 2
fi
exec python3 "$root/communication_reference/pc/zero_test.py" --report "$root/logs/zero-test-last.json" "$@"
