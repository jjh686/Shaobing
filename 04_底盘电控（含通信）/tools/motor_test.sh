#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
if [[ $# -lt 1 || -z "$1" ]]; then
  echo '用法：bash tools/motor_test.sh /dev/serial/by-id/云台USB --wheels-raised --direction forward --duration 1' >&2
  exit 2
fi
exec python3 "$root/communication_reference/pc/motor_test.py" --report "$root/logs/motor-test-last.json" "$@"
