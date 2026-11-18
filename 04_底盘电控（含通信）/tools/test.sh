#!/usr/bin/env bash
set -euo pipefail
root=$(cd -- "$(dirname -- "$0")/.." && pwd)
python3 "$root/tools/test_host.py"
python3 "$root/tools/test_fifo_regression.py"
python3 "$root/tests/test_usb_diagnostic.py"
exec python3 "$root/tests/test_motor_test.py"
