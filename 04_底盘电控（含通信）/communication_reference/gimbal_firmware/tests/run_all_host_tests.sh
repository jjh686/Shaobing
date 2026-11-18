#!/usr/bin/env sh
set -eu
tracking_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
sh "$tracking_root/tests/run_gimbal_host_tests.sh"
bash "$tracking_root/tests/driver_hook/run_driver_tests.sh" "$tracking_root"
sh "$tracking_root/tests/run_gimbal_adapter_tests.sh" "$tracking_root"
sh "$tracking_root/tests/run_robot_comm_tests.sh" "$tracking_root"
