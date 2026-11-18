#!/usr/bin/env bash
set -eo pipefail
gateway_pc=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
gateway_nav='/home/wangjiaming/桌面/rm_nav_source_20261110(zbh)'
gateway_mode=${1:-preview}
if [[ $# -gt 0 ]]; then shift; fi
case "$gateway_mode" in
  zero) exec python3 "$gateway_pc/zero_test.py" "$@" ;;
  comm|navigation|chassis_navigation)
    [[ $# -ge 1 ]] || { echo '请指定云台 C 板 /dev/serial/by-id/... 路径' >&2; exit 2; }
    gateway_port=$1; shift
    python3 "$gateway_pc/check_port.py" "$gateway_port" >/dev/null ;;
  preview) gateway_port=/dev/ttyACM0 ;;
  *) echo '用法: run.sh preview | zero PORT | comm PORT | chassis_navigation PORT | navigation PORT [launch arguments]' >&2; exit 2 ;;
esac
if [[ "$gateway_mode" == chassis_navigation ]]; then
  exec bash "$gateway_nav/scripts/run_chassis_nav.sh" navigation "$gateway_port" "$@"
fi
if [[ "$gateway_mode" == navigation ]]; then
  exec bash "$gateway_nav/scripts/run_sentry.sh" navigation "$gateway_port" "$@"
fi
source "$gateway_nav/scripts/local_env.sh"
gateway_connect=false
[[ "$gateway_mode" != comm ]] || gateway_connect=true
exec ros2 run rm_chassis_comm chassis_node --ros-args \
  -r __ns:=/chassis -r cmd_vel:=/chassis/manual_body_cmd_vel \
  -p "connect:=$gateway_connect" -p "serial_port:=$gateway_port" \
  -p route:=2 -p enabled:=false -p sp_proxy_enabled:=true \
  -p command_timeout_ms:=150 -p feedback_timeout_ms:=250 -p send_hz:=50 \
  -p max_linear_speed:=0.3 -p max_angular_speed:=0.5 "$@"
