#!/usr/bin/env bash
set -euo pipefail
gateway_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$gateway_root"
mkdir -p build/tests
common_sources=(common/{protocol,mcu_link,gimbal_protocol,nav_can,nav_usb_mux,nav_gateway}.c)
test_flags=(-std=c11 -Wall -Wextra -Werror -g -fsanitize=undefined -fno-sanitize-recover=all -Icommon)
cc "${test_flags[@]}" tests/test_protocol.c "${common_sources[@]}" -lm -o build/tests/protocol
build/tests/protocol
cc "${test_flags[@]}" -Itests/stubs tests/test_can_hal.c "${common_sources[@]}" -lm -o build/tests/can_hal
build/tests/can_hal
cc -std=c11 -Wall -Wextra -Werror -shared -fPIC -Icommon common/protocol.c -lm -o build/tests/protocol.so
python3 tests/test_pc.py
for variant in locked enabled; do
  defines=()
  if [[ $variant == enabled ]]; then defines=(-DNAV_RELAY_ALLOW_MOTION=1 -DNAV_AXES_VERIFIED=1); fi
  cc "${test_flags[@]}" "${defines[@]}" -Itests/stubs \
    -Ichassis_firmware/modules/nav_relay -Igimbal_firmware/application/communication \
    tests/test_chain.c chassis_firmware/modules/nav_relay/nav_can_adapter.c \
    gimbal_firmware/application/communication/nav_gateway_port.c "${common_sources[@]}" \
    -lm -o "build/tests/chain_$variant"
  "build/tests/chain_$variant"
done
sh gimbal_firmware/tests/run_all_host_tests.sh
