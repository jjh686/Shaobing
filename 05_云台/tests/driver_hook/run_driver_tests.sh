#!/usr/bin/env bash
set -euo pipefail
test_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
source_root="${1:?Pass the firmware source root}"
driver_dir="${2:-$source_root/modules/motor/DJImotor}"
test_binary="$(mktemp /tmp/tracking-dji-tests.XXXXXX)"
trap 'rm -f "$test_binary"' EXIT
cc -std=gnu11 -Wall -Wextra -Werror -Wno-sign-compare -g \
    -I"$test_dir/stubs" -I"$driver_dir" \
    -I"$source_root/modules/algorithm" -I"$source_root/modules/motor" \
    -I"$source_root/modules/daemon" -I"$source_root/application/gimbal" \
    "$test_dir/test_dji_speed_hook.c" "$source_root/modules/algorithm/controller.c" \
    -lm -o "$test_binary"
"$test_binary"
