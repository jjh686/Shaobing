#!/usr/bin/env sh
set -eu
controller_root=$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)
controller_test_bin=$(mktemp "${TMPDIR:-/tmp}/gimbal-speed-pi-test.XXXXXX")
trap 'rm -f "$controller_test_bin"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
    -fsanitize=undefined -fno-sanitize-recover=all \
    -I"$controller_root/application/gimbal" \
    "$controller_root/application/gimbal/gimbal_speed_pi.c" \
    "$controller_root/tests/gimbal_speed_pi_test.c" \
    -lm -o "$controller_test_bin"
"$controller_test_bin"
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -pedantic \
    -fsanitize=undefined -fno-sanitize-recover=all \
    -I"$controller_root/application/gimbal" \
    "$controller_root/application/gimbal/gimbal_pitch_control.c" \
    "$controller_root/application/gimbal/gimbal_speed_pi.c" \
    "$controller_root/tests/gimbal_pitch_control_test.c" \
    -lm -o "$controller_test_bin"
"$controller_test_bin"
