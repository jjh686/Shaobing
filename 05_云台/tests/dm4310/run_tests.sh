#!/usr/bin/env bash
set -euo pipefail
test_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
source_root="${1:-$(cd -- "$test_dir/../.." && pwd)}"
test_binary="$(mktemp /tmp/dm4310-pitch-tests.XXXXXX)"
trap 'rm -f "$test_binary"' EXIT
"${CC:-cc}" -std=gnu11 -Wall -Wextra -Werror -g \
    -fsanitize=undefined -fno-sanitize-recover=all \
    -I"$source_root/tests/driver_hook/stubs" \
    -I"$source_root/modules/motor/DMmotor" -I"$source_root/modules/motor/DJImotor" \
    -I"$source_root/modules/algorithm" -I"$source_root/modules/motor" \
    -I"$source_root/modules/daemon" -I"$source_root/application/gimbal" \
    "$test_dir/test_dm4310_pitch.c" "$source_root/modules/algorithm/controller.c" \
    -lm -o "$test_binary"
"$test_binary"
