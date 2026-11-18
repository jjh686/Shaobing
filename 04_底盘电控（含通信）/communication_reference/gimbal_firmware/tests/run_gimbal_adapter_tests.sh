#!/usr/bin/env sh
set -eu
adapter_test_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
adapter_root=${1:?Pass the isolated firmware source root}
adapter_test_bin=$(mktemp "${TMPDIR:-/tmp}/gimbal-adapter-test.XXXXXX")
trap 'rm -f "$adapter_test_bin"' EXIT HUP INT TERM
for installation in 0 1; do
"${CC:-cc}" -std=gnu11 -Wall -Wextra -Werror -g -DGIMBAL_INSTALLATION_CONFIRMED="$installation" \
    -fsanitize=undefined -fno-sanitize-recover=all \
    -I"$adapter_test_dir/adapter_stubs" \
    -I"$adapter_root/application/gimbal" -I"$adapter_root/application" \
    -I"$adapter_root/modules/motor/DJImotor" -I"$adapter_root/modules/motor" \
    -I"$adapter_root/modules/algorithm" -I"$adapter_root/modules/daemon" \
    -I"$adapter_root/modules/imu" -I"$adapter_root/modules/master_machine" \
    -I"$adapter_root/modules/message_center" \
    "$adapter_test_dir/test_gimbal_adapter.c" \
    "$adapter_root/application/gimbal/gimbal_math.c" \
    "$adapter_root/application/gimbal/gimbal_speed_pi.c" \
    "$adapter_root/modules/algorithm/controller.c" \
    -lm -o "$adapter_test_bin"
"$adapter_test_bin"
done
