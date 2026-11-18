#!/usr/bin/env sh
# [USB_SP 通信整理] 可传入整理前源码目录，以同一组输入核对收发行为。
set -eu
comm_test_dir=$(CDPATH='' cd -- "$(dirname -- "$0")" && pwd)
comm_source_root=${1:-$(CDPATH='' cd -- "$comm_test_dir/.." && pwd)}
comm_test_bin=$(mktemp "${TMPDIR:-/tmp}/robot-comm-test.XXXXXX")
trap 'rm -f "$comm_test_bin"' EXIT HUP INT TERM

compile_and_run() {
    "${CC:-cc}" -std=gnu11 -Wall -Wextra -Werror -g \
        -fsanitize=undefined -fno-sanitize-recover=all \
        -I"$comm_test_dir/communication_stubs" -I"$comm_test_dir/adapter_stubs" \
        -I"$comm_source_root/application/gimbal" \
        -I"$comm_source_root/application/communication" \
        -I"$comm_source_root/modules/nav_conversion" \
        -I"$comm_source_root/modules/imu" -I"$comm_source_root/modules/master_machine" \
        "$comm_test_dir/test_robot_comm.c" \
        "$comm_source_root/modules/master_machine/master_process.c" \
        "$comm_source_root/modules/master_machine/sp_protocol.c" \
        "$comm_source_root/application/gimbal/gimbal_math.c" \
        "$comm_source_root/modules/nav_conversion/nav_usb_mux.c" \
        "$comm_source_root/modules/nav_conversion/nav_can.c" \
        "$comm_source_root/modules/nav_conversion/protocol.c" \
        "$@" -lm -o "$comm_test_bin"
    "$comm_test_bin"
}

if [ -f "$comm_source_root/application/communication/robot_comm.c" ]; then
    compile_and_run "$comm_source_root/application/communication/robot_comm.c"
    compile_and_run "$comm_source_root/application/communication/robot_comm.c" -DROBOT_COMM_TEST_DIRECT
else
    compile_and_run
fi
