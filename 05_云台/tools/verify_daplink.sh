#!/usr/bin/env bash
# 编译固件、检查调试符号并识别 SWD 目标；不烧录、不复位。
set -euo pipefail

dap_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd -- "$dap_project_dir"

for dap_tool in make arm-none-eabi-gcc gdb-multiarch openocd timeout; do
    if ! command -v "$dap_tool" >/dev/null 2>&1; then
        printf '缺少工具：%s。请先完成 tools/setup_daplink_ubuntu.sh 的安装。\n' "$dap_tool" >&2
        exit 1
    fi
done

make -j16 all
gdb-multiarch --batch build/basic_framework.elf \
    -ex 'info address main' -ex 'info line main'
timeout 20s make probe_dap
