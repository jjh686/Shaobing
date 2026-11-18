#!/usr/bin/env bash
# [USB_SP 新增 开始] 桌面实际入口；源码与配置归视觉工程，运行输出归下载/codex。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="/home/wangjiaming/下载/codex/视觉_USB_SP_2026-10-28/runtime"
mkdir -p "$runtime_dir/logs"
cd -- "$runtime_dir"
exec 9>"$runtime_dir/usb_vision.lock"
if ! flock -n 9; then
    echo "USB视觉程序已经运行，请查看现有窗口。" >&2
    exit 1
fi
binary="$project_dir/build/auto_aim_test"
config="$project_dir/configs/live_usb_sp.yaml"
if [[ ! -x "$binary" ]]; then
    echo "缺少可执行程序，请先运行：cmake --build $project_dir/build --target auto_aim_test -j2" >&2
    exit 1
fi
echo "USB SP视觉联动：等待C板43字节SP回传；收到后开启相机识别与29字节发送。"
echo "窗口显示实际RX/TX计数和板端模式；退出按q。此入口不发送开火请求。"
exec "$binary" --camera=true --usb=true --show-control=true --config-path="$config" "$@"
# [USB_SP 新增 结束]
