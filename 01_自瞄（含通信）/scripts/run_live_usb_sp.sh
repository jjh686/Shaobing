#!/usr/bin/env bash
# 桌面实际入口：编译、模型和运行输出均使用当前视觉工程。
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="$project_dir/build/runtime"
mkdir -p "$runtime_dir/logs"
cd -- "$runtime_dir"
exec 9>"$runtime_dir/usb_vision.lock"
if ! flock -n 9; then
    echo "USB视觉程序已经运行，请查看现有窗口。" >&2
    exit 1
fi
binary="$project_dir/build/auto_aim_test"
config="$project_dir/configs/live_usb_sp.yaml"
if [[ ! -f "$project_dir/build/CMakeCache.txt" ]]; then
    cmake -S "$project_dir" -B "$project_dir/build"
fi
# 增量编译，让此目录的源码修改在下次双击时生效。
cmake --build "$project_dir/build" --target auto_aim_test -j2
echo "视觉工程：$project_dir"
echo "运行输出：$runtime_dir"
args=(--camera=true --show-control=true --config-path="$config")
if [[ "${1:-}" == --preview ]]; then
    shift
    args+=(--usb=false --decision-status=false)
    echo "仅识别预览：不连接C板；退出按q。"
else
    args+=(--usb=true --decision-status=true)
    echo "USB SP视觉联动：等待C板43字节SP回传；收到后开启相机识别与29字节发送。"
    echo "窗口显示实际RX/TX计数和板端模式；退出按q。此入口不发送开火请求。"
fi
exec "$binary" "${args[@]}" "$@"
