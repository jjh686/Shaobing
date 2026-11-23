#!/usr/bin/env bash
# 桌面错误对话框保留串口诊断，运行日志写入当前工程的build/runtime。
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_dir="$script_dir/../build/runtime"
mkdir -p "$runtime_dir/logs" || exit 1
log_file="$(mktemp "$runtime_dir/logs/desktop_XXXXXX.log")" || exit 1
bash "$script_dir/run_live_usb_sp.sh" "$@" >"$log_file" 2>&1
status=$?
if (( status != 0 )); then
    details="$(tail -n 14 -- "$log_file")"
    # [USB_SP 修改 开始] 权限失败和协议失败分别显示，避免一律提示重新烧录。
    message="USB视觉启动未完成（退出码$status）。具体原因见下面的日志。"
    if grep -qF '[USB_PERMISSION]' "$log_file"; then
        message="C板已连接，但当前用户没有串口读写权限。烧录或拔插后临时授权可能失效；先恢复设备权限，无需因此重新烧录。"
    elif grep -qF '[USB_NO_DATA]' "$log_file"; then
        message="串口已打开，但C板尚未回传数据。检查板端USB发送任务、IMU初始化和供电。"
    elif grep -qF '[USB_PROTOCOL]' "$log_file"; then
        message="电脑收到字节，但尚未解析到有效SP状态。请核对板端实际回传格式与校验。"
    fi
    # [USB_SP 修改 结束]
    printf -v message '%s\n\n日志：%s\n\n%s' "$message" "$log_file" "$details"
    if command -v zenity >/dev/null 2>&1; then
        zenity --error --no-markup --title="装甲板实时识别 · USB SP" --width=700 --text="$message" || true
    else
        printf '%s\n' "$message" >&2
    fi
fi
exit "$status"
# [USB_SP 新增 结束]
