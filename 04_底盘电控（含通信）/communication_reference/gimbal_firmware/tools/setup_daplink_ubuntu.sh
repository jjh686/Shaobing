#!/usr/bin/env bash
# 在 Ubuntu 主机上安装本工程的 DAPLink 编译、调试工具及 USB 权限规则。
set -euo pipefail

dap_project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--keep-open" ]]; then
    trap 'dap_setup_status=$?; if [[ $dap_setup_status == 0 ]]; then printf "\nDAPLink 工具安装完成。\n"; else printf "\n安装失败，退出码：%s。请查看上方错误。\n" "$dap_setup_status"; fi; read -r -p "按回车关闭此窗口……" || true; exit "$dap_setup_status"' EXIT
fi

printf '安装 DAPLink 工具；sudo 密码输入时不会显示字符。\n'
# 本次安装单独使用 Ubuntu 22.04 官方源，避免无关第三方源的错误阻塞安装。
# APT 选项只对当前命令生效，不修改系统已有的软件源配置。
dap_apt_options=(
    -o "Dir::Etc::sourcelist=$dap_project_dir/tools/ubuntu-jammy.list"
    -o 'Dir::Etc::sourceparts=-'
)
sudo apt-get "${dap_apt_options[@]}" update
sudo apt-get "${dap_apt_options[@]}" install -y --no-install-recommends \
    gcc-arm-none-eabi libnewlib-arm-none-eabi gdb-multiarch openocd

sudo install -m 644 "$dap_project_dir/tools/60-puso-daplink.rules" \
    /etc/udev/rules.d/60-puso-daplink.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=usb \
    --attr-match=idVendor=1234 --attr-match=idProduct=5678
sudo udevadm settle --timeout=10

arm-none-eabi-gcc --version
gdb-multiarch --version
openocd --version
printf '\nDAPLink 工具安装完成。\n'
