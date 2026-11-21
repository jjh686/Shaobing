#!/usr/bin/env python3
"""Identify a supported gimbal application USB; STOP ACK verifies its protocol."""
from pathlib import Path
import os
import stat
import sys


SYS_TTY = Path('/sys/class/tty')
# 05_云台 retains its vision descriptor while also forwarding chassis traffic.
# The descriptor identifies a candidate port, not the firmware/protocol version.
GIMBAL_USB_IDENTITIES = frozenset({
    ('0483', '00ca', 'Vision Comm port'),
    ('0483', '00ca', 'RM Nav Convert SP'),
})


def check(path):
    if not os.fspath(path).strip():
        raise ValueError('串口参数为空：$PORT 可能尚未在当前终端设置。请先设置 PORT，或直接传入 /dev/serial/by-id/... 设备路径')
    device = Path(path).resolve(strict=True)
    if device.is_dir():
        raise ValueError('串口参数指向目录: ' + str(device) + '；请传入 /dev/serial/by-id/... 设备路径')
    if not stat.S_ISCHR(device.stat().st_mode):
        raise ValueError('不是串口字符设备: ' + str(device))
    if not os.access(device, os.R_OK | os.W_OK):
        raise ValueError('设备不可读写: ' + str(device))
    sysdev = SYS_TTY / device.name / 'device'
    if not sysdev.exists():
        raise ValueError('需要真实 C 板 USB 串口: ' + str(device))
    for parent in sysdev.resolve().parents:
        if (parent/'idVendor').exists():
            vid = (parent/'idVendor').read_text().strip()
            pid = (parent/'idProduct').read_text().strip()
            product = (parent/'product').read_text().strip() if (parent/'product').exists() else ''
            if (vid, pid, product) not in GIMBAL_USB_IDENTITIES:
                raise ValueError(f'USB 设备不匹配: {vid}:{pid} {product}；需要云台应用串口 '
                                 '(0483:00ca，Vision Comm port 或 RM Nav Convert SP)，不要选择 CMSIS-DAP')
            return str(device)
    raise ValueError('未找到 USB 产品信息')


if __name__ == '__main__':
    try:
        print(check(sys.argv[1]))
    except (OSError, ValueError, IndexError) as exc:
        sys.exit(str(exc))
