#!/usr/bin/env python3
"""Package built USB/CAN1 + CAN2 firmware in each project's release directory."""
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from zoneinfo import ZoneInfo


root = Path(__file__).resolve().parents[1]
metadata = json.loads((root / 'artifacts/build_config.json').read_text())
stamp = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d-%H%M%S')
release_name = '双板通信-' + stamp
profiles = (
    ('chassis', 'DiPan_Board', root / 'chassis_driver/basic_framework',
     '底盘 C 板', 'CAN2 接云台 CAN1；CAN1 接四个 C620/M3508',
     ('ChassisCommPortInit', 'ChassisCommPortCommand', 'ChassisCommPortFeedback')),
    ('gimbal', 'Gimbal_USB_CAN1', root / 'communication_reference/gimbal_firmware',
     '云台 C 板', 'USB 接小电脑；CAN1 接底盘 CAN2；UART3/DBUS 接 S.BUS',
     ('NavGatewayInit', 'NavGatewayTask', 'NavGatewayReceive')),
)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


reports = {}
for name in ('host-test-results.json', 'fifo-regression.json'):
    path = root / 'logs' / name
    report = json.loads(path.read_text())
    if not report.get('passed') or report.get('hardware_accessed') is not False:
        raise SystemExit('需要先通过软件测试：' + str(path))
    reports[name] = report

# Validate both images before creating either release. Never access a probe.
for target, prefix, source, board, wiring, required_symbols in profiles:
    entry = metadata['targets'][target]
    if entry['source'] != str(source):
        raise SystemExit('构建来源路径不匹配，请重新编译：' + target)
    actual = {str(p.relative_to(root)): digest(p) for p in sorted(source.rglob('*'))
              if p.is_file() and p.suffix in ('.c', '.h', '.ld', '.s', '.S')}
    if actual != entry['source_sha256']:
        raise SystemExit('源码在构建后发生变化，请重新编译：' + target)
    build_file, field = (('CMakeLists.txt', 'cmake_file_sha256') if target == 'chassis'
                         else ('Makefile', 'makefile_sha256'))
    if digest(source / build_file) != entry[field]:
        raise SystemExit('构建配置发生变化，请重新编译：' + target)
    for ext in ('elf', 'hex', 'bin'):
        path = root / 'artifacts' / (prefix + '.' + ext)
        if digest(path) != entry['artifacts'][path.name]['sha256']:
            raise SystemExit('产物校验不匹配：' + str(path))
    symbols = subprocess.check_output(
        ['arm-none-eabi-nm', '--defined-only', str(root / 'artifacts' / (prefix + '.elf'))],
        text=True)
    defined = {line.split()[-1] for line in symbols.splitlines() if line.split()}
    if not set(required_symbols) <= defined:
        raise SystemExit('固件缺少通信入口：' + target)
    with tempfile.TemporaryDirectory(prefix='rm-dual-image-check-') as folder:
        for ext in ('elf', 'hex'):
            converted = Path(folder) / (ext + '.bin')
            subprocess.run(['arm-none-eabi-objcopy', '-I',
                            'ihex' if ext == 'hex' else 'elf32-littlearm', '-O', 'binary',
                            str(root / 'artifacts' / (prefix + '.' + ext)),
                            str(converted)], check=True)
            if converted.read_bytes() != (root / 'artifacts' / (prefix + '.bin')).read_bytes():
                raise SystemExit('ELF/HEX/BIN 内容不一致：' + target)

index = root / 'release' / release_name
index.mkdir(parents=True)
destinations = []
for target, prefix, source, board, wiring, required_symbols in profiles:
    destination = source / 'release' / release_name
    destination.mkdir(parents=True)
    for ext in ('elf', 'hex', 'bin'):
        shutil.copy2(root / 'artifacts' / (prefix + '.' + ext), destination)
    manifest = {
        'release': release_name,
        'board': board,
        'wiring': wiring,
        'protocol': {'usb_route': 2, 'can_bitrate': 1000000,
                     'command_ids': ['0x330', '0x331', '0x332'],
                     'feedback_ids': ['0x338', '0x339', '0x33A'],
                     'remote_ids': ['0x340', '0x341', '0x342'],
                     'can_tx_deadline_ms': 8, 'can_group_deadline_ms': 10},
        'bin_flash_address': '0x08000000',
        'flashed': False, 'hardware_validated': False,
        'build': metadata['targets'][target],
        'software_validation': reports,
    }
    (destination / 'build_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    note = ('底盘运动宏为 1；遥控许可、四电机在线、STOP 会话和 TTL 仍必须满足。'
            if target == 'chassis' else
            '本包来自 04 目录自带配套中转工程；GIMBAL_BOARD 已启用，云台执行器安装门控仍为 0。')
    (destination / '烧录说明.md').write_text(
        f'# {board}：{release_name}\n\n'
        f'推荐烧录 `{prefix}.elf`，也可选同名 HEX；BIN 起始地址为 `0x08000000`。'
        '三种格式是同一程序，选一种即可。\n\n'
        f'接线：{wiring}。板间 CAN 为 1 Mbps，H 对 H、L 对 L，两板共地。\n\n'
        f'{note}\n\n'
        '两块板必须分别连接 SWD，核对目标板后烧写、校验并复位。'
        '本次仅生成固件，未访问硬件、未烧录、未进行实物通信验收。\n\n'
        '配套固件与测试步骤见总工程 docs/双板通信固件生成与烧录-20261120.md。'
        '烧录后先用 tools/zero_test.sh 执行 10 秒 STOP/ACK 测试。\n')
    checksum_files = sorted(p for p in destination.iterdir() if p.is_file())
    (destination / 'SHA256SUMS').write_text(''.join(
        digest(p) + '  ' + p.name + '\n' for p in checksum_files))
    destinations.append({'board': board, 'directory': str(destination),
                         'elf': str(destination / (prefix + '.elf'))})
    print(board + ': ' + str(destination))

(index / '固件位置.json').write_text(json.dumps({
    'release': release_name, 'flashed': False, 'targets': destinations,
}, ensure_ascii=False, indent=2) + '\n')
(index / 'README.md').write_text(
    '# 双板通信固件入口\n\n'
    '本次固件分别保存在两份 MCU 工程各自的 release 目录：\n\n' + ''.join(
        '- [' + item['board'] + '](' +
        os.path.relpath(item['directory'], index) + '/烧录说明.md)\n'
        for item in destinations) +
    '\n两板独立烧录；小电脑 USB → 云台 CAN1 ↔ 底盘 CAN2。'
    '本次未烧录、未访问实物。\n')
print('Index: ' + str(index))
