#!/usr/bin/env python3
"""Record the 05 gimbal build and its source hashes without accessing hardware."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_inventory():
    # Match the files used by Make; omit test fixtures and generated build trees.
    folders = ('application', 'bsp', 'modules', 'Inc', 'Src', 'Drivers', 'Middlewares')
    files = [p for folder in folders for p in (root / folder).rglob('*')
             if p.is_file() and p.suffix in ('.c', '.h', '.ld', '.s', '.S')]
    files += [p for p in root.iterdir() if p.suffix in ('.ld', '.s', '.S', '.ioc')]
    return {str(p.relative_to(root)): digest(p) for p in sorted(files)}


if __name__ == '__main__':
    entry = {
        'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
        'board': 'gimbal C board', 'source': str(root),
        'role': '05 complete gimbal; USB navigation gateway; CAN1 to chassis CAN2',
        'bin_flash_address': '0x08000000',
        'compiler': subprocess.check_output(['arm-none-eabi-gcc', '--version'], text=True).splitlines()[0],
        'size_output': subprocess.check_output(
            ['arm-none-eabi-size', str(root / 'artifacts/Gimbal_ChassisLink.elf')], text=True).strip(),
        'source_sha256': source_inventory(),
        'makefile_sha256': digest(root / 'Makefile'),
        'artifacts': {},
        'configuration_note': 'Retains 05 gimbal control and CAN1 motor retransmission; '
                              'SBUS is initialized for chassis permission even in auto-aim mode.',
    }
    for ext in ('elf', 'hex', 'bin'):
        path = root / 'artifacts' / ('Gimbal_ChassisLink.' + ext)
        entry['artifacts'][path.name] = {'sha256': digest(path), 'bytes': path.stat().st_size}
    metadata = {'project_root': str(root), 'flashed_by_this_task': False,
                'hardware_validated_by_this_task': False, 'targets': {'gimbal': entry}}
    (root / 'artifacts/build_config.json').write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + '\n')
    print('Recorded 05 gimbal image and source hashes')
