#!/usr/bin/env python3
"""Record only a just-built target; never access or flash hardware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('target', choices=('chassis', 'gimbal'))
args = parser.parse_args()

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def inventory(directory):
    return {str(p.relative_to(root)): digest(p) for p in sorted(directory.rglob('*'))
            if p.is_file() and p.suffix in ('.c', '.h', '.ld', '.s', '.S')}

metadata_path = root/'artifacts/build_config.json'
metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
metadata.update({'project_root': str(root), 'flashed_by_this_task': False,
                 'hardware_validated_by_this_task': False})
targets = metadata.setdefault('targets', {})
prefix = 'DiPan_Board' if args.target == 'chassis' else 'Gimbal_USB_CAN1'
entry = {'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
         'board': 'chassis C board' if args.target == 'chassis' else 'gimbal C board',
         'role': 'CAN2 navigation endpoint; CAN1 motors' if args.target == 'chassis'
                 else 'USB navigation gateway; CAN1 to chassis CAN2',
         'bin_flash_address': '0x08000000',
         'artifacts': {}}
for ext in ('elf', 'hex', 'bin'):
    path = root/'artifacts'/(prefix+'.'+ext)
    entry['artifacts'][path.name] = {'sha256': digest(path), 'bytes': path.stat().st_size}
entry['compiler'] = subprocess.check_output(['arm-none-eabi-gcc', '--version'], text=True).splitlines()[0]
entry['size_output'] = subprocess.check_output(['arm-none-eabi-size', str(root/'artifacts'/(prefix+'.elf'))], text=True).strip()
if args.target == 'chassis':
    source = root/'chassis_driver/basic_framework'
    commands = json.loads((root/'build/chassis/compile_commands.json').read_text())
    command = next(c for c in commands if c['file'].endswith('/chassis_comm.c'))
    tokens = shlex.split(command['command'])
    preprocessing = []
    skip_next = False
    for token in tokens:
        if skip_next:
            skip_next = False
            continue
        if token == '-o':
            skip_next = True
            continue
        if token != '-c':
            preprocessing.append(token)
    macros = subprocess.check_output(preprocessing + ['-E', '-dM'],
                                     cwd=command['directory'], text=True)
    entry['compiled_configuration'] = {
        line.split(maxsplit=2)[1]: line.split(maxsplit=2)[2]
        if len(line.split(maxsplit=2)) == 3 else ''
        for line in macros.splitlines()
        if line.startswith('#define CHASSIS_COMM_')
    }
    entry['source_sha256'] = inventory(source)
    entry['cmake_file_sha256'] = digest(source/'CMakeLists.txt')
    enabled = all(entry['compiled_configuration'].get(k) == '1' for k in
                  ('CHASSIS_COMM_ALLOW_MOTION', 'CHASSIS_COMM_AXES_VERIFIED'))
    entry['motion_note'] = ('ARM motion gates enabled; RC, motor-online, STOP/session and TTL still required. Physical wheel verification pending.'
                            if enabled else 'See compiled_configuration for the ARM motion gates; host simulation variants are independent.')
else:
    source = root/'communication_reference/gimbal_firmware'
    entry['source_sha256'] = inventory(source)
    entry['makefile_sha256'] = digest(source/'Makefile')
entry['source'] = str(source)
targets[args.target] = entry
metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+chr(10))
print('Recorded '+args.target+' image hashes and build source configuration')
