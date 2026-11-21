#!/usr/bin/env python3
"""Run 05 host simulations and record their source snapshot; no hardware access."""
import hashlib
import json
from pathlib import Path
import runpy
import subprocess
import time

root = Path(__file__).resolve().parents[1]
inventory = runpy.run_path(str(root / 'tools/record_build.py'))['source_inventory']
before = inventory()
started = time.monotonic()
(root / 'logs').mkdir(exist_ok=True)
log_path = root / 'logs/gimbal-host-tests.log'
command = ['sh', str(root / 'tests/run_all_host_tests.sh')]
with log_path.open('w') as log:
    process = subprocess.Popen(command, cwd=root, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT)
    for line in process.stdout:
        log.write(line)
        print(line, end='', flush=True)
    result = process.wait()
report = {
    'passed': result == 0 and before == inventory(),
    'hardware_accessed': False, 'flashed': False,
    'command': 'sh tests/run_all_host_tests.sh', 'exit_code': result,
    'source_sha256': before,
    'log_sha256': hashlib.sha256(log_path.read_bytes()).hexdigest(),
    'test_sha256': {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sorted((root / 'tests').rglob('*')) if p.is_file()},
    'results': log_path.read_text().splitlines(),
    'duration_seconds': time.monotonic() - started,
}
(root / 'logs/gimbal-test-results.json').write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + '\n')
raise SystemExit(0 if report['passed'] else 1)
