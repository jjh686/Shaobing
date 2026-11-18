#!/usr/bin/env python3
"""Reproduce delayed dual-FIFO reordering; compile host simulations only."""
import json
from pathlib import Path
import re
import resource
import signal
import subprocess

root = Path(__file__).resolve().parents[1]
module = Path('chassis_driver/basic_framework/modules/chassis_comm')
common = Path('communication_reference/common')
gimbal = Path('communication_reference/gimbal_firmware/application/communication')
directory = root / 'build/tests/fifo-regression'
directory.mkdir(parents=True, exist_ok=True)
(root / 'logs').mkdir(exist_ok=True)

# Generated negative-test source is deliberately wrong. Never build ARM images from it.
ports = [module / 'chassis_comm_port.c', gimbal / 'nav_gateway_port.c']
negative_ports = []
for source in ports:
    fixed = (root / source).read_text()
    legacy, count = re.subn(r'(\.rx_fifo_policy\s*=\s*)CAN_RX_FIFO_POLICY_FIFO0',
                            r'\1CAN_RX_FIFO_POLICY_AUTO', fixed)
    if count != 1:
        raise RuntimeError(f'Expected one explicit FIFO0 policy in {source}; got {count}')
    output = directory / source.name
    output.write_text('/* HOST NEGATIVE TEST ONLY: intentionally restores old defect. */\n' + legacy)
    negative_ports.append(str(output))

flags = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-g', '-DCHASSIS_COMM_HOST_TEST',
         '-fsanitize=undefined', '-fno-sanitize-recover=all',
         '-I' + str(module), '-I' + str(module / 'protocol'), '-Itests/stubs',
         '-I' + str(common), '-I' + str(gimbal)]
sources = ['tests/test_chain.c', str(module / 'chassis_comm.c')]
sources += [str(module / 'protocol' / f) for f in ('protocol.c', 'mcu_link.c', 'nav_can.c')]
sources += [str(common / f) for f in ('gimbal_protocol.c', 'nav_usb_mux.c', 'nav_gateway.c')]
report = {'passed': False, 'hardware_accessed': False, 'cases': [],
          'negative_test_sources': 'Generated only under build/tests/fifo-regression; not firmware inputs'}

def no_core_dump():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

try:
    for name, selected in [('legacy_alternating', negative_ports),
                           ('fixed_single_fifo', [str(p) for p in ports])]:
        executable = directory / name
        built = subprocess.run(flags + sources + selected + ['-lm', '-o', str(executable)],
                               cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if built.returncode:
            raise RuntimeError(f'{name} compile failed:\n{built.stdout}')
        result = subprocess.run([str(executable)], cwd=root, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=30, preexec_fn=no_core_dump)
        expected = ((result.returncode == -signal.SIGABRT and 'ack_count==1' in result.stdout)
                    if name == 'legacy_alternating' else result.returncode == 0)
        report['cases'].append({'variant': name, 'exit_code': result.returncode,
                                'expected_result_observed': expected,
                                'output': result.stdout.strip()})
        if not expected:
            raise RuntimeError(f'{name}: unexpected result:\n{result.stdout}')
        print('PASS ' + name + (': expected STOP ACK failure reproduced'
                               if name == 'legacy_alternating' else ': corrected round trips passed'))
    report['passed'] = True
    report['conclusion'] = ('Deferred RX FIFO interrupt delivery reproduces legacy group reordering; '
                            'explicit same-FIFO policy preserves order in this simulation.')
finally:
    (root / 'logs/fifo-regression.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
