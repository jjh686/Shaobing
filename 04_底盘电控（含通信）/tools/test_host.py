#!/usr/bin/env python3
"""Build and execute actual source against simulated peripherals; no hardware."""
import json
from pathlib import Path
import subprocess
import time

root = Path(__file__).resolve().parents[1]
module = Path('chassis_driver/basic_framework/modules/chassis_comm')
gimbal = root.parent/'05_云台'
remote_headers = [root/'communication_reference/common/remote_can.h',
                  gimbal/'modules/nav_conversion/remote_can.h',
                  root/module/'protocol/remote_can.h']
assert len({p.read_bytes() for p in remote_headers}) == 1, 'Remote CAN protocol copies differ'
ref = Path('communication_reference')
protocol = module/'protocol'
build = root/'build/tests'
build.mkdir(parents=True, exist_ok=True)
(root/'logs').mkdir(exist_ok=True)
flags = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-g', '-DCHASSIS_COMM_HOST_TEST',
         '-fsanitize=undefined', '-fno-sanitize-recover=all',
         '-I'+str(module), '-I'+str(protocol), '-Itests/stubs',
         '-I'+str(ref/'common'), '-I'+str(gimbal/'application/communication'), '-I'+str(gimbal/'modules/chassis_link')]
codecs = [str(protocol/f) for f in ('protocol.c', 'mcu_link.c', 'nav_can.c')]
gateway = [str(ref/'common'/f) for f in ('gimbal_protocol.c', 'nav_usb_mux.c', 'nav_gateway.c')]
tests = [('remote_can', [], ['tests/test_remote_can.c', *codecs]),
         ('usb_tx', ['-I'+str(gimbal/'bsp/usb')], ['tests/test_usb_tx.c']),
         ('usb_bsp', ['-Itests/usb_stubs', '-I'+str(gimbal/'bsp/usb')],
                     ['tests/test_usb_bsp.c', str(gimbal/'bsp/usb/bsp_usb.c')]),
         ('sbus', ['-Itests/remote_stubs', '-I'+str(gimbal/'modules/remote'),
                   '-I'+str(gimbal/'bsp/usart'), '-Wno-unused-parameter'],
                  ['tests/test_sbus.c', str(gimbal/'modules/remote/remote_control.c')]),
         ('uart_rx', ['-Itests/remote_stubs', '-I'+str(gimbal/'bsp/usart')],
                     ['tests/test_uart_rx.c', str(gimbal/'bsp/usart/bsp_usart.c')]),
         ('protocol', [], ['tests/test_protocol.c', *codecs, *gateway]),
         ('can_hal', [], ['tests/test_can_hal.c', *codecs, *gateway])]
for variant, defines in [('locked',
        ['-DCHASSIS_COMM_ALLOW_MOTION=0', '-DCHASSIS_COMM_AXES_VERIFIED=0']), ('enabled_simulation',
        ['-DCHASSIS_COMM_ALLOW_MOTION=1', '-DCHASSIS_COMM_AXES_VERIFIED=1'])]:
    tests.append(('core_'+variant, defines, ['tests/test_chassis_comm.c',
                                           str(module/'chassis_comm.c'), *codecs]))
    tests.append(('chain_'+variant, defines, ['tests/test_chain.c',
        str(module/'chassis_comm.c'), str(module/'chassis_comm_port.c'),
        str(gimbal/'modules/chassis_link/chassis_link.c'), *codecs, *gateway]))
report = {'gimbal_source': str(gimbal), 'hardware_accessed': False,
          'variants': 'Explicit locked=0/0 and enabled=1/1 host simulations; ARM configuration is recorded separately',
          'tests': [], 'passed': False}
start = time.monotonic()
try:
    with (root/'logs/host-tests.log').open('w') as log:
        for name, defines, sources in tests:
            compile_cmd = flags[:1] + defines + flags[1:] + sources + ['-lm', '-o', str(build/name)]
            compiled = subprocess.run(compile_cmd, cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            log.write('BUILD '+name+'\n'+compiled.stdout)
            if compiled.returncode:
                raise RuntimeError(name+' compile failed:\n'+compiled.stdout)
            result = subprocess.run([str(build/name)], cwd=root, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
            log.write(result.stdout)
            report['tests'].append({'name': name, 'exit_code': result.returncode, 'output': result.stdout.strip()})
            print(result.stdout, end='', flush=True)
            if result.returncode:
                raise RuntimeError(name+' failed')
    report['passed'] = True
finally:
    report['duration_seconds'] = time.monotonic()-start
    (root/'logs/host-test-results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
print('PASS',len(tests),'host test executables; no hardware accessed')
