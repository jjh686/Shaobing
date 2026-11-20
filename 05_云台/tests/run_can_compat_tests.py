#!/usr/bin/env python3
"""Run 05 against the unchanged 04 codecs and chassis port; no hardware access."""
import argparse
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--reference', type=Path, default=root.parent / '04_DiPan_Board')
args = parser.parse_args()
reference = args.reference.resolve()
nav = root / 'modules/nav_conversion'
original = reference / 'communication_reference/gimbal_firmware/modules/nav_conversion'
# Exact implementation equality includes CRC, units, IDs, quantization, session
# rules and the shared USB protocol. read_text normalizes historical CRLF files.
for name in ('nav_can.h', 'nav_can.c', 'remote_can.h', 'protocol.c', 'mcu_link.c',
             'gimbal_protocol.c', 'nav_gateway.c', 'nav_gateway.h',
             'nav_usb_mux.c', 'nav_usb_mux.h', 'rm_chassis_comm/protocol.h',
             'rm_chassis_comm/mcu_link.h', 'rm_chassis_comm/gimbal_protocol.h'):
    actual = (nav / name).read_text()
    expected = (original / name).read_text()
    if name == 'nav_can.h':
        actual = actual.replace('8u /* Match 04; finish/abort before the 10 ms assembly window. */', '8u')
    assert actual == expected, f'04/05 protocol mismatch: {name}'
print('PASS 04/05 codecs, headers, USB mux and session gate match', flush=True)

chassis = reference / 'chassis_driver/basic_framework/modules/chassis_comm'
flags = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-g',
         '-DCHASSIS_COMM_HOST_TEST', '-fsanitize=undefined', '-fno-sanitize-recover=all',
         '-I' + str(root / 'tests/navigation_stubs'), '-I' + str(nav),
         '-I' + str(root / 'modules/chassis_link'),
         '-I' + str(root / 'application/communication'), '-I' + str(chassis),
         '-I' + str(reference / 'tests/stubs')]
codecs = [nav / name for name in ('protocol.c', 'mcu_link.c', 'nav_can.c')]
gateway = [nav / name for name in ('gimbal_protocol.c', 'nav_usb_mux.c', 'nav_gateway.c')]
with tempfile.TemporaryDirectory(prefix='gimbal-can-compat-') as temporary:
    build = Path(temporary)
    def run(name, sources, defines=()):
        executable = build / name
        subprocess.run(flags + list(defines) + [str(p) for p in sources] +
                       ['-lm', '-o', str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=60)

    run('protocol', [reference / 'tests/test_protocol.c', *codecs, *gateway])
    run('remote', [reference / 'tests/test_remote_can.c', *codecs])
    run('mailboxes', [root / 'tests/test_nav_can_hal.c', *codecs])
    bsp_flags = flags.copy()
    bsp_flags[bsp_flags.index('-I' + str(root / 'tests/navigation_stubs'))] = (
        '-I' + str(root / 'tests/can_bsp_stubs'))
    bsp_flags.insert(1, '-I' + str(root / 'bsp/can'))
    subprocess.run(bsp_flags + [
        str(root / 'tests/test_can_bsp.c'), str(root / 'bsp/can/bsp_can.c'),
        '-o', str(build / 'bsp')], check=True)
    subprocess.run([str(build / 'bsp')], check=True, timeout=30)
    subprocess.run(flags[:1] + ['-I' + str(root / 'tests/chassis_link_task_stubs')] +
        flags[1:] + [str(root / 'tests/test_chassis_link_task.c'),
        str(root / 'modules/chassis_link/chassis_link_task.c'),
        '-o', str(build / 'task')], check=True)
    subprocess.run([str(build / 'task')], check=True, timeout=30)
    # The current USB CDC implementation reports both Init/DeInit through the
    # zero-length RX callback, which calls NavGatewayReset. Model that event as
    # well as USBIsConfigured, so ENABLE cannot cross a USB session boundary.
    source = (reference / 'tests/test_chain.c').read_text()
    old = 'configured=0;tick=511;step(1);configured=1;tick=531;'
    new = ('configured=0;NavGatewayReset();tick=511;step(1);'
           'configured=1;NavGatewayReset();tick=531;')
    assert source.count(old) == 1, 'Review 04 chain fixture: USB session event changed'
    source = source.replace(old, new)
    # 05 deliberately drains feedback even with USB absent; retain this behavior.
    source = source.replace('configured=0;NavGatewayReset();tick=511;step(1);',
        'configured=0;NavGatewayReset();tick=511;'
        'unsigned feedback_before=nav_feedback_received;step(1);'
        'assert(nav_feedback_received>feedback_before);')
    chain = build / 'test_chain.c'
    chain.write_text(source)
    for variant, enabled in (('locked', '0'), ('enabled', '1')):
        run('chain_' + variant,
            [chain, chassis / 'chassis_comm.c', chassis / 'chassis_comm_port.c',
             root / 'modules/chassis_link/chassis_link.c', *codecs, *gateway],
            ['-DCHASSIS_COMM_ALLOW_MOTION=' + enabled,
             '-DCHASSIS_COMM_AXES_VERIFIED=' + enabled])
print('PASS CAN compatibility: 7 executables; 04 sources unchanged; no hardware accessed')
