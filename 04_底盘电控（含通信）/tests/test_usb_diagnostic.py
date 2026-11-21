#!/usr/bin/env python3
"""Exercise the real STOP checker through a PTY, including false ACK cases."""
import contextlib
import io
import json
import os
from pathlib import Path
import pty
import select
import struct
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'communication_reference/pc'))
from usb_protocol import Stream, FeedbackStats, crc
from zero_test import run
from check_port import check


@contextlib.contextmanager
def usb_identity(port, vid='0483', pid='00ca', product='Vision Comm port'):
    """Exercise the real device check with a PTY and a fake sysfs USB tree."""
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        usb = root / 'devices/usb1/1-1'
        interface = usb / '1-1:1.0/tty' / Path(port).name
        interface.mkdir(parents=True)
        for name, value in (('idVendor', vid), ('idProduct', pid), ('product', product)):
            (usb / name).write_text(value + '\n')
        tty = root / 'class/tty' / Path(port).name
        tty.mkdir(parents=True)
        (tty / 'device').symlink_to(interface, target_is_directory=True)
        with patch('check_port.SYS_TTY', root / 'class/tty'):
            yield


class PortIdentityTest(unittest.TestCase):
    def setUp(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        self.port = os.ttyname(slave)

    def test_current_and_legacy_gimbal_ports(self):
        for product in ('Vision Comm port', 'RM Nav Convert SP'):
            with self.subTest(product=product), usb_identity(self.port, product=product):
                self.assertEqual(check(self.port), self.port)

    def test_debug_probe_wrong_ids_and_unknown_products_are_rejected(self):
        identities = [('0d28', '0204', 'CMSIS-DAP'),
                      ('0d28', '0204', 'Vision Comm port'),
                      ('0483', '5740', 'Vision Comm port'),
                      ('0483', '00ca', 'Unknown application')]
        for vid, pid, product in identities:
            with self.subTest(identity=(vid, pid, product)), \
                    usb_identity(self.port, vid, pid, product):
                with self.assertRaisesRegex(ValueError, 'USB 设备不匹配'):
                    check(self.port)

    def test_empty_directory_regular_file_and_non_usb_tty_are_rejected(self):
        with self.assertRaisesRegex(ValueError, '串口参数为空'):
            check('')
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(ValueError, '指向目录'):
                check(temporary)
            file = Path(temporary) / 'not-a-serial-port'
            file.touch()
            with self.assertRaisesRegex(ValueError, '不是串口字符设备'):
                check(file)
            # A bare PTY remains rejected in production; tests must simulate USB.
            with patch('check_port.SYS_TTY', Path(temporary) / 'empty-sysfs'):
                with self.assertRaisesRegex(ValueError, '需要真实 C 板 USB 串口'):
                    check(self.port)


def frame(kind, payload):
    header = bytes((0x5a, len(payload), kind))
    data = header + bytes((crc(header, 255, 0x8c, 255),)) + payload
    return data + struct.pack('<H', crc(data))


def joint(session, seq, uptime):
    return frame(0x22, struct.pack('<IBBHHHIIIffff', uptime, 1, 2, seq,
                                  0, 0, session, uptime, uptime, 0., 0., 0., 0.))


class DiagnosticTest(unittest.TestCase):
    def test_fragmented_and_corrupt_joint(self):
        parser = Stream()
        packet = joint(123, 7, 800)
        corrupt = bytearray(packet)
        corrupt[-1] ^= 1
        self.assertEqual(list(parser.push(corrupt)), [])
        self.assertEqual(parser.frames['gimbal'], 0)
        self.assertGreaterEqual(parser.diagnostics['crc_errors'], 1)
        for byte in packet:
            self.assertEqual(list(parser.push(bytes((byte,)))), [])
        self.assertEqual(parser.frames['gimbal'], 1)
        self.assertEqual(parser.last_gimbal['session'], 123)
        self.assertEqual(parser.last_gimbal['sequence'], 7)

    def exchange(self, mode, product=None):
        master, slave = pty.openpty()
        stop = threading.Event()
        failures = []

        def board():
            buffer = bytearray()
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], .02)[0]:
                        continue
                    buffer.extend(os.read(master, 4096))
                    while len(buffer) >= 34:
                        if buffer[:3] != b'\x5a\x1c\x20':
                            del buffer[0]
                            continue
                        packet = bytes(buffer[:34])
                        del buffer[:34]
                        self.assertEqual(crc(packet[:-2]), int.from_bytes(packet[-2:], 'little'))
                        up, version, route, seq, ttl, flags, session, vx, vy, wz = struct.unpack('<IBBHHHIfff', packet[4:32])
                        self.assertEqual((ttl, flags, vx, vy, wz), (0, 0, 0., 0., 0.))
                        if mode == 'joint_only':
                            response = joint(session, seq, up + 1000)
                        else:
                            if mode == 'wrong_session':
                                session = (session % 0xffffffff) + 1
                            response = frame(0x21, struct.pack('<IBBHHHIfff', up + 1000,
                                version, route, seq, 64, 15, session, 0., 0., 0.))
                        # Actual USB chunks need not coincide with protocol frames.
                        os.write(master, response[:9])
                        os.write(master, response[9:])
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=board)
        worker.start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                report = Path(tmp) / 'result.json'
                port = os.ttyname(slave)
                device = (usb_identity(port, product=product) if product is not None else
                          patch('zero_test.check', return_value=port))
                with device, contextlib.redirect_stdout(io.StringIO()):
                    if mode == 'good':
                        run(os.ttyname(slave), 1, report)
                    else:
                        with self.assertRaisesRegex(RuntimeError, '有效 0 帧'):
                            run(os.ttyname(slave), 1, report)
                data = json.loads(report.read_text())
        finally:
            stop.set()
            worker.join(timeout=2)
            os.close(master)
            os.close(slave)
        self.assertFalse(worker.is_alive())
        if failures:
            raise failures[0]
        return data

    def test_fresh_chassis_ack_passes(self):
        data = self.exchange('good')
        self.assertTrue(data['passed'])
        self.assertGreaterEqual(data['fresh_chassis_ACK'], 10)
        self.assertEqual(data['fresh_chassis_ACK'], data['ack_diagnostics']['accepted'])

    def test_05_usb_identity_then_fresh_chassis_ack_passes(self):
        data = self.exchange('good', product='Vision Comm port')
        self.assertTrue(data['passed'])
        self.assertGreaterEqual(data['fresh_chassis_ACK'], 10)

    def test_05_usb_identity_does_not_turn_telemetry_into_chassis_ack(self):
        data = self.exchange('joint_only', product='Vision Comm port')
        self.assertFalse(data['passed'])
        self.assertEqual(data['fresh_chassis_ACK'], 0)
        self.assertGreater(data['frames']['gimbal'], 10)

    def test_joint_is_never_chassis_ack(self):
        data = self.exchange('joint_only')
        self.assertFalse(data['passed'])
        self.assertGreater(data['frames']['gimbal'], 10)
        self.assertEqual(data['frames']['feedback'], 0)
        self.assertIn('已解析本轮', data['diagnosis'])

    def test_other_session_ack_fails(self):
        data = self.exchange('wrong_session')
        self.assertFalse(data['passed'])
        self.assertGreater(data['frames']['feedback'], 10)
        self.assertEqual(data['fresh_chassis_ACK'], 0)
        self.assertEqual(data['frames']['feedback'], data['ack_diagnostics']['wrong_session'])

    def test_feedback_gap_counts_wrap_without_inventing_a_loss_rate(self):
        stats = FeedbackStats()
        stats.received({'uptime': 0xfffffff0}, 1.)
        stats.reject('wrong_session')
        stats.received({'uptime': 24}, 1.04)
        stats.accept(1.04)
        report = stats.report()
        self.assertEqual(report['received'], 2)
        self.assertEqual(report['accepted'], 1)
        self.assertEqual(report['max_uptime_gap_ms'], 40)
        self.assertAlmostEqual(report['max_feedback_gap_ms'], 40.)


if __name__ == '__main__':
    unittest.main()
