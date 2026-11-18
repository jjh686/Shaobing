#!/usr/bin/env python3
"""PTY-only checks: no real USB or motor is accessed."""
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
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'communication_reference/pc'))
from motor_test import run
from usb_protocol import crc
from test_usb_diagnostic import frame


class MotorTest(unittest.TestCase):
    def exchange(self, mode, duration=.2):
        master, slave = pty.openpty()
        stop = threading.Event()
        packets = []
        failures = []

        def board():
            data = bytearray()
            started = time.monotonic()
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], .02)[0]:
                        continue
                    data.extend(os.read(master, 4096))
                    while len(data) >= 34:
                        packet = bytes(data[:34])
                        del data[:34]
                        self.assertEqual(packet[:3], b'\x5a\x1c\x20')
                        self.assertEqual(crc(packet[:-2]), int.from_bytes(packet[-2:], 'little'))
                        up, ver, route, seq, ttl, flags, session, vx, vy, wz = struct.unpack('<IBBHHHIfff', packet[4:32])
                        packets.append((flags, ttl, vx, vy, wz))
                        if flags:
                            self.assertEqual(ttl, 100)
                            self.assertAlmostEqual(vx, .05)
                            self.assertEqual((vy, wz), (0., 0.))
                        else:
                            self.assertEqual((ttl, vx, vy, wz), (0, 0., 0., 0.))
                        if mode == 'drop_active' and flags:
                            continue
                        status, fault = 73 | (2 if flags else 0), 0
                        if mode == 'motor_offline':
                            status, fault = 64, 4
                        if mode == 'rc_lost' and time.monotonic() - started >= 3.3:
                            status, fault = 104 | (2 if flags else 0), 2
                        if mode == 'wrong_session':
                            session = session % 0xffffffff + 1
                        uptime = int((time.monotonic()-started)*1000) + 1000
                        response = frame(0x21, struct.pack('<IBBHHHIfff', uptime,
                            ver, route, seq, status, fault, session, 0., 0., 0.))
                        os.write(master, response)
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=board)
        worker.start()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                report_path = Path(tmp)/'result.json'
                with patch('motor_test.check', return_value=os.ttyname(slave)), contextlib.redirect_stdout(io.StringIO()):
                    if mode == 'good':
                        run(os.ttyname(slave), duration=duration, wheels_raised=True, report_path=report_path)
                    else:
                        with self.assertRaises(RuntimeError):
                            run(os.ttyname(slave), duration=duration, wheels_raised=True, report_path=report_path)
                report = json.loads(report_path.read_text())
        finally:
            stop.set()
            worker.join(timeout=2)
            os.close(master)
            os.close(slave)
        self.assertFalse(worker.is_alive())
        if failures:
            raise failures[0]
        self.assertTrue(packets)
        self.assertFalse(packets[-1][0], 'Last transmitted packet must be disabled STOP')
        return report, packets

    def test_enabled_then_stop(self):
        report, packets = self.exchange('good')
        self.assertTrue(report['passed'])
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(any(p[0] for p in packets))

    def test_motor_fault_blocks_all_enable(self):
        report, packets = self.exchange('motor_offline')
        self.assertFalse(report['passed'])
        self.assertFalse(any(p[0] for p in packets))
        self.assertIn('四电机', report['error'])
        self.assertTrue(report['stop_confirmed'])

    def test_ten_second_enabled_then_stop(self):
        report, packets = self.exchange('good', duration=10)
        self.assertTrue(report['passed'])
        self.assertTrue(report['stop_confirmed'])
        self.assertEqual(report['duration'], 10)
        self.assertGreater(report['active_ACK'], 100)
        self.assertTrue(any(p[0] for p in packets))

    def test_ten_second_test_aborts_on_later_remote_fault(self):
        report, packets = self.exchange('rc_lost', duration=10)
        self.assertFalse(report['passed'])
        self.assertIn('底盘撤销许可', report['error'])
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(any(p[0] for p in packets))

    def test_active_ack_loss_aborts_and_stops(self):
        report, packets = self.exchange('drop_active')
        self.assertFalse(report['passed'])
        self.assertIn('150ms', report['error'])
        self.assertTrue(report['stop_confirmed'])
        self.assertTrue(any(p[0] for p in packets))

    def test_unrelated_ack_never_unlocks(self):
        report, packets = self.exchange('wrong_session')
        self.assertFalse(report['passed'])
        self.assertFalse(any(p[0] for p in packets))
        self.assertFalse(report['stop_confirmed'])

    def test_confirmation_and_limits_before_opening(self):
        for kwargs in ({}, {'wheels_raised': True, 'speed': .11},
                       {'wheels_raised': True, 'duration': float('nan')},
                       {'wheels_raised': True, 'duration': 10.01}):
            with patch('motor_test.check') as check:
                with self.assertRaises(ValueError):
                    run('/unused', **kwargs)
                check.assert_not_called()


if __name__ == '__main__':
    unittest.main()
