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
from usb_protocol import Stream, crc
from zero_test import run


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
        for byte in packet:
            self.assertEqual(list(parser.push(bytes((byte,)))), [])
        self.assertEqual(parser.frames['gimbal'], 1)
        self.assertEqual(parser.last_gimbal['session'], 123)
        self.assertEqual(parser.last_gimbal['sequence'], 7)

    def exchange(self, mode):
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
                with patch('zero_test.check', return_value=os.ttyname(slave)), contextlib.redirect_stdout(io.StringIO()):
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


if __name__ == '__main__':
    unittest.main()
