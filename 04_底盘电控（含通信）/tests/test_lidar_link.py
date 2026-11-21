#!/usr/bin/env python3
"""Real ROS PointCloud2 + a simulated USB board on PTYs; no hardware I/O."""
import contextlib
import io
import math
import os
from pathlib import Path
import pty
import select
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2, PointField

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'communication_reference/pc'))
from lidar_link_test import LidarInput, LidarWatchdog, cloud_points, move_once
from usb_protocol import crc


def cloud(stamp_ns=None, value=1., big=False, rows=1):
    stamp_ns = time.time_ns() if stamp_ns is None else stamp_ns
    msg = PointCloud2()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.header.frame_id = 'front_mid360'
    msg.width, msg.height, msg.point_step = 32, rows, 12
    msg.row_step = msg.width * msg.point_step + (4 if rows > 1 else 0)
    msg.is_bigendian = big
    msg.fields = [PointField(name=name, offset=i*4, datatype=7, count=1)
                  for i, name in enumerate(('x', 'y', 'z'))]
    row = struct.pack(('>' if big else '<') + 'fff', value, 1., 1.) * msg.width
    msg.data = (row + bytes(msg.row_step - len(row))) * rows
    return msg


class WatchdogTest(unittest.TestCase):
    def warm(self):
        watchdog = LidarWatchdog()
        for i in range(5):
            mono = 1. + i*.02
            stamp = 100_000_000_000 + i*20_000_000
            self.assertTrue(watchdog.receive(cloud(stamp), stamp, mono))
        watchdog.arm(stamp, mono)
        return watchdog, stamp, mono

    def test_xyz_validation_with_padding_and_endianness(self):
        self.assertEqual(cloud_points(cloud(big=True, rows=2)), (64, 64))
        self.assertEqual(cloud_points(cloud(value=math.nan)), (32, 0))
        malformed = cloud()
        malformed.data = malformed.data[:-1]
        with self.assertRaises(ValueError):
            cloud_points(malformed)

    def test_missing_stale_and_future_source_are_rejected(self):
        watchdog = LidarWatchdog()
        with self.assertRaises(RuntimeError):
            watchdog.require_fresh(100_000_000_000, 1.)
        for stamp in (0, 99_000_000_000, 101_000_000_000):
            self.assertFalse(watchdog.receive(cloud(stamp), 100_000_000_000, 1.))
        self.assertEqual(watchdog.accepted, 0)

    def test_replayed_stamp_latches_stop(self):
        watchdog, stamp, mono = self.warm()
        self.assertFalse(watchdog.receive(cloud(stamp), stamp+10_000_000, mono+.01))
        self.assertTrue(watchdog.receive(cloud(stamp+20_000_000), stamp+20_000_000, mono+.02))
        with self.assertRaisesRegex(RuntimeError, '重复'):
            watchdog.require_fresh(stamp+20_000_000, mono+.02)

    def test_gap_cannot_be_hidden_by_a_fresh_recovery_frame(self):
        watchdog, stamp, mono = self.warm()
        self.assertTrue(watchdog.receive(cloud(stamp+200_000_000), stamp+200_000_000, mono+.2))
        with self.assertRaisesRegex(RuntimeError, '中断'):
            watchdog.require_fresh(stamp+200_000_000, mono+.2)

    def test_fresh_receive_does_not_extend_source_expiry(self):
        watchdog, stamp, mono = self.warm()
        self.assertTrue(watchdog.receive(cloud(stamp+10_000_000), stamp+140_000_000, mono+.14))
        with self.assertRaisesRegex(RuntimeError, '超时'):
            watchdog.require_fresh(stamp+170_000_000, mono+.17)

    def test_clock_jump_and_invalid_frame_latch_stop(self):
        watchdog, stamp, mono = self.warm()
        with self.assertRaisesRegex(RuntimeError, '时钟'):
            watchdog.require_fresh(stamp+500_000_000, mono+.01)
        watchdog, stamp, mono = self.warm()
        msg = cloud(stamp+20_000_000)
        msg.header.frame_id = 'wrong_frame'
        self.assertFalse(watchdog.receive(msg, stamp+20_000_000, mono+.02))
        with self.assertRaisesRegex(RuntimeError, 'frame_id'):
            watchdog.require_fresh(stamp+20_000_000, mono+.02)


class RosUsbTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def exercise(self, mode):
        topic = '/lidar_link_check_' + str(os.getpid()) + '/' + mode
        sensor = LidarInput(topic, .15)
        publisher_node = rclpy.create_node('lidar_link_check_publisher_' + mode)
        publisher = publisher_node.create_publisher(PointCloud2, topic, qos_profile_sensor_data)
        master, slave = pty.openpty()
        quit_event, active_event = threading.Event(), threading.Event()
        packets, failures = [], []
        last_published, active_started = [None], [None]

        def publish():
            try:
                while not quit_event.wait(.025):
                    now = time.monotonic()
                    if mode == 'drop' and active_event.is_set() and now-active_started[0] < .4:
                        continue
                    stamp = time.time_ns()
                    if mode == 'stale' and active_event.is_set():
                        stamp -= 2_000_000_000
                    publisher.publish(cloud(stamp))
                    last_published[0] = now
            except BaseException as exc:
                failures.append(exc)

        def board():
            buffer = bytearray()
            uptime = 0
            try:
                while not quit_event.is_set():
                    if not select.select([master], [], [], .01)[0]:
                        continue
                    buffer.extend(os.read(master, 4096))
                    while len(buffer) >= 34:
                        packet = bytes(buffer[:34])
                        del buffer[:34]
                        self.assertEqual(packet[:3], b'\x5a\x1c\x20')
                        self.assertEqual(crc(packet[:-2]), int.from_bytes(packet[-2:], 'little'))
                        up, ver, route, seq, ttl, flags, session, vx, vy, wz = struct.unpack('<IBBHHHIfff', packet[4:32])
                        packets.append((time.monotonic(), flags, ttl, vx, vy, wz))
                        if flags:
                            self.assertEqual(ttl, 100)
                            self.assertAlmostEqual(vx, .05)
                            self.assertEqual((vy, wz), (0., 0.))
                            if not active_event.is_set():
                                active_started[0] = time.monotonic()
                                active_event.set()
                        else:
                            self.assertEqual((ttl, vx, vy, wz), (0, 0., 0., 0.))
                        if mode == 'ack_loss' and flags:
                            continue
                        fault = 2 if mode == 'remote_fault' and flags else 0
                        status = 73 | (2 if flags else 0)
                        measured = vx if flags and mode != 'no_motion' else 0.
                        uptime += 20
                        payload = struct.pack('<IBBHHHIfff', uptime, ver, route, seq, status,
                                              fault, session, measured, 0., 0.)
                        header = bytes((0x5a, len(payload), 0x21))
                        response = header + bytes((crc(header, 255, 0x8c, 255),)) + payload
                        response += struct.pack('<H', crc(response))
                        os.write(master, response[:9])
                        os.write(master, response[9:])
            except BaseException as exc:
                failures.append(exc)

        workers = [threading.Thread(target=publish), threading.Thread(target=board)]
        for worker in workers:
            worker.start()
        report = dict(passed=False, stop_confirmed=False)
        error = None
        try:
            sensor.wait(3.)
            original_guard = sensor.guard
            if mode == 'interrupt':
                def guard():
                    original_guard()
                    if active_event.is_set():
                        raise KeyboardInterrupt('模拟Ctrl+C')
                sensor.guard = guard
            args = SimpleNamespace(port=os.ttyname(slave), speed=.05, duration=.6)
            with patch('lidar_link_test.check', return_value=args.port), \
                    contextlib.redirect_stdout(io.StringIO()):
                try:
                    move_once(args, sensor, report)
                except (RuntimeError, KeyboardInterrupt) as exc:
                    error = str(exc)
        finally:
            quit_event.set()
            for worker in workers:
                worker.join(timeout=2)
                self.assertFalse(worker.is_alive())
            publisher_node.destroy_node()
            sensor.node.destroy_node()
            os.close(master)
            os.close(slave)
        if failures:
            raise failures[0]
        self.assertTrue(any(p[1] for p in packets), '未进入运动阶段，测试未覆盖故障路径')
        self.assertTrue(report['stop_confirmed'])
        moving_indices = [i for i, packet in enumerate(packets) if packet[1]]
        self.assertTrue(all(not p[1] for p in packets[moving_indices[-1]+1:]))
        self.assertGreater(len(packets[moving_indices[-1]+1:]), 2)
        return report, error, packets, active_started[0]

    def test_live_cloud_drives_once_and_confirms_stop(self):
        report, error, _, _ = self.exercise('good')
        self.assertIsNone(error)
        self.assertTrue(report['passed'])
        self.assertTrue(report['motion_observed'])

    def test_cloud_loss_stops_without_automatic_restart(self):
        report, error, packets, started = self.exercise('drop')
        self.assertIn('超时', error)
        self.assertFalse(report['passed'])
        first_stop = next(p for p in packets if p[0] > started and not p[1])
        self.assertLess(first_stop[0] - started, .23)
        self.assertFalse(any(p[1] for p in packets if p[0] > first_stop[0]))

    def test_old_source_stamps_stop_even_while_messages_arrive(self):
        _, error, _, _ = self.exercise('stale')
        self.assertIn('时间戳', error)

    def test_remote_fault_stops(self):
        _, error, _, _ = self.exercise('remote_fault')
        self.assertIn('撤销许可', error)

    def test_ack_loss_stops(self):
        _, error, _, _ = self.exercise('ack_loss')
        self.assertIn('150ms', error)

    def test_keyboard_interrupt_still_confirms_stop(self):
        _, error, _, _ = self.exercise('interrupt')
        self.assertIn('Ctrl+C', error)

    def test_ack_without_measured_motion_is_not_a_motion_pass(self):
        report, error, _, _ = self.exercise('no_motion')
        self.assertIn('未检测到前向轮速', error)
        self.assertTrue(report['command_accepted'])
        self.assertFalse(report['motion_observed'])


if __name__ == '__main__':
    unittest.main()
