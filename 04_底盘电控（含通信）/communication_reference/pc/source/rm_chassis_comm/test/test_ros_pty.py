#!/usr/bin/env python3
"""End-to-end ROS/serial tests against PTYs only; never opens a real device.
Run after sourcing the built package: python3 test_ros_pty.py /path/chassis_node
"""
import math
import os
import select
import struct
import subprocess
import sys
import tempfile
import time
import tty

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray


def crc(data, initial, polynomial, width):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value & ((1 << width) - 1)


def packet(route, sequence, session, uptime, status=73, measured=.123):
    head = bytes([0x5a, 28, 0x21])
    head += bytes([crc(head, 255, 0x8c, 8)])
    data = head + struct.pack('<IBBHHHIfff', uptime, 1, route, sequence,
                              status, 0, session, measured, 0, 0)
    return data + struct.pack('<H', crc(data, 65535, 0x8408, 16))


def sp_packet(size):
    data = b'SP' + bytes(size - 4)
    return data + struct.pack('<H', crc(data, 65535, 0x8408, 16))


class Check:
    def __init__(self, executable, route):
        self.route = route
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.node = rclpy.create_node('chassis_pty_check_' + str(route))
        self.ns = '/chassis_test_' + str(os.getpid()) + '_' + str(route)
        self.vel = self.node.create_publisher(Twist, self.ns + '/cmd_vel', 1)
        self.measurements = []
        self.proxy = None
        self.diag = None
        self.diag_sub = self.node.create_subscription(DiagnosticArray, self.ns + '/diagnostics',
            lambda m: setattr(self, 'diag', m), 10)
        self.measure_sub = self.node.create_subscription(
            TwistStamped, self.ns + '/measured_velocity', self.measurements.append, 10)
        self.proxy_sub = self.node.create_subscription(
            String, self.ns + '/sp_proxy', lambda m: setattr(self, 'proxy', m.data),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.enable = self.node.create_client(SetBool, self.ns + '/set_enabled')
        self.log = tempfile.TemporaryFile(mode='w+')
        self.proc = subprocess.Popen([
            executable, '--ros-args', '-r', '__ns:=' + self.ns,
            '-p', 'connect:=true', '-p', 'serial_port:=' + os.ttyname(self.slave),
            '-p', 'route:=' + str(route), '-p', 'sp_proxy_enabled:=' + str(route == 2).lower(),
        ], stdout=self.log, stderr=subprocess.STDOUT)
        self.buffer = bytearray()
        self.command_frames = []
        self.sp_commands = []
        self.last = None
        self.start = time.monotonic()
        self.last_feedback = 0
        self.send_feedback = True
        self.override_uptime = None
        self.status = 73  # READY | MEASURED | ACK_VALID

    def io(self):
        if self.proc.poll() is not None:
            raise AssertionError('Driver exited: ' + self.logs())
        try:
            data = os.read(self.master, 4096)
            self.buffer.extend(data)
        except BlockingIOError:
            pass
        while self.buffer:
            if self.buffer[0] == 0x5a:
                if len(self.buffer) < 34:
                    break
                data = bytes(self.buffer[:34])
                if data[1:3] != bytes([28, 0x20]) or crc(data[:3],255,0x8c,8) != data[3] or \
                   crc(data[:-2],65535,0x8408,16) != struct.unpack('<H',data[-2:])[0]:
                    del self.buffer[0]
                    continue
                _, version, route, seq, ttl, flags, session, vx, vy, wz = struct.unpack('<IBBHHHIfff',data[4:-2])
                assert version == 1 and route == self.route and session != 0
                self.last = (seq, session)
                self.command_frames.append((time.monotonic(), flags, ttl, vx, vy, wz))
                del self.buffer[:34]
            elif self.buffer[:2] == b'SP':
                if len(self.buffer) < 29:
                    break
                data = bytes(self.buffer[:29])
                assert crc(data[:-2],65535,0x8408,16) == struct.unpack('<H',data[-2:])[0]
                self.sp_commands.append(data)
                del self.buffer[:29]
            else:
                del self.buffer[0]
        now = time.monotonic()
        if self.send_feedback and self.last and now - self.last_feedback >= .02:
            seq, session = self.last
            uptime = int((now - self.start) * 1000) if self.override_uptime is None else self.override_uptime
            os.write(self.master,packet(self.route,seq,session,uptime,self.status))
            self.last_feedback = now
        rclpy.spin_once(self.node, timeout_sec=.005)

    def pump(self, seconds, send=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if send:
                self.vel.publish(send)
            self.io()

    def set_enabled(self, enabled):
        deadline = time.monotonic() + 4
        while not self.enable.service_is_ready() and time.monotonic() < deadline:
            self.io()
        assert self.enable.service_is_ready()
        request = SetBool.Request(); request.data = enabled
        future = self.enable.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            self.io()
        assert future.done(), 'Enable service timeout'
        return future.result().success

    def logs(self):
        self.log.seek(0)
        return self.log.read()

    def close(self):
        self.proc.send_signal(2)
        try:
            self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.proc.kill(); self.proc.wait()
        self.node.destroy_node()
        os.close(self.master); os.close(self.slave)
        self.log.close()


def run(executable, route):
    c = Check(executable, route)
    try:
        deadline = time.monotonic() + 4
        while not c.measurements and time.monotonic() < deadline:
            c.pump(.05)
        assert c.command_frames and all(f[1] == 0 for f in c.command_frames)
        assert c.measurements and math.isclose(c.measurements[-1].twist.linear.x,.123,abs_tol=1e-6)
        velocity = Twist(); velocity.linear.x = 3.0; velocity.linear.y = 4.0; velocity.angular.z = 9.0
        c.pump(.1, velocity)  # A command before enable must not be cached for later motion.
        assert c.set_enabled(True)
        mark = len(c.command_frames);c.pump(.12)
        assert all(f[1] == 0 for f in c.command_frames[mark:])
        c.pump(.18, velocity)
        active = [f for f in c.command_frames if f[1] == 1]
        assert active
        assert math.isclose(active[-1][3],.6,abs_tol=1e-5)
        assert math.isclose(active[-1][4],.8,abs_tol=1e-5)
        assert math.isclose(active[-1][5],3.5,abs_tol=1e-5)
        c.pump(.30)
        assert c.command_frames[-1][1] == 0, 'cmd_vel dropout must emit STOP'
        bad = Twist();bad.linear.x = float('nan')
        c.pump(.05,bad);assert c.command_frames[-1][1] == 0
        if route == 2:
            assert c.proxy, 'SP proxy was not announced'
            proxy = os.open(c.proxy,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
            try:
                tty.setraw(proxy)
                wanted = sp_packet(29)
                os.write(proxy,wanted[:8]);c.pump(.02)
                os.write(proxy,wanted[8:]);c.pump(.08)
                assert wanted in c.sp_commands
                os.write(c.master,sp_packet(43));c.pump(.08)
                assert sp_packet(43) in os.read(proxy,4096)
            finally:
                os.close(proxy)
        c.pump(.08,velocity)
        # A repeated MCU timestamp must not keep the feedback watchdog alive.
        c.override_uptime = int((time.monotonic()-c.start)*1000)
        c.pump(.50,velocity)
        assert c.command_frames[-1][1] == 0, 'Replayed heartbeat prolonged motion'
        c.override_uptime = None;c.pump(.15,velocity)
        assert c.command_frames[-1][1] == 0, 'Feedback recovery must not auto-enable'
        assert c.set_enabled(True)
        c.pump(.12,velocity)
        assert c.command_frames[-1][1] == 1
        c.status = 73 | 32  # Manual takeover on the MCU.
        c.pump(.12,velocity)
        assert c.command_frames[-1][1] == 0
        print('PASS route',route,': handshake, enable, limits, TTL, invalid cmd, replay, manual, SP proxy')
    except Exception:
        print('FAIL route',route,'frames',len(c.command_frames),'last',c.last,
              'measurements',len(c.measurements), 'diag:',c.diag,'log:',c.logs(), flush=True)
        raise
    finally:
        c.close()


if __name__ == '__main__':
    rclpy.init()
    failed = []
    try:
        for route in (1,2):
            try:
                run(sys.argv[1],route)
            except Exception as exc:
                failed.append((route, type(exc).__name__, str(exc)))
    finally:
        rclpy.shutdown()
    if failed:
        print('FAILED_ROUTES', failed, flush=True)
        sys.exit(1)
