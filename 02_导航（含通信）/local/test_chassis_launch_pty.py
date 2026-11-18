#!/usr/bin/env python3
"""Exercise the fixed-chassis launch against a PTY; never opens physical USB.

Run: source scripts/local_env.sh; python3 local/test_chassis_launch_pty.py
ROS is isolated in domain 96 and restricted to localhost before importing rclpy.
"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback
import tty

os.environ['ROS_DOMAIN_ID'] = '96'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'local/chassis-launch-pty-test-20261112.json'
LOG = ROOT / 'local/chassis-launch-pty-test-20261112.log'


# Same wire encoder as src/rm_chassis_comm/test/test_ros_pty.py.
def crc(data, initial, polynomial, width):
    value = initial
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (polynomial if value & 1 else 0)
    return value & ((1 << width) - 1)


def feedback_packet(sequence, session, uptime, status=73):
    head = bytes([0x5a, 28, 0x21])
    head += bytes([crc(head, 255, 0x8c, 8)])
    data = head + struct.pack('<IBBHHHIfff', uptime, 1, 2, sequence,
                              status, 0, session, .123, 0, 0)
    return data + struct.pack('<H', crc(data, 65535, 0x8408, 16))


class Test:
    def __init__(self):
        self.report = {'test': 'actual chassis_navigation launch with PTY',
                       'hardware_accessed': False, 'ros_domain_id': 96,
                       'ros_localhost_only': True, 'steps': [], 'passed': False}
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.pty = os.ttyname(self.slave)
        assert self.pty.startswith('/dev/pts/')
        self.ns = '/chassis_launch_pty_' + str(os.getpid())
        self.node = rclpy.create_node('chassis_launch_pty_test_' + str(os.getpid()))
        self.velocity = self.node.create_publisher(Twist, self.ns + '/cmd_vel_nav2_result', 1)
        self.wrong_velocity = self.node.create_publisher(Twist, self.ns + '/cmd_vel', 1)
        self.diag = None
        self.measurements = []
        self.transforms = []
        self.subscriptions = [
            self.node.create_subscription(DiagnosticArray, self.ns + '/diagnostics',
                lambda m: setattr(self, 'diag', m), 10),
            self.node.create_subscription(TwistStamped, self.ns + '/measured_velocity',
                self.measurements.append, 10),
            self.node.create_subscription(TFMessage, self.ns + '/tf_static',
                lambda m: self.transforms.extend(m.transforms),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        self.enable = self.node.create_client(SetBool, self.ns + '/set_enabled')
        self.buffer = bytearray()
        self.frames = []
        self.last_ack = None
        self.send_feedback = False
        self.status = 73  # READY | MEASURED | ACK_VALID
        self.last_feedback = 0
        self.start = time.monotonic()
        self.last_publish = 0
        command = [
            'ros2', 'launch', 'rm_chassis_comm', 'chassis_navigation.launch.py',
            'namespace:=' + self.ns[1:], 'connect:=true', 'serial_port:=' + self.pty,
            'start_navigation:=false', 'start_lidar:=false',
            'mounting_config:=' + str(ROOT / 'local/measured_mounting.yaml'),
            'nav_params:=' + str(ROOT / 'local/chassis_nav2_params.yaml'),
        ]
        self.report['command'] = command
        self.log = LOG.open('w')
        self.proc = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def io(self):
        assert self.proc.poll() is None, 'Launch exited; see ' + str(LOG)
        try:
            self.buffer.extend(os.read(self.master, 4096))
        except BlockingIOError:
            pass
        while len(self.buffer) >= 34:
            data = bytes(self.buffer[:34])
            if data[:3] != bytes([0x5a, 28, 0x20]) or \
                    crc(data[:3], 255, 0x8c, 8) != data[3] or \
                    crc(data[:-2], 65535, 0x8408, 16) != struct.unpack('<H', data[-2:])[0]:
                del self.buffer[0]
                continue
            uptime, version, route, seq, ttl, flags, session, vx, vy, wz = \
                struct.unpack('<IBBHHHIfff', data[4:-2])
            assert version == 1 and route == 2 and session != 0
            self.last_ack = (seq, session)
            self.frames.append({'at': time.monotonic(), 'flags': flags, 'ttl_ms': ttl,
                                'vx': vx, 'vy': vy, 'wz': wz})
            del self.buffer[:34]
        now = time.monotonic()
        if self.send_feedback and self.last_ack and now - self.last_feedback >= .02:
            seq, session = self.last_ack
            os.write(self.master, feedback_packet(seq, session, int((now - self.start) * 1000), self.status))
            self.last_feedback = now
        rclpy.spin_once(self.node, timeout_sec=.005)

    def pump(self, seconds, velocity=None, wrong=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if velocity is not None and time.monotonic() - self.last_publish >= .02:
                (self.wrong_velocity if wrong else self.velocity).publish(velocity)
                self.last_publish = time.monotonic()
            self.io()

    def wait_for(self, predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.io()
        assert predicate(), 'Timed out waiting for expected ROS/PTY state'

    def set_enabled(self, enabled):
        self.wait_for(self.enable.service_is_ready)
        request = SetBool.Request()
        request.data = enabled
        future = self.enable.call_async(request)
        self.wait_for(future.done)
        return future.result()

    def passed(self, step, **data):
        item = {'step': step, 'passed': True, **data}
        self.report['steps'].append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def only_stop(self, since):
        frames = self.frames[since:]
        assert frames and all(f['flags'] == 0 and f['vx'] == f['vy'] == f['wz'] == 0 for f in frames), frames

    def run(self):
        self.wait_for(lambda: len(self.frames) >= 5 and self.diag and self.transforms)
        self.only_stop(0)
        assert not self.set_enabled(True).success, 'Enable accepted before actual STOP ACK'
        self.passed('default STOP and enable rejected without feedback', frames=len(self.frames))

        self.send_feedback = True
        self.wait_for(lambda: bool(self.measurements))
        self.pump(1.1)
        diag = {p.key: p.value for p in self.diag.status[0].values}
        expected = {'route': '2', 'frame_id': 'base_footprint',
                    'use_stamped_commands': 'false', 'require_gimbal_state': 'false',
                    'require_frame_ready': 'false', 'enabled': 'false'}
        assert all(diag.get(k) == v for k, v in expected.items()), diag
        assert self.measurements[-1].header.frame_id == 'base_footprint'
        transform = next(t for t in self.transforms if t.child_frame_id == 'front_mid360')
        assert transform.header.frame_id == 'base_footprint'
        assert math.isclose(transform.transform.translation.z, .20, abs_tol=1e-8)
        graph = self.node.get_node_names_and_namespaces()
        actual = [name for name, ns in graph if ns == self.ns]
        assert sorted(actual) == ['chassis_comm', 'chassis_lidar_mount'], actual
        subscriptions = [name for name, _ in self.node.get_subscriber_names_and_types_by_node(
            'chassis_comm', self.ns)]
        assert self.ns + '/cmd_vel_nav2_result' in subscriptions, subscriptions
        assert self.ns + '/cmd_vel' not in subscriptions, subscriptions
        self.passed('launch profile, real remap, static lidar TF and no gimbal dependencies',
                    diagnostics=expected, nodes=actual, subscription_topics=subscriptions)

        velocity = Twist()
        velocity.linear.x, velocity.linear.y, velocity.angular.z = 3., 4., 9.
        self.pump(.12, velocity)  # Pre-enable command must not survive enable.
        assert self.set_enabled(True).success
        mark = len(self.frames)
        self.pump(.12)
        self.only_stop(mark)
        self.pump(.18, velocity, wrong=True)
        self.only_stop(mark)
        self.passed('no cached velocity; original unremapped cmd_vel ignored')

        mark = len(self.frames)
        self.pump(.20, velocity)
        active = [f for f in self.frames[mark:] if f['flags'] == 1]
        assert active, 'Nav2 result topic did not produce ENABLE command'
        for f in active:
            assert math.isclose(f['vx'], .18, abs_tol=1e-5), f
            assert math.isclose(f['vy'], .24, abs_tol=1e-5), f
            assert math.isclose(f['wz'], .5, abs_tol=1e-5), f
            assert 20 <= f['ttl_ms'] <= 150, f
        self.passed('Nav2 Twist -> route2 wire limits and TTL', sample=active[-1],
                    linear_norm=math.hypot(active[-1]['vx'], active[-1]['vy']))

        stale_start = self.last_publish
        self.pump(.30)
        stop = next(f for f in self.frames[mark:] if f['at'] > stale_start and f['flags'] == 0)
        latency = stop['at'] - stale_start
        assert latency <= .20, latency
        assert self.frames[-1]['flags'] == 0
        self.passed('150ms input watchdog emits STOP', observed_stop_after_last_publish_ms=latency*1000)

        self.pump(.15, velocity)
        assert self.frames[-1]['flags'] == 1
        feedback_cut = self.last_feedback
        mark = len(self.frames)
        self.send_feedback = False
        self.pump(.40, velocity)
        stop = next(f for f in self.frames[mark:] if f['flags'] == 0)
        latency = stop['at'] - feedback_cut
        assert latency <= .31, latency
        assert self.frames[-1]['flags'] == 0
        self.send_feedback = True
        mark = len(self.frames)
        self.pump(.20, velocity)
        self.only_stop(mark)
        self.passed('250ms feedback watchdog and no automatic enable after recovery',
                    observed_stop_after_last_feedback_ms=latency*1000)

        assert self.set_enabled(True).success
        self.pump(.15, velocity)
        assert self.frames[-1]['flags'] == 1
        self.status = 73 | 32
        mark = len(self.frames)
        manual_start = time.monotonic()
        self.pump(.15, velocity)
        stop = next(f for f in self.frames[mark:] if f['flags'] == 0)
        assert stop['at'] - manual_start < .08
        assert self.frames[-1]['flags'] == 0
        assert not self.set_enabled(True).success
        self.status = 73
        mark = len(self.frames)
        self.pump(.15, velocity)
        self.only_stop(mark)
        self.passed('MANUAL takeover stops, rejects enable, and recovery remains disabled',
                    observed_stop_ms=(stop['at']-manual_start)*1000)

        assert self.set_enabled(True).success
        self.pump(.15, velocity)
        assert self.frames[-1]['flags'] == 1
        assert self.set_enabled(False).success
        mark = len(self.frames)
        self.pump(.10, velocity)
        self.only_stop(mark)
        self.passed('explicit disable emits zero STOP despite continuing Nav2 input')
        self.report['passed'] = True

    def close(self):
        if self.proc.poll() is None:
            # Let ros2 launch forward SIGINT once to its own children.
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.report['launch_exit_code_after_sigint'] = self.proc.returncode
        self.report['frames_received'] = len(self.frames)
        self.report['measured_messages'] = len(self.measurements)
        self.report['launch_log'] = str(LOG)
        self.report['duration_s'] = time.monotonic() - self.start
        self.log.close()
        self.node.destroy_node()
        os.close(self.master)
        os.close(self.slave)
        REPORT.write_text(json.dumps(self.report, ensure_ascii=False, indent=2) + '\n')


def main():
    rclpy.init()
    test = Test()
    try:
        test.run()
    except Exception:
        test.report['error'] = traceback.format_exc()
        traceback.print_exc()
    finally:
        test.close()
        rclpy.shutdown()
    print('PASS' if test.report['passed'] else 'FAIL', str(REPORT), flush=True)
    return 0 if test.report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
