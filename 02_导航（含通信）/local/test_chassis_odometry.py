#!/usr/bin/env python3
"""Regression test for sensor_scan_generation odometry twist, using synthetic ROS data only.

Run after sourcing scripts/local_env.sh and rebuilding sensor_scan_generation:
  python3 local/test_chassis_odometry.py
The test uses ROS_DOMAIN_ID=98, localhost-only DDS, and never opens hardware.
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

os.environ['ROS_DOMAIN_ID'] = '98'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / 'local/chassis-odometry-test-20261112.json'
LOG = ROOT / 'local/chassis-odometry-test-20261112.log'
TOL = 0.025


def quaternion_yaw(yaw, negate=False):
    q = (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))
    return tuple(-v for v in q) if negate else q


class Test:
    def __init__(self):
        self.report = {'test': 'sensor_scan_generation odometry twist regression',
                       'hardware_accessed': False, 'ros_domain_id': 98,
                       'ros_localhost_only': True, 'steps': [], 'passed': False}
        self.ns = '/chassis_odom_test_' + str(os.getpid())
        self.node = rclpy.create_node('chassis_odom_test_' + str(os.getpid()))
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.odom_pub = self.node.create_publisher(Odometry, self.ns + '/lidar_odometry', qos)
        self.cloud_pub = self.node.create_publisher(PointCloud2, self.ns + '/registered_scan', qos)
        self.odometry = []
        self.odom_sub = self.node.create_subscription(
            Odometry, self.ns + '/odometry', self.odometry.append, 10)
        self.tf_pub = StaticTransformBroadcaster(self.node)
        mount = TransformStamped()
        mount.header.frame_id = 'base_footprint'
        mount.child_frame_id = 'front_mid360'
        mount.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(mount)
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '98'
        env['ROS_LOCALHOST_ONLY'] = '1'
        self.log = LOG.open('w')
        self.proc = subprocess.Popen([
            'ros2', 'run', 'sensor_scan_generation', 'sensor_scan_generation_node',
            '--ros-args', '-r', '__ns:=' + self.ns,
            '-p', 'lidar_frame:=front_mid360',
            '-p', 'base_frame:=base_footprint',
            '-p', 'robot_base_frame:=base_footprint',
        ], stdout=self.log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        self.stamp_to_output = {}
        self.sent = 0

    def spin(self, duration=0.01):
        assert self.proc.poll() is None, 'sensor_scan_generation exited; see ' + str(LOG)
        rclpy.spin_once(self.node, timeout_sec=duration)

    def wait_for(self, predicate, timeout=8):
        until = time.monotonic() + timeout
        while not predicate() and time.monotonic() < until:
            self.spin(.01)
        assert predicate(), 'Timed out waiting for sensor_scan_generation output'

    def publish_sample(self, stamp_ns, parent, x, y, yaw, negate_q=False, wall_delay=0.0):
        if wall_delay:
            end = time.monotonic() + wall_delay
            while time.monotonic() < end:
                self.spin(.01)
        odom = Odometry()
        odom.header.stamp.sec, odom.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        odom.header.frame_id = parent
        odom.child_frame_id = 'front_mid360'
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        qw, qx, qy, qz = quaternion_yaw(yaw, negate_q)
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        cloud = PointCloud2()
        cloud.header = odom.header
        cloud.height = 1
        cloud.width = 1
        cloud.is_bigendian = False
        cloud.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                        for i, name in enumerate(('x', 'y', 'z'))]
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.data = struct.pack('<fff', 1., 0., 0.)
        cloud.is_dense = True
        before = len(self.odometry)
        self.odom_pub.publish(odom)
        self.cloud_pub.publish(cloud)
        self.sent += 1
        self.wait_for(lambda: len(self.odometry) > before)
        out = self.odometry[-1]
        got = (out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec)
        assert got == stamp_ns, (got, stamp_ns)
        assert out.header.frame_id == parent, (out.header.frame_id, parent)
        assert out.child_frame_id == 'base_footprint', out.child_frame_id
        self.stamp_to_output[stamp_ns] = out
        return out

    @staticmethod
    def twist(msg):
        return (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                msg.twist.twist.linear.z, msg.twist.twist.angular.z)

    def expect(self, label, msg, expected):
        actual = self.twist(msg)
        assert all(math.isfinite(a) and abs(a - e) <= TOL
                   for a, e in zip(actual, expected)), (label, actual, expected)
        item = {'step': label, 'passed': True, 'twist_x_y_z_wz': actual,
                'expected_x_y_z_wz': expected, 'tolerance': TOL}
        self.report['steps'].append(item)
        print(json.dumps(item, ensure_ascii=False), flush=True)

    def run(self):
        # Wait until the real process has discovered both input topics. DDS discovery
        # is asynchronous, so allow the first pair to be retransmitted if necessary.
        self.wait_for(lambda: self.proc.poll() is None)
        self.wait_for(lambda: self.odom_pub.get_subscription_count() > 0 and
                      self.cloud_pub.get_subscription_count() > 0)
        # Let the subscriber graph and static TF transient-local sample propagate.
        propagation_end = time.monotonic() + .5
        while time.monotonic() < propagation_end:
            self.spin(.01)

        s1 = self.publish_sample(1_000_000_000, 'odom', 0, 0, math.pi / 2)
        self.expect('first sample initializes baseline with zero velocity', s1, (0, 0, 0, 0))

        s2 = self.publish_sample(1_500_000_000, 'odom', 0, 1.0, math.pi / 2)
        self.expect('world +Y at yaw 90 degrees becomes body +X using stamp dt',
                    s2, (2.0, 0, 0, 0))

        s3 = self.publish_sample(2_500_000_000, 'odom', 0, 1.1, math.pi / 2,
                                 wall_delay=.30)
        self.expect('velocity uses header interval despite different wall interval',
                    s3, (.1, 0, 0, 0))

        s4 = self.publish_sample(3_000_000_000, 'odom_yaw_wrap', 0, 1.1, math.radians(179))
        self.expect('new parent-frame baseline before yaw wrap', s4, (0, 0, 0, 0))

        s5 = self.publish_sample(3_200_000_000, 'odom_yaw_wrap', 0, 1.1, math.radians(-179),
                                 negate_q=True)
        self.expect('yaw crosses plus/minus pi with quaternion sign flip on short path',
                    s5, (0, 0, 0, math.radians(2) / .2))

        s6 = self.publish_sample(3_400_000_000, 'odom_yaw_wrap', 0, 1.1, math.radians(-179))
        self.expect('equivalent quaternion sign change produces no angular spike',
                    s6, (0, 0, 0, 0))

        s7 = self.publish_sample(4_000_000_000, 'odom_duplicate', 1.0, 0, 0)
        self.expect('new parent-frame baseline before repeated stamp', s7, (0, 0, 0, 0))

        s8 = self.publish_sample(4_000_000_000, 'odom_duplicate', 2.0, 0, 0)
        self.expect('repeated stamp produces zero velocity and resets baseline', s8,
                    (0, 0, 0, 0))
        s9 = self.publish_sample(4_200_000_000, 'odom_duplicate', 2.1, 0, 0)
        self.expect('motion after repeated stamp uses reset pose/time baseline', s9,
                    (.5, 0, 0, 0))

        s10 = self.publish_sample(4_100_000_000, 'odom_duplicate', 10.0, 0, 0)
        self.expect('backward stamp produces zero velocity and resets baseline', s10,
                    (0, 0, 0, 0))
        s11 = self.publish_sample(4_300_000_000, 'odom_duplicate', 10.1, 0, 0)
        self.expect('motion after backward stamp uses reset pose/time baseline', s11,
                    (.5, 0, 0, 0))

        s12 = self.publish_sample(4_500_000_000, 'map', -100.0, 0, 0)
        self.expect('different parent frame starts a zero-velocity baseline', s12,
                    (0, 0, 0, 0))
        s13 = self.publish_sample(4_700_000_000, 'map', -99.9, 0, 0)
        self.expect('new parent-frame baseline supports subsequent velocity', s13,
                    (.5, 0, 0, 0))
        self.report['samples_sent'] = self.sent
        self.report['samples_received'] = len(self.odometry)
        self.report['passed'] = True

    def close(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait()
        self.report['node_exit_code_after_sigint'] = self.proc.returncode
        self.report['node_log'] = str(LOG)
        self.log.close()
        self.node.destroy_node()
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
    print(('PASS' if test.report['passed'] else 'FAIL') + ' ' + str(REPORT), flush=True)
    return 0 if test.report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
