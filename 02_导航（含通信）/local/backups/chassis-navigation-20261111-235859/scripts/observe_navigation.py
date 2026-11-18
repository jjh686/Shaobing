#!/usr/bin/env python3
"""Observe topics, timestamps, TF and lifecycle; never publish control or enable."""
import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='red_standard_robot1')
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if not 1 <= args.duration <= 300:
        parser.error('duration must be between 1 and 300 seconds')
    ns = '/'+args.namespace.strip('/') if args.namespace.strip('/') else ''
    rclpy.init()
    node = rclpy.create_node('sentry_navigation_observer', namespace=ns,
        cli_args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    report = {'namespace': ns, 'duration_seconds': args.duration,
              'note': 'Read-only observer; no velocity, goals or enable requests published.',
              'topics': {}, 'tf': {}, 'lifecycle': {}}
    subscriptions, latest, samples = [], {}, {}

    def receive(name, message):
        now = time.monotonic()
        state = samples.setdefault(name, {'count': 0, 'first': now, 'last': now,
            'max_gap_seconds': 0.0, 'timestamp_regressions': 0, 'repeated_timestamps': 0})
        if state['count']:
            state['max_gap_seconds'] = max(state['max_gap_seconds'], now-state['last'])
        state['count'] += 1
        state['last'] = now
        latest[name] = message
        if hasattr(message, 'header'):
            stamp = message.header.stamp.sec*1000000000+message.header.stamp.nanosec
            if state.get('stamp_ns') is not None:
                state['timestamp_regressions'] += int(stamp < state['stamp_ns'])
                state['repeated_timestamps'] += int(stamp == state['stamp_ns'])
            state['stamp_ns'] = stamp
            state['frame'] = message.header.frame_id
        if isinstance(message, PointCloud2):
            state['last_point_count'] = message.width*message.height
        if isinstance(message, OccupancyGrid):
            state['grid_size'] = [message.info.width, message.info.height]
            state['resolution_m'] = message.info.resolution

    kinds = [('livox/imu', Imu), ('livox/lidar/pointcloud', PointCloud2),
             ('cloud_registered', PointCloud2), ('registered_scan', PointCloud2),
             ('odometry', Odometry), ('aft_mapped_to_init', Odometry),
             ('mid360_test/scan', LaserScan), ('map', OccupancyGrid),
             ('gimbal_joint_state', JointState),
             ('cmd_vel_gimbal_stamped', TwistStamped), ('cmd_vel_body', TwistStamped),
             ('diagnostics', DiagnosticArray), ('frame_ready', Bool)]
    for name, kind in kinds:
        qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          reliability=ReliabilityPolicy.RELIABLE)
               if kind is OccupancyGrid else qos_profile_sensor_data)
        subscriptions.append(node.create_subscription(kind, name,
            lambda msg, name=name: receive(name, msg), qos))
    try:
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        for name, _ in kinds:
            entry = dict(samples.get(name, {'count': 0}))
            if entry['count'] > 1:
                entry['received_hz'] = (entry['count']-1)/(entry['last']-entry['first'])
            if entry['count']:
                entry['last_arrival_age_seconds'] = time.monotonic()-entry['last']
            entry.pop('first', None)
            entry.pop('last', None)
            if entry.get('stamp_ns'):
                entry['last_stamp_age_seconds'] = (node.get_clock().now().nanoseconds-entry['stamp_ns'])/1e9
            report['topics'][name] = entry
        if 'diagnostics' in latest:
            report['diagnostics'] = [{'message': status.message,
                'values': {item.key: item.value for item in status.values}}
                for status in latest['diagnostics'].status]
        if 'frame_ready' in latest:
            report['frame_ready'] = latest['frame_ready'].data
        tf_pairs = ([('base_footprint', 'front_mid360'), ('base_footprint', 'gimbal_yaw'),
                     ('gimbal_yaw', 'gimbal_pitch'), ('odom', 'base_footprint'),
                     ('odom', 'gimbal_yaw_fake'), ('map', 'odom')] if ns else
                    [('map', 'camera_init'), ('camera_init', 'aft_mapped'), ('aft_mapped', 'body')])
        for parent, child in tf_pairs:
            try:
                tf = buffer.lookup_transform(parent, child, rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                report['tf'][parent+' <- '+child] = {
                    'xyz': [t.x, t.y, t.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w],
                    'stamp_ns': tf.header.stamp.sec*1000000000+tf.header.stamp.nanosec}
            except Exception as error:
                report['tf'][parent+' <- '+child] = {'unavailable': str(error)}
        pending = {}
        for name in ('map_server', 'controller_server', 'planner_server', 'bt_navigator'):
            client = node.create_client(GetState, name+'/get_state')
            if client.service_is_ready():
                pending[name] = (client, client.call_async(GetState.Request()))
            else:
                report['lifecycle'][name] = 'service unavailable'
        deadline = time.monotonic()+2
        while any(not future.done() for _, future in pending.values()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        for name, (_, future) in pending.items():
            report['lifecycle'][name] = future.result().current_state.label if future.done() and future.result() else 'timeout'
        report['nodes'] = sorted('/'+namespace.strip('/')+'/'+name if namespace != '/' else '/'+name
            for name, namespace in node.get_node_names_and_namespaces())
        output = Path(args.output) if args.output else Path(os.environ['RM_NAV_ROOT'])/'local/runtime-observation.json'
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
