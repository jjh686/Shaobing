"""Synthetic full chassis navigation pipeline; opens no physical device.

Run after sourcing scripts/local_env.sh with ROS_DOMAIN_ID=97 and
ROS_LOCALHOST_ONLY=1. Inputs emulate Point-LIO outputs; this cannot establish
lidar accuracy, physical mounting, motor directions, or real stopping distance.
"""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt8MultiArray
from tf2_ros import Buffer, TransformListener
import yaml


assert os.environ.get('ROS_DOMAIN_ID') == '97'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
root = Path(__file__).resolve().parents[1]
runtime = Path(os.environ['RM_NAV_RUNTIME'])
artifact = Path(tempfile.mkdtemp(prefix='synthetic-chassis-nav-', dir=root/'local/test-artifacts'))
artifact_alias = runtime/'source/local/test-artifacts'/artifact.name
pixels = np.full((200, 200), 254, dtype=np.uint8)
pixels[[0, -1], :] = 0
pixels[:, [0, -1]] = 0
(artifact/'map.pgm').write_bytes(b'P5\n200 200\n255\n'+pixels.tobytes())
(artifact/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', mode='trinary',
    resolution=.05, origin=[-5., -5., 0.], negate=0, occupied_thresh=.65, free_thresh=.25)))
grid = np.arange(-3., 3.001, .12)
floor = np.array([(x, y, -.2, 1.) for x in grid for y in grid], dtype=np.float32)
height = np.arange(-.2, 1.5, .12)
walls = np.array([(x, edge, z, 1.) for x in grid for edge in (-3., 3.) for z in height] +
    [(edge, y, z, 1.) for y in grid for edge in (-3., 3.) for z in height], dtype=np.float32)
points = np.vstack([floor, walls])
count = len(points)
pcd = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
    f'COUNT 1 1 1\nWIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n')
(artifact/'map.pcd').write_bytes(pcd.encode('ascii')+points[:, :3].astype('<f4').tobytes())
log = (root/'local/chassis-nav-pipeline.log').open('w')
command = ['bash', str(root/'scripts/run_chassis_nav.sh'), 'check', 'autostart:=true',
    'map:='+str(artifact_alias/'map.yaml'), 'prior_pcd_file:='+str(artifact_alias/'map.pcd')]
process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
node = rclpy.create_node('synthetic_chassis_nav_test', namespace='/red_standard_robot1',
    cli_args=['--ros-args', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'])
buffer = Buffer()
listener = TransformListener(buffer, node)
counts, latest, subscriptions = {}, {}, []
report = {'synthetic_data_only': True, 'physical_port_opened': False,
    'connect': False, 'start_lidar': False, 'output': str(artifact), 'command': command,
    'limitations': 'Synthetic Point-LIO outputs; no physical lidar, USB, CAN, motors, localization accuracy, or motion enabled.'}
state_clients = {name: node.create_client(GetState, name+'/get_state') for name in (
    'map_server', 'controller_server', 'planner_server', 'smoother_server',
    'behavior_server', 'bt_navigator', 'velocity_smoother')}
state_futures, states = {}, {}
action = None
goal_handle = None
stop_frames, enabled_frames = 0, 0
max_command = [0., 0., 0.]


def receive(name, msg):
    global stop_frames, enabled_frames
    counts[name] = counts.get(name, 0)+1
    latest[name] = msg
    if name == 'tx_preview':
        payload = bytes(msg.data)
        if len(payload) == 34:
            flags = struct.unpack_from('<H', payload, 14)[0]
            stop_frames += int(flags == 0)
            enabled_frames += int(flags != 0)
            assert struct.unpack_from('<fff', payload, 20) == (0., 0., 0.), 'Preview emitted motion'
    if name == 'cmd_vel_nav2_result':
        for index, value in enumerate((msg.linear.x, msg.linear.y, msg.angular.z)):
            max_command[index] = max(max_command[index], abs(value))


for name, kind in [('odometry', Odometry), ('sensor_scan', PointCloud2),
    ('registered_scan', PointCloud2), ('terrain_map', PointCloud2),
    ('cmd_vel_controller', Twist), ('cmd_vel_nav2_result', Twist),
    ('tx_preview', UInt8MultiArray), ('diagnostics', DiagnosticArray), ('map', OccupancyGrid)]:
    qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE) if kind is OccupancyGrid else qos_profile_sensor_data)
    subscriptions.append(node.create_subscription(kind, name, lambda msg, name=name: receive(name, msg), qos))
odom_pub = node.create_publisher(Odometry, 'aft_mapped_to_init', 5)
cloud_pub = node.create_publisher(PointCloud2, 'cloud_registered', 5)
fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
    for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
last_publish = 0.


def spin(duration=.05):
    global last_publish
    assert process.poll() is None, 'Navigation launcher stopped unexpectedly'
    now = time.monotonic()
    if now-last_publish >= .1:
        last_publish = now
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='camera_init')
        odom.child_frame_id = 'body'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id='camera_init'), fields, points))
    rclpy.spin_once(node, timeout_sec=duration)


def until(condition, timeout):
    deadline = time.monotonic()+timeout
    while not condition() and time.monotonic() < deadline:
        spin()
    return condition()


try:
    deadline = time.monotonic()+35
    while time.monotonic() < deadline:
        spin()
        for name, client in state_clients.items():
            if name in state_futures and state_futures[name].done():
                result = state_futures.pop(name).result()
                states[name] = result.current_state.label if result else 'no response'
            if states.get(name) != 'active' and name not in state_futures and client.service_is_ready():
                state_futures[name] = client.call_async(GetState.Request())
        if len(states) == len(state_clients) and all(state == 'active' for state in states.values()):
            break
    report['lifecycle'] = states
    assert len(states) == len(state_clients) and all(state == 'active' for state in states.values()), str(states)
    tf = {}
    for parent, child in [('base_footprint', 'front_mid360'), ('odom', 'base_footprint'), ('map', 'odom')]:
        value = buffer.lookup_transform(parent, child, rclpy.time.Time())
        t, q = value.transform.translation, value.transform.rotation
        tf[parent+' <- '+child] = {'xyz': [t.x, t.y, t.z], 'quaternion_xyzw': [q.x, q.y, q.z, q.w]}
    report['tf'] = tf
    planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    assert until(planner.server_is_ready, 5), 'Planner action unavailable'
    goal = ComputePathToPose.Goal()
    goal.goal = PoseStamped(header=Header(stamp=node.get_clock().now().to_msg(), frame_id='map'))
    goal.goal.pose.position.x = 1.
    goal.goal.pose.orientation.w = 1.
    goal.planner_id = 'GridBased'
    sent = planner.send_goal_async(goal)
    assert until(sent.done, 5), 'Planner goal timed out'
    planner_handle = sent.result()
    assert planner_handle.accepted, 'Planner rejected goal'
    result = planner_handle.get_result_async()
    assert until(result.done, 8), 'Planner result timed out'
    path = result.result().result.path
    report['planner'] = {'status': result.result().status, 'path_poses': len(path.poses)}
    assert result.result().status == 4 and len(path.poses) > 2, str(report['planner'])
    action = ActionClient(node, FollowPath, 'follow_path')
    assert until(action.server_is_ready, 3), 'Controller action unavailable'
    follow = FollowPath.Goal()
    follow.path, follow.controller_id, follow.goal_checker_id = path, 'FollowPath', 'general_goal_checker'
    sent = action.send_goal_async(follow)
    assert until(sent.done, 3), 'Controller goal timed out'
    goal_handle = sent.result()
    assert goal_handle.accepted, 'Controller rejected goal'
    assert until(lambda: max(max_command) > .01, 4), 'No nonzero smoothed software command'
    cancel = goal_handle.cancel_goal_async()
    assert until(cancel.done, 3), 'Controller cancel timed out'
    until(lambda: False, .5)
    report['controller'] = {'accepted': True, 'cancel_requested': True, 'max_smoothed_abs_xyw': max_command}
    report['received'] = counts
    report['preview'] = {'stop_frames': stop_frames, 'enabled_frames': enabled_frames}
    if 'diagnostics' in latest:
        report['diagnostics'] = [{'message': status.message, 'values': {item.key: item.value for item in status.values}}
            for status in latest['diagnostics'].status]
    assert counts.get('odometry', 0) > 5 and counts.get('sensor_scan', 0) > 5
    assert stop_frames > 10 and enabled_frames == 0
    assert all(status['values']['enabled'] == 'false' for status in report['diagnostics'])
    report['passed'] = True
except Exception as error:
    report.update(passed=False, error=type(error).__name__+': '+str(error), received=counts,
        lifecycle=states, preview={'stop_frames': stop_frames, 'enabled_frames': enabled_frames})
finally:
    node.destroy_node()
    rclpy.shutdown()
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        report['launcher_exit'] = process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        report['launcher_exit'] = process.wait(timeout=5)
    log.close()
    (root/'local/chassis-nav-pipeline.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
raise SystemExit(0 if report.get('passed') else 1)
