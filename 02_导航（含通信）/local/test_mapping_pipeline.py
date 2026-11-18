"""Synthetic square room through real scan/SLAM/save nodes; no lidar or chassis.

Run only in a spare local ROS domain. Synthetic TF exists only during this test.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
import yaml


assert os.environ.get('ROS_DOMAIN_ID') == '96' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
root = Path(__file__).resolve().parents[1]
(root/'local/test-artifacts').mkdir(exist_ok=True)
output = Path(tempfile.mkdtemp(prefix='synthetic-pipeline-', dir=root/'local/test-artifacts'))
log = (root/'local/mapping-pipeline.log').open('w')
process = subprocess.Popen(['bash', str(root/'scripts/run_mapping.sh'),
    'start_lidar:=false', 'output_dir:='+str(output)], stdout=log,
    stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init()
node = rclpy.create_node('synthetic_mapping_test')
counts = {'scans': 0, 'grids': 0}
report = {'synthetic_data_only': True, 'output': str(output)}
try:
    tf = TransformStamped()
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'camera_init', 'aft_mapped'
    tf.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(tf)
    from sensor_msgs.msg import PointCloud2
    publisher = node.create_publisher(PointCloud2, 'cloud_registered', 5)
    from rclpy.qos import qos_profile_sensor_data
    def receive_scan(message):
        counts['scans'] += 1
    def receive_grid(message):
        counts['grids'] += 1
    scan_subscription = node.create_subscription(LaserScan, 'mid360_test/scan', receive_scan, qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    grid_subscription = node.create_subscription(OccupancyGrid, 'map', receive_grid, latched)
    client = node.create_client(Trigger, 'mid360_test/save_map')
    angles = np.linspace(-math.pi, math.pi, 1440, endpoint=False)
    distance = 2. / np.maximum(np.abs(np.cos(angles)), np.abs(np.sin(angles)))
    points = np.column_stack((distance*np.cos(angles), distance*np.sin(angles),
                             np.full(1440,.3), np.full(1440,100.))).astype(np.float32)
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i,name in enumerate(('x','y','z','intensity'))]
    deadline = time.monotonic()+18
    while time.monotonic() < deadline:
        assert process.poll() is None, 'Mapping launcher stopped unexpectedly'
        cloud = point_cloud2.create_cloud(Header(stamp=node.get_clock().now().to_msg(), frame_id='camera_init'), fields, points)
        publisher.publish(cloud)
        rclpy.spin_once(node, timeout_sec=.1)
        if counts['grids'] and counts['scans'] >= 10 and client.service_is_ready(): break
    assert counts['scans'] >= 10 and counts['grids'] >= 1, str(counts)
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic()+5
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    assert future.done() and future.result().success, str(future.result())
    metadata = yaml.safe_load((output/'map.yaml').read_text())
    summary = json.loads((output/'summary.json').read_text())
    assert summary['grid']['occupied_cells'] > 0 and summary['point_count'] > 0
    assert metadata['image'] == 'map.pgm' and (output/'map.pgm').is_file()
    observed = subprocess.run(['bash', str(root/'scripts/observe_navigation.sh'),
        '--namespace', '', '--duration', '2', '--output',
        str(root/'local/mapping-pipeline-observation.json')], capture_output=True,
        text=True, timeout=12)
    assert observed.returncode == 0, observed.stdout+observed.stderr
    observation = json.loads((root/'local/mapping-pipeline-observation.json').read_text())
    assert observation['topics']['map']['count'] >= 1
    assert 'xyz' in observation['tf']['map <- camera_init']
    report.update(passed=True, received=counts, saved=summary, service_message=future.result().message)
finally:
    node.destroy_node()
    rclpy.shutdown()
    if process.poll() is None: process.send_signal(signal.SIGINT)
    try:
        report['launcher_exit'] = process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        report['launcher_exit'] = process.wait(timeout=5)
    log.close()
    (root/'local/mapping-pipeline-test.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
