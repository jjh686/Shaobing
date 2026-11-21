"""Real SLAM/Nav2/TF nodes with synthetic LIO outputs; USB remains unopened."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, UInt8MultiArray
from std_srvs.srv import Trigger
import yaml
from rm_live_nav.geometry import matrix_quaternion, rpy_matrix


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '97' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    root, runtime = Path(os.environ['RM_NAV_ROOT']), Path(os.environ['RM_NAV_RUNTIME'])
    mapping_only = '--mapping-only' in sys.argv
    artifact = Path(tempfile.mkdtemp(prefix='synthetic-live-', dir=runtime))
    mount = dict(configured=True, fixed_gimbal=True, footprint_confirmed=True,
        parent_frame='base_footprint', child_frame='front_mid360',
        translation_m=[-.044, -.04425, .39336], rotation_rpy_rad=[0., math.pi/4, .7],
        footprint_xy_m=[[.29666, .46458], [-.23234, .46458], [-.23234, -.18172], [.29666, -.18172]])
    (artifact/'mount.yaml').write_text(yaml.safe_dump(mount))
    log = (artifact/'launch.log').open('w')
    process = subprocess.Popen(['bash', str(root/'scripts/run_live_exploration.sh'), 'mapping' if mapping_only else 'preview',
        'start_lidar:=false', 'mounting_config:='+str(root/'local/live_mounting.yaml' if mapping_only else artifact/'mount.yaml'),
        'output_dir:='+str(artifact/'maps')],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('live_pipeline_test')
    report = {'synthetic_only': True, 'physical_usb_opened': False, 'mapping_only': mapping_only, 'artifact': str(artifact)}
    counts, latest, subscriptions = {}, {}, []
    enabled_frames = 0
    peak_controller_speed = 0.
    def receive(name, msg):
        nonlocal enabled_frames, peak_controller_speed
        counts[name] = counts.get(name, 0)+1
        latest[name] = msg
        if name == 'tx_preview' and len(msg.data) == 34:
            enabled_frames += int(struct.unpack_from('<H', bytes(msg.data), 14)[0] != 0)
        if name == 'live/cmd_vel_raw':
            peak_controller_speed = max(peak_controller_speed, math.hypot(msg.linear.x, msg.linear.y))
    for topic, type_, qos in [('live/odometry', Odometry, 5), ('live/scan', LaserScan, qos_profile_sensor_data),
                             ('live/cmd_vel_raw', Twist, 5), ('tx_preview', UInt8MultiArray, 5),
                             ('map', OccupancyGrid, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
        subscriptions.append(node.create_subscription(type_, topic, lambda msg, name=topic: receive(name, msg), qos))
    odom_pub = node.create_publisher(Odometry, 'aft_mapped_to_init', 5)
    cloud_pub = node.create_publisher(PointCloud2, 'cloud_registered', 5)
    raw_pub = node.create_publisher(PointCloud2, 'livox/lidar/pointcloud', 5)
    fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
    angles = np.linspace(-math.pi, math.pi, 1440, endpoint=False)
    distance = 4./np.maximum(np.abs(np.cos(angles)), np.abs(np.sin(angles)))
    # Room walls plus an internal obstacle at x=1, y=0.
    walls = np.column_stack((distance*np.cos(angles), distance*np.sin(angles), np.full(len(angles), .4), np.full(len(angles), 100.)))
    obstacle = np.array([(x, y, .4, 100.) for x in np.arange(.9, 1.11, .03) for y in np.arange(-.3, .31, .03)])
    points = np.vstack((walls, obstacle)).astype(np.float32)
    base_lidar_r = rpy_matrix(mount['rotation_rpy_rad'])
    imu_position = np.array(mount['translation_m']) - base_lidar_r @ np.array([-.011, -.02329, .04412])
    imu_q = matrix_quaternion(base_lidar_r)
    last_publish = 0.
    publish_count = 0
    def pump(seconds):
        nonlocal last_publish, publish_count
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            assert process.poll() is None, 'Launch exited: '+str(artifact/'launch.log')
            if time.monotonic()-last_publish > .05:
                last_publish = time.monotonic()
                publish_count += 1
                stamp = node.get_clock().now().to_msg()
                odom = Odometry()
                odom.header, odom.child_frame_id = Header(stamp=stamp, frame_id='camera_init'), 'body'
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, imu_position)
                odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = map(float, imu_q)
                odom_pub.publish(odom)
                # Observe the room first, then add an obstacle to test live updates.
                current_points = walls.astype(np.float32) if publish_count < 60 else points
                cloud = point_cloud2.create_cloud(Header(stamp=stamp, frame_id='camera_init'), fields, current_points)
                cloud_pub.publish(cloud)
                raw_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id='front_mid360'), fields, points))
            rclpy.spin_once(node, timeout_sec=.01)
    def wait_future(future, timeout=8.):
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline:
            pump(.05)
        assert future.done(), 'Future timed out'
        return future.result()
    try:
        states = {name: node.create_client(GetState, name+'/get_state') for name in (() if mapping_only else ('controller_server', 'planner_server', 'bt_navigator'))}
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            pump(.2)
            if counts.get('map', 0) and all(client.service_is_ready() for client in states.values()):
                active = {name: wait_future(client.call_async(GetState.Request())).current_state.id for name, client in states.items()}
                if all(value == 3 for value in active.values()):
                    break
        else:
            raise AssertionError('Map or active Nav2 missing; '+str(counts))
        report['lifecycle'] = active
        pump(5.)
        assert counts.get('live/scan', 0) >= 10 and counts.get('live/odometry', 0) >= 10, str(counts)
        pose = latest['live/odometry'].pose.pose
        if mapping_only:
            assert latest['live/odometry'].child_frame_id == 'mapping_base'
            assert abs(pose.orientation.x)+abs(pose.orientation.y) < 1e-5, 'Mapping frame must be horizontal despite tilted radar'
            saver = node.create_client(Trigger, 'mid360_test/save_map')
            saved = wait_future(saver.call_async(Trigger.Request()))
            assert saved.success and (artifact/'maps/map.pcd').is_file() and (artifact/'maps/map.yaml').is_file(), saved.message
            assert counts.get('tx_preview', 0) == 0, 'Mapping-only must not launch chassis communication'
            report.update(passed=True, counts=counts, no_chassis_node=True, horizontal_scan_frame=True, map_saved=True)
            return
        assert abs(pose.position.x)+abs(pose.position.y)+abs(pose.position.z) < 1e-5, str(pose)
        assert abs(pose.orientation.x)+abs(pose.orientation.y)+abs(pose.orientation.z) < 1e-5
        planner = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
        deadline = time.monotonic()+5
        while not planner.server_is_ready() and time.monotonic() < deadline:
            pump(.1)
        assert planner.server_is_ready()
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped(header=Header(frame_id='map', stamp=node.get_clock().now().to_msg()))
        goal.goal.pose.position.x, goal.goal.pose.orientation.w = 2., 1.
        goal.planner_id = 'GridBased'
        handle = wait_future(planner.send_goal_async(goal))
        assert handle.accepted
        result = wait_future(handle.get_result_async())
        assert result.status == 4 and len(result.result.path.poses) > 2, str(result)
        path = result.result.path
        # The planner must route around the obstacle, not drive through x=1,y=0.
        near = [p.pose.position for p in path.poses if .7 < p.pose.position.x < 1.3]
        assert near and all(abs(p.y) > .3 for p in near), 'Planner crossed internal obstacle'
        report['planner'] = {'status': result.status, 'path_poses': len(path.poses), 'obstacle_detour': True}
        follower = ActionClient(node, FollowPath, 'follow_path')
        pump(.2)
        command_count = counts.get('live/cmd_vel_raw', 0)
        follow = FollowPath.Goal(path=path, controller_id='FollowPath', goal_checker_id='goal_checker')
        follow_handle = wait_future(follower.send_goal_async(follow))
        assert follow_handle.accepted
        pump(1.)
        assert counts.get('live/cmd_vel_raw', 0) > command_count, 'No real controller output'
        report['peak_controller_speed_m_s'] = peak_controller_speed
        assert 0.001 < peak_controller_speed <= .1001, f'Controller did not produce a bounded nonzero velocity: {peak_controller_speed}'
        wait_future(follow_handle.cancel_goal_async())
        saver = node.create_client(Trigger, 'mid360_test/save_map')
        saved = wait_future(saver.call_async(Trigger.Request()))
        assert saved.success and (artifact/'maps/map.pcd').is_file() and (artifact/'maps/map.yaml').is_file(), saved.message
        assert counts.get('tx_preview', 0) > 10 and enabled_frames == 0
        report.update(passed=True, counts=counts, motion_enabled_frames=enabled_frames,
                      peak_controller_speed_m_s=peak_controller_speed, map_saved=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        log.close()
        (root/('local/live-mapping-test-20261121.json' if mapping_only else 'local/live-pipeline-test-20261121.json')).write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
