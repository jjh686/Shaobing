#!/usr/bin/env python3
"""Run an offline ROS closed-loop check with Navfn and a fake motion action server.

Use an unused ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1. Source the built
rm2027_decision overlay before running this file. The test starts no hardware nodes.
"""

import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
from collections import deque
from contextlib import ExitStack, contextmanager
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rm2027_decision.msg import DecisionIntent, NavigationEvidence
from std_msgs.msg import String, UInt8MultiArray
from tf2_ros import TransformBroadcaster

SOURCE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SOURCE / 'scripts' / 'adapters'))
from referee_rx import crc


ACTION = '/rm_loop_check/navigate_to_pose'
DECISION = '/rm_loop_check'
TARGET = (-4.75, 3.0)
START = (-4.0, 2.8)


def referee_frame(command, payload):
    sof = struct.pack('<BHB', 0xa5, len(payload), 0)
    body = sof + bytes([crc(sof, 0xff, 0x8c)]) + struct.pack('<H', command) + payload
    return body + struct.pack('<H', crc(body, 0xffff, 0x8408))


def executable(package, name):
    return str(Path(get_package_prefix(package)) / 'lib' / package / name)


def write_tasks(path, *, patrol=('start_marker', *START)):
    patrol_id, patrol_x, patrol_y = patrol
    path.write_text(yaml.safe_dump({'map_frame': 'map', 'objectives': [
        {'id': 'red_supply', 'kind': 'recovery', 'x': TARGET[0], 'y': TARGET[1], 'risk': 0.1},
        {'id': patrol_id, 'kind': 'patrol', 'x': patrol_x, 'y': patrol_y, 'risk': 0.1},
        {'id': 'blocked', 'kind': 'patrol', 'x': -6.15, 'y': 4.15, 'risk': 0.1},
    ]}), encoding='utf-8')


@contextmanager
def process(command, log_path):
    with log_path.open('w+', encoding='utf-8') as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            yield child
        except (AssertionError, RuntimeError):
            log.seek(0)
            output = log.read()
            if output:
                print(f'{log_path.name}:\n{output}', file=sys.stderr)
            raise
        finally:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=4)


class Harness(Node):
    def __init__(self, *, mock_motion=True):
        super().__init__('rm_closed_loop_check')
        self.clock = self.get_clock()
        self.status = deque(maxlen=1024)
        self.intents = deque(maxlen=1024)
        self.evidence = None
        self.verified_targets = set()
        self.pose = None
        self.canceled = False
        self.send_pose = True
        self.phase = 0
        self.hp, self.max_hp = 80, 400
        self.tf = TransformBroadcaster(self)
        self.raw = self.create_publisher(UInt8MultiArray, DECISION + '/referee_bytes', 10)
        self.create_subscription(String, DECISION + '/navigation_status',
                                 lambda m: self.status.append(m.data), 10)
        self.create_subscription(NavigationEvidence, DECISION + '/navigation_evidence',
                                 self.on_evidence, qos_profile_sensor_data)
        self.create_subscription(DecisionIntent, DECISION + '/intent', self.intents.append, 10)
        self.motion = None
        if mock_motion:
            self.motion = ActionServer(self, NavigateToPose, ACTION, self.on_goal,
                                       callback_group=ReentrantCallbackGroup(),
                                       cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(0.05, self.inputs)

    def on_evidence(self, message):
        self.evidence = message
        self.verified_targets.update(point.id for point in message.objectives if point.route_verified)

    def routes_verified(self):
        evidence = self.evidence
        if evidence is None or not evidence.localization_valid or not evidence.objectives_valid:
            return False
        points = {point.id: point for point in evidence.objectives if point.route_verified}
        return ('red_supply' in points and not points['red_supply'].self_present
                and 'start_marker' in points and points['start_marker'].self_present)

    def on_goal(self, handle):
        self.pose = handle.request.pose
        deadline = time.monotonic() + 12
        while not handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.02)
        if handle.is_cancel_requested:
            handle.canceled()
            self.canceled = True
        else:
            handle.abort()
        return NavigateToPose.Result()

    def inputs(self):
        if self.send_pose:
            pose = TransformStamped()
            pose.header.stamp = self.clock.now().to_msg()
            pose.header.frame_id = 'map'
            pose.child_frame_id = 'base_footprint'
            pose.transform.translation.x, pose.transform.translation.y = START
            pose.transform.rotation.w = 1.0
            self.tf.sendTransform(pose)
        if self.phase:
            game = bytes([(self.phase << 4) | 4]) + struct.pack('<HQ', 280, 0)
            robot = bytearray(17)
            robot[0] = 7
            struct.pack_into('<HHHHHfB', robot, 2, self.hp, self.max_hp, 30, 260, 100, 25.0, 7)
            stream = referee_frame(0x0001, game) + referee_frame(0x0201, robot)
            self.raw.publish(UInt8MultiArray(data=list(stream)))


def wait_until(executor, condition, description, children, timeout=12):
    deadline = time.monotonic() + timeout
    while not condition():
        ended = [child.returncode for child in children if child.poll() is not None]
        if ended:
            raise RuntimeError(f'Processes exited during {description}: {ended}')
        if time.monotonic() >= deadline:
            raise AssertionError(f'Timed out waiting for {description}')
        executor.spin_once(timeout_sec=0.05)


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise SystemExit('Set ROS_LOCALHOST_ONLY=1 and an unused ROS_DOMAIN_ID.')
    rclpy.init()
    harness = Harness()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(harness)
    try:
        with tempfile.TemporaryDirectory(prefix='rm-loop-check-') as scratch, ExitStack() as cleanup:
            temp = Path(scratch)
            map_path = SOURCE / 'maps' / 'offline' / 'red_map.yaml'
            tasks_path = temp / 'tasks.yaml'
            write_tasks(tasks_path)
            params_path = temp / 'planner.yaml'
            nav2_share = Path(get_package_share_directory('nav2_bringup'))
            defaults = yaml.safe_load((nav2_share / 'params' / 'nav2_params.yaml').read_text())
            planner = defaults['planner_server']['ros__parameters']
            planner.update(use_sim_time=False)
            planner[planner['planner_plugins'][0]].update(allow_unknown=False, tolerance=0.0)
            global_costmap = defaults['global_costmap']['global_costmap']['ros__parameters']
            global_costmap.update(use_sim_time=False, robot_base_frame='base_footprint',
                                  plugins=['static_layer', 'inflation_layer'])
            params_path.write_text(yaml.safe_dump({
                'planner_server': defaults['planner_server'],
                'global_costmap': defaults['global_costmap'],
            }), encoding='utf-8')

            children = []

            def start(package, program, *arguments):
                command = [executable(package, program), '--ros-args', *arguments]
                children.append(cleanup.enter_context(process(command, temp / f'{program}.log')))

            start('nav2_map_server', 'map_server', '-p', f'yaml_filename:={map_path}')
            start('nav2_planner', 'planner_server', '--params-file', str(params_path))
            start('nav2_lifecycle_manager', 'lifecycle_manager', '-p', 'autostart:=true',
                  '-p', 'node_names:=[map_server, planner_server]')

            namespace = ('-r', '__ns:=' + DECISION)
            start('rm2027_decision', 'navigation_evidence.py', *namespace,
                  '-p', f'tasks_file:={tasks_path}', '-p', 'protected_map_confirmed:=true')
            start('rm2027_decision', 'referee_state_adapter.py', *namespace,
                  '-p', 'vision_status:=false')
            start('rm2027_decision', 'rm2027_tree', *namespace,
                  '--params-file', str(SOURCE / 'config' / 'decision.yaml'),
                  '-p', 'enabled:=true', '-p', f'tree_file:={SOURCE / "behavior_trees" / "intents.xml"}')
            start('rm2027_decision', 'intent_nav_bridge.py', *namespace,
                  '-p', 'execute_navigation:=true', '-p', f'tasks_file:={tasks_path}',
                  '-p', f'navigation_action:={ACTION}')

            wait_until(executor, harness.routes_verified, 'locally verified Navfn route', children, timeout=35)
            harness.phase = 3
            wait_until(executor, lambda: any(m.mode == 'WaitGame' for m in harness.intents),
                       'decision waiting for match', children)
            harness.phase = 4
            wait_until(executor, lambda: 'active:red_supply' in harness.status,
                       f'accepted recovery goal {TARGET}', children)
            assert harness.pose is not None and harness.pose.header.frame_id == 'map'
            assert math.isclose(harness.pose.pose.position.x, TARGET[0])
            assert math.isclose(harness.pose.pose.position.y, TARGET[1])
            harness.send_pose = False
            wait_until(executor, lambda: harness.canceled and 'canceled:red_supply' in harness.status,
                       'motion goal canceled after TF stopped', children)
            wait_until(executor, lambda: any(m.mode == 'LocalizationLost' for m in harness.intents),
                       'decision reporting stale localization', children)
            assert harness.verified_targets == {'red_supply', 'start_marker'}, harness.verified_targets
            print('PASS: Navfn on offline 3V3 map, position confirmed, blocked destination excluded, '
                  'recovery goal canceled on stale TF.')
    finally:
        executor.shutdown(timeout_sec=14.0)
        harness.motion.destroy()
        harness.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
