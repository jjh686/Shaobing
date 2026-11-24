#!/usr/bin/env python3
"""Exercise the decision-to-Nav2 path with an unarmed Gazebo Fortress test base."""

import argparse
import math
import os
import shutil
import tempfile
import time
import uuid
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path

import rclpy
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from gazebo_world_fixture import write_world
from navigation_closed_loop import (
    DECISION, SOURCE, START, TARGET, Harness, executable, process, wait_until, write_tasks,
)

WORLD = 'offline_3v3'
MODEL = 'offline_base'
DRIVE = '/model/' + MODEL
WORLD_TOPICS = '/world/' + WORLD
DECISION_TF = DECISION + '/decision_tf'
ROUTE_NS = DECISION + '/route'
SCENARIO_PATROL = ('practice_patrol', 0.0, 0.0)
SCENARIO_PHASES = {'waiting': 3, 'running': 4}


class GazeboHarness(Harness):
    def __init__(self):
        super().__init__(mock_motion=False)
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.send_pose = False
        self.feed_decision_tf = True
        self.world_pose = None
        self.last_pose = 0.0
        self.joint_angles = [0.0, 0.0]
        self.joints = self.create_publisher(JointState, '/joint_states', 10)
        self.sensor_transforms = Buffer(node=self)
        self.tf_listener = TransformListener(self.sensor_transforms, self)
        self.decision_tf = self.create_publisher(TFMessage, DECISION_TF, 10)
        self.create_subscription(TFMessage, WORLD_TOPICS + '/pose/info', self.observe_world,
                                 qos_profile_sensor_data)

    def observe_world(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == MODEL:
                self.world_pose = transform.transform
                self.last_pose = time.monotonic()
                return

    @property
    def position(self):
        if self.world_pose is None:
            return None
        xyz = self.world_pose.translation
        return xyz.x, xyz.y

    def inputs(self):
        now = self.clock.now()
        if now.nanoseconds == 0:
            return
        super().inputs()
        joints = JointState(name=['gimbal_yaw_joint', 'gimbal_pitch_joint'], position=self.joint_angles)
        joints.header.stamp = now.to_msg()
        self.joints.publish(joints)
        if self.world_pose is None or time.monotonic() - self.last_pose > 0.25:
            return
        pose = TransformStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'map'
        pose.child_frame_id = 'base_footprint'
        pose.transform = self.world_pose
        self.tf.sendTransform(pose)
        if self.feed_decision_tf:
            self.decision_tf.publish(TFMessage(transforms=[pose]))


def nav2_parameters(folder):
    share = Path(get_package_share_directory('nav2_bringup'))
    defaults = yaml.safe_load((share / 'params' / 'nav2_params.yaml').read_text())
    planner = defaults['planner_server']['ros__parameters']
    planner['GridBased'].update(allow_unknown=False, tolerance=0.0)
    for layer in ('global_costmap', 'local_costmap'):
        params = defaults[layer][layer]['ros__parameters']
        params.update(global_frame='map', robot_base_frame='base_footprint',
                      robot_radius=0.22, plugins=['static_layer', 'inflation_layer'])
        params['static_layer'].update(map_topic='/map', subscribe_to_updates=False)
    control = defaults['controller_server']['ros__parameters']
    control['odom_topic'] = DRIVE + '/odometry'
    control['progress_checker'].update(required_movement_radius=0.08, movement_time_allowance=15.0)
    control['general_goal_checker'].update(xy_goal_tolerance=0.16, yaw_goal_tolerance=0.25)
    control['FollowPath'] = {
        'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
        'desired_linear_vel': 0.23,
        'rotate_to_heading_angular_vel': 0.8,
    }
    navigator = defaults['bt_navigator']['ros__parameters']
    navigator.update(robot_base_frame='base_footprint', odom_topic=DRIVE + '/odometry')
    for name, planner_node, goal in (
        ('nav_to_pose', 'ComputePathToPose', 'goal'),
        ('nav_through_poses', 'ComputePathThroughPoses', 'goals'),
    ):
        tree_path = folder / (name + '.xml')
        tree_path.write_text(
            '<root main_tree_to_execute="Main"><BehaviorTree ID="Main"><Sequence>'
            f'<{planner_node} {goal}="{{{goal}}}" path="{{path}}" planner_id="GridBased"/>'
            '<FollowPath path="{path}" controller_id="FollowPath"/>'
            '</Sequence></BehaviorTree></root>', encoding='utf-8')
        navigator[f'default_{name}_bt_xml'] = str(tree_path)
    params_path = folder / 'nav2.yaml'
    params_path.write_text(yaml.safe_dump({
        '/**': {'ros__parameters': {'use_sim_time': True}},
        **{name: defaults[name] for name in
           ('bt_navigator', 'controller_server', 'local_costmap', 'planner_server', 'global_costmap')},
        ROUTE_NS.lstrip('/'): {name: deepcopy(defaults[name])
                              for name in ('planner_server', 'global_costmap')},
    }), encoding='utf-8')
    return params_path


def exercise(executor, harness, children):
    def wait(condition, description, timeout=18):
        wait_until(executor, condition, description, children, timeout=timeout)

    wait(harness.routes_verified, 'Gazebo pose and independently verified routes', 45)
    assert math.dist(harness.position, START) < 0.08, harness.position
    buffer = harness.sensor_transforms
    camera = 'front_camera_optical'
    lidar = 'front_mid360'
    wait(lambda: buffer.can_transform('map', camera, Time()) and buffer.can_transform('map', lidar, Time()),
         'camera and lidar connected to the Gazebo map pose')
    initial = buffer.lookup_transform('base_footprint', camera, Time()).transform.rotation
    initial_lidar = buffer.lookup_transform('base_footprint', lidar, Time()).transform
    harness.joint_angles = [0.25, -0.15]

    def camera_rotated():
        current = buffer.lookup_transform('base_footprint', camera, Time()).transform.rotation
        dot = initial.x * current.x + initial.y * current.y + initial.z * current.z + initial.w * current.w
        return 2.0 * math.acos(min(1.0, abs(dot))) > 0.20

    wait(camera_rotated, 'camera TF following the simulated yaw and pitch')
    assert buffer.lookup_transform('base_footprint', lidar, Time()).transform == initial_lidar
    print('PASS: camera and lidar share the Gazebo map TF. Joint changes move the camera TF only.')
    harness.phase = 3
    wait(lambda: any(msg.mode == 'WaitGame' for msg in harness.intents), 'waiting decision')
    harness.phase = 4
    wait(lambda: 'active:red_supply' in harness.status, 'Nav2 accepting the decision goal')
    origin = harness.position
    wait(lambda: math.dist(harness.position, origin) >= 0.12, 'physical Gazebo motion', 24)
    traveled = math.dist(harness.position, origin)

    harness.feed_decision_tf = False
    wait(lambda: any(msg.mode == 'LocalizationLost' for msg in harness.intents)
         and 'canceled:red_supply' in harness.status, 'Nav2 canceling after decision TF becomes stale')
    stop_pose = harness.position
    assert math.dist(stop_pose, TARGET) > 0.18, stop_pose
    deadline = time.monotonic() + 1.0

    def stationary_window():
        assert time.monotonic() - harness.last_pose < 0.35, 'Gazebo pose stream stopped during stop check'
        assert math.dist(harness.position, stop_pose) < 0.08, 'Gazebo kept moving after cancellation'
        return time.monotonic() >= deadline

    wait(stationary_window, 'stationary physical pose after cancellation', 2)
    stop_drift = math.dist(harness.position, stop_pose)

    harness.feed_decision_tf = True
    wait(lambda: 'succeeded:red_supply' in harness.status, 'Nav2 reaching the target after TF recovery', 32)
    goal_error = math.dist(harness.position, TARGET)
    assert goal_error < 0.19, (harness.position, goal_error)
    assert harness.verified_targets == {'red_supply', 'start_marker'}, harness.verified_targets
    print(f'PASS: Gazebo translated {traveled:.3f} m; after cancel, drift was {stop_drift:.3f} m; '
          f'after restoring TF, Nav2 succeeded with {goal_error:.3f} m world-pose error.')


def gazebo_gui_command():
    command = ['ign', 'gazebo', '-g', '--render-engine-gui']
    if os.environ.get('WSL_DISTRO_NAME') and os.environ.get('WAYLAND_DISPLAY'):
        # Restrict the Mesa workaround to the GUI process. The physics server keeps its environment.
        return ['env', 'MESA_EXTENSION_OVERRIDE=+GL_ARB_copy_image', *command, 'ogre2']
    return [*command, 'ogre']


def read_scenario(path):
    scenario = yaml.safe_load(path.read_text(encoding='utf-8'))
    required = {'phase', 'hp', 'max_hp', 'localization'}
    if not isinstance(scenario, dict) or not required <= scenario.keys() or set(scenario) - required - {'patrol'}:
        raise ValueError('Expected phase, hp, max_hp, localization and optional patrol in the scenario file.')
    phase = scenario['phase']
    hp, max_hp = scenario['hp'], scenario['max_hp']
    if not isinstance(phase, str) or phase not in SCENARIO_PHASES:
        raise ValueError('phase must be waiting or running.')
    if type(hp) is not int or type(max_hp) is not int or not 0 <= hp <= max_hp <= 65535 or not max_hp:
        raise ValueError('Expected integer HP with 0 <= hp <= max_hp and 1 <= max_hp <= 65535.')
    if type(scenario['localization']) is not bool:
        raise ValueError('localization must be true or false.')
    patrol = scenario.get('patrol')
    if patrol is not None:
        if (not isinstance(patrol, list) or len(patrol) != 2
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in patrol)):
            raise ValueError('patrol must be [x, y] with two finite numbers.')
        scenario['patrol'] = [float(patrol[0]), float(patrol[1])]
    return scenario


def follow_scenario(executor, harness, children, gui, path):
    harness.phase = SCENARIO_PHASES['waiting']
    wait_until(executor, lambda: any(msg.mode == 'WaitGame' for msg in harness.intents),
               'decision receiving the initial waiting state', children, timeout=30)
    wait_until(executor, lambda: {'red_supply', SCENARIO_PATROL[0]} <= harness.verified_targets,
               'verification of both manual scenario routes', children, timeout=45)
    print(f'Manual scenario ready. Edit and save {path}. Close Gazebo or press Ctrl+C to stop.')
    previous = error = intent = status = None
    previous_patrol = None
    next_read = 0.0
    while rclpy.ok() and gui.poll() is None:
        executor.spin_once(timeout_sec=0.1)
        if any(child.poll() is not None for child in children if child is not gui):
            raise RuntimeError('A scenario process exited.')
        now = time.monotonic()
        if now >= next_read:
            next_read = now + 0.25
            try:
                scenario = read_scenario(path)
            except (OSError, ValueError, yaml.YAMLError) as exc:
                if str(exc) != error:
                    error = str(exc)
                    print(f'Scenario input ignored: {error}')
            else:
                live = {key: value for key, value in scenario.items() if key != 'patrol'}
                patrol = scenario.get('patrol', list(SCENARIO_PATROL[1:]))
                if live != previous:
                    harness.hp, harness.max_hp = live['hp'], live['max_hp']
                    harness.phase = SCENARIO_PHASES[live['phase']]
                    harness.feed_decision_tf = live['localization']
                    previous = live
                    print(f'Scenario: {live}')
                if previous_patrol is not None and patrol != previous_patrol:
                    print('Patrol point changed; restart the scenario to apply it.')
                previous_patrol = patrol
                error = None
        if harness.intents:
            msg = harness.intents[-1]
            result = (msg.mode, msg.target or '-', msg.reason)
            if result != intent:
                intent = result
                print(f'Decision: {intent[0]}  target={intent[1]}  reason={intent[2]}')
        if harness.status and harness.status[-1] != status:
            status = harness.status[-1]
            print(f'Navigation: {status}')


def run(executor, harness, folder, cleanup, show_gui, scenario):
    children = []

    def start(label, command):
        child = cleanup.enter_context(process(command, folder / (label + '.log')))
        children.append(child)
        return child

    def node(package, program, *args, label=None):
        start(label or program, [executable(package, program), '--ros-args', *args])

    map_path = SOURCE / 'maps' / 'offline' / 'red_map.yaml'
    world = folder / 'offline_3v3.sdf'
    write_world(map_path, world, START)
    tasks = folder / 'tasks.yaml'
    if scenario is None:
        write_tasks(tasks)
    else:
        patrol = read_scenario(scenario).get('patrol', SCENARIO_PATROL[1:])
        write_tasks(tasks, patrol=(SCENARIO_PATROL[0], *patrol))
    robot = SOURCE / 'robot_description' / 'sentry_preview.urdf.xacro'
    model_params = folder / 'preview_model.yaml'
    model_params.write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {
        'robot_description': xacro.process_file(str(robot)).toxml(),
    }}}), encoding='utf-8')
    params = ('--params-file', str(nav2_parameters(folder)))
    sim = ('-p', 'use_sim_time:=true')
    ns = ('-r', '__ns:=' + DECISION)
    start('gazebo', ['ign', 'gazebo', '-s', '-r', str(world)])
    gui = start('gazebo_gui', gazebo_gui_command()) if show_gui else None
    start('gz_bridge', [executable('ros_gz_bridge', 'parameter_bridge'),
                        DRIVE + '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                        DRIVE + '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                        WORLD_TOPICS + '/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                        WORLD_TOPICS + '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                        '--ros-args', '-r', WORLD_TOPICS + '/clock:=/clock'])
    node('robot_state_publisher', 'robot_state_publisher', *sim, '--params-file', str(model_params))
    node('nav2_map_server', 'map_server', *sim, '-p', f'yaml_filename:={map_path}')
    node('nav2_planner', 'planner_server', *params)
    node('nav2_planner', 'planner_server', *params, '-r', '__ns:=' + ROUTE_NS, label='route_planner')
    node('nav2_controller', 'controller_server', *params, '-r', 'cmd_vel:=' + DRIVE + '/cmd_vel')
    node('nav2_bt_navigator', 'bt_navigator', *params)
    node('nav2_lifecycle_manager', 'lifecycle_manager', *sim, '-p', 'autostart:=true',
         '-p', 'node_names:=[map_server,planner_server,controller_server,bt_navigator]')
    node('nav2_lifecycle_manager', 'lifecycle_manager', *sim,
         '-r', '__ns:=' + ROUTE_NS, '-p', 'autostart:=true',
         '-p', 'node_names:=[planner_server]', label='route_lifecycle')
    node('rm2027_decision', 'navigation_evidence.py', *ns, *sim,
         '-p', f'tasks_file:={tasks}', '-p', 'protected_map_confirmed:=true',
         '-p', f'planner_action:={ROUTE_NS}/compute_path_to_pose', '-r', '/tf:=' + DECISION_TF)
    node('rm2027_decision', 'referee_state_adapter.py', *ns, *sim, '-p', 'vision_status:=false')
    node('rm2027_decision', 'rm2027_tree', *ns,
         '--params-file', str(SOURCE / 'config' / 'decision.yaml'),
         *sim, '-p', 'enabled:=true',
         '-p', f'tree_file:={SOURCE / "behavior_trees" / "intents.xml"}')
    node('rm2027_decision', 'intent_nav_bridge.py', *ns, *sim,
         '-p', 'execute_navigation:=true', '-p', f'tasks_file:={tasks}',
         '-p', 'navigation_action:=/navigate_to_pose')
    try:
        if scenario is not None:
            follow_scenario(executor, harness, children, gui, scenario)
            return
        exercise(executor, harness, children)
        if gui is not None:
            print('Gazebo is ready for inspection. Close the window or press Ctrl+C to stop.')
            while rclpy.ok() and gui.poll() is None:
                executor.spin_once(timeout_sec=0.2)
    except (ExternalShutdownException, KeyboardInterrupt):
        if gui is None:
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='show Gazebo and keep the window open after the test')
    parser.add_argument('--scenario', type=Path, help='open Gazebo with a live-editable scenario YAML file')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or not os.environ.get('ROS_DOMAIN_ID'):
        raise SystemExit('Set ROS_LOCALHOST_ONLY=1 and an unused ROS_DOMAIN_ID.')
    if not shutil.which('ign'):
        raise SystemExit('Gazebo Fortress (ign gazebo) is required.')
    if args.scenario is not None:
        try:
            read_scenario(args.scenario)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            parser.error(str(exc))
    os.environ['IGN_PARTITION'] = 'rm_offline_' + uuid.uuid4().hex
    rclpy.init()
    harness = GazeboHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    try:
        with tempfile.TemporaryDirectory(prefix='rm-gazebo-check-') as scratch, ExitStack() as cleanup:
            run(executor, harness, Path(scratch), cleanup, args.gui or args.scenario is not None, args.scenario)
    finally:
        executor.shutdown()
        harness.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
