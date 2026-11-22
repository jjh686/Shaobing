"""Standalone onboard decision adapter for the RM_11_13 navigation interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nodes(context):
    share = get_package_share_directory('rm2027_decision')
    arg = LaunchConfiguration
    ns = arg('namespace')
    execute = arg('execute_navigation').perform(context).lower()
    enabled = arg('enabled').perform(context).lower()
    port = arg('referee_uart').perform(context)
    backend = arg('navigation_backend').perform(context)
    live = backend == 'live'
    action = arg('navigation_action').perform(context) or (
        '/navigate_to_pose' if live else '/red_standard_robot1/navigate_to_pose')
    if live and (ns.perform(context).strip('/') != 'decision2027' or action != '/navigate_to_pose'):
        raise RuntimeError('Live navigation requires namespace decision2027 and action /navigate_to_pose')
    if execute == 'true' and (enabled != 'true' or not port or os.path.realpath(port).startswith('/dev/pts/')
                              or arg('robot_id').perform(context) == '0'
                              or arg('use_sim_time').perform(context) == 'true'):
        raise RuntimeError('Execution requires enabled=true, a confirmed ID and a physical referee UART')
    clock = {'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool)}
    policy = os.path.join(share, 'config', 'decision.yaml')
    return [
        Node(package='rm2027_decision', executable='referee_state_adapter.py', namespace=ns,
             output='screen', parameters=[policy, clock, {
                 'referee_uart': port,
                 'robot_id': ParameterValue(arg('robot_id'), value_type=int),
                 'vision_status': ParameterValue(arg('vision_status'), value_type=bool),
             }]),
        Node(package='rm2027_decision', executable='rm2027_tree', namespace=ns,
             output='screen', parameters=[policy, clock, {
                 'enabled': ParameterValue(arg('enabled'), value_type=bool),
                 'tree_file': os.path.join(share, 'trees', 'intents.xml'),
             }]),
        Node(package='rm2027_decision', executable='intent_nav_bridge.py', namespace=ns,
             output='screen', parameters=[policy, clock, {
                 'execute_navigation': ParameterValue(arg('execute_navigation'), value_type=bool),
                 'tasks_file': arg('tasks_file'),
                 'navigation_action': action,
                 'navigation_backend': backend,
             }]),
    ]


def generate_launch_description():
    share = get_package_share_directory('rm2027_decision')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='decision2027'),
        DeclareLaunchArgument('enabled', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('execute_navigation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('referee_uart', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='0', choices=['0', '7', '107']),
        DeclareLaunchArgument('vision_status', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('tasks_file', default_value=os.path.join(share, 'config', 'approved_tasks.yaml')),
        DeclareLaunchArgument('navigation_backend', default_value='fixed_map', choices=['fixed_map', 'live']),
        DeclareLaunchArgument('navigation_action', default_value=''),
        OpaqueFunction(function=nodes),
    ])
