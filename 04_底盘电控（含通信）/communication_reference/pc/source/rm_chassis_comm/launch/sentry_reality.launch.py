"""Physical sentry USB gateway + joint TF + timestamped command conversion."""
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    with open(value('frame_config'), encoding='utf-8') as source:
        config = yaml.safe_load(source)
    serial = dict(config['serial'])
    frames = dict(config['frames'])
    serial.update(connect=value('connect').lower() == 'true', serial_port=value('serial_port'))
    # This launch has one canonical body-command source and enforces all gates.
    serial.update(route=2, use_stamped_commands=True, require_gimbal_state=True, require_frame_ready=True)
    namespace = value('namespace')
    share = get_package_share_directory('rm_chassis_comm')
    nodes = [
        Node(package='rm_chassis_comm', executable='chassis_node', name='chassis_comm',
             namespace=namespace, output='screen',
             parameters=[os.path.join(share, 'config', 'chassis.yaml'), serial],
             remappings=[('cmd_vel_stamped', 'cmd_vel_body')]),
        Node(package='rm_chassis_comm', executable='sentry_frame_bridge', name='sentry_frame_bridge',
             namespace=namespace, output='screen', parameters=[frames],
             remappings=[('cmd_vel_gimbal', value('cmd_vel_topic')),
                         ('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ]
    seen = {frames['yaw_frame'], frames['pitch_frame']}
    for item in config.get('static_transforms', []):
        if not config.get('static_extrinsics_confirmed', frames.get('geometry_confirmed', False)):
            raise RuntimeError('Static extrinsics require measured static_extrinsics_confirmed=true')
        xyz, rpy = item['xyz'], item['rpy']
        if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(x) for x in xyz + rpy):
            raise RuntimeError('Each static transform requires finite xyz/rpy triples')
        if not item['parent'] or not item['child'] or item['parent'] == item['child'] or item['child'] in seen:
            raise RuntimeError('Duplicate or invalid static TF child')
        if item['child'] in ('map', 'odom', frames['body_frame']):
            raise RuntimeError('Map/odom/body transforms belong to localization, not this static list')
        seen.add(item['child'])
        nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                          name='sentry_static_' + str(len(seen)), namespace=namespace,
                          remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                          arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                                     '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                                     '--frame-id', item['parent'], '--child-frame-id', item['child']]))
    if value('start_navigation').lower() == 'true':
        autostart = value('autostart').lower() == 'true'
        if autostart and not frames.get('geometry_confirmed', False):
            autostart = False
            nodes.append(LogInfo(msg='Gimbal geometry unconfirmed: Nav2 lifecycle autostart deferred.'))
        if not value('map') or not value('prior_pcd_file'):
            raise RuntimeError('Physical navigation needs explicit map and prior_pcd_file paths')
        for name in ('map', 'prior_pcd_file', 'nav_params'):
            if value(name) and not os.path.isfile(value(name)):
                raise RuntimeError('Missing navigation input: ' + name + '=' + value(name))
        bringup = get_package_share_directory('pb2025_nav_bringup')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'rm_navigation_reality_launch.py')), launch_arguments={
                'namespace': namespace, 'map': value('map'), 'prior_pcd_file': value('prior_pcd_file'),
                'params_file': value('nav_params') or os.path.join(bringup, 'config', 'reality', 'nav2_params.yaml'),
                'use_sim_time': 'false', 'use_robot_state_pub': 'false',
                'use_rviz': value('use_rviz'),
                'start_lidar': value('start_lidar'),
                'start_joystick': value('start_joystick'),
                'use_composition': value('use_composition'),
                'autostart': str(autostart).lower(),
            }.items()))
    return nodes


def generate_launch_description():
    share = get_package_share_directory('rm_chassis_comm')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='red_standard_robot1'),
        DeclareLaunchArgument('connect', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('frame_config', default_value=os.path.join(share, 'config', 'sentry_real.yaml')),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel_gimbal_stamped'),
        DeclareLaunchArgument('start_navigation', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_joystick', default_value='false'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('prior_pcd_file', default_value=''),
        DeclareLaunchArgument('nav_params', default_value=''),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        OpaqueFunction(function=setup),
    ])
