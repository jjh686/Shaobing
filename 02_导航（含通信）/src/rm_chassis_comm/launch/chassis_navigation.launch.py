"""Fixed chassis lidar; the gimbal C board is only a USB/CAN gateway."""
import math
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    with open(value('mounting_config'), encoding='utf-8') as source:
        mounting = yaml.safe_load(source)
    if mounting.get('configured') is not True or \
            mounting.get('parent_frame') != 'base_footprint' or \
            mounting.get('child_frame') != 'front_mid360':
        raise RuntimeError('Chassis navigation requires confirmed base_footprint -> front_mid360 mounting')
    xyz, rpy = mounting['translation_m'], mounting['rotation_rpy_rad']
    if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(x) for x in xyz + rpy):
        raise RuntimeError('Lidar mounting needs finite xyz/rpy triples')
    share = get_package_share_directory('rm_chassis_comm')
    namespace = value('namespace')
    nodes = [
        Node(package='rm_chassis_comm', executable='chassis_node', name='chassis_comm',
             namespace=namespace, output='screen', parameters=[
                 os.path.join(share, 'config', 'chassis.yaml'), {
                     'connect': value('connect').lower() == 'true',
                     'serial_port': value('serial_port'), 'route': 2,
                     'frame_id': 'base_footprint', 'enabled': False,
                     'use_stamped_commands': False, 'require_gimbal_state': False,
                     'require_frame_ready': False, 'sp_proxy_enabled': False,
                 }], remappings=[('cmd_vel', 'cmd_vel_nav2_result'),
                                 ('cmd_vel_stamped', 'unused_cmd_vel_stamped')]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='chassis_lidar_mount', namespace=namespace, output='screen',
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')], arguments=[
                 '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                 '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                 '--frame-id', 'base_footprint', '--child-frame-id', 'front_mid360']),
    ]
    if value('start_navigation').lower() == 'true':
        for key in ('nav_params', 'map', 'prior_pcd_file'):
            if not os.path.isfile(value(key)):
                raise RuntimeError('Missing navigation input: ' + key + '=' + value(key))
        with open(value('nav_params'), encoding='utf-8') as source:
            params = yaml.safe_load(source)
        for section in ('sensor_scan_generation', 'small_gicp_relocalization', 'bt_navigator', 'behavior_server'):
            if params[section]['ros__parameters']['robot_base_frame'] != 'base_footprint':
                raise RuntimeError('Use chassis_nav2_params.yaml: wrong base frame in ' + section)
        for section in ('local_costmap', 'global_costmap'):
            if params[section][section]['ros__parameters']['robot_base_frame'] != 'base_footprint':
                raise RuntimeError('Use chassis_nav2_params.yaml: wrong base frame in ' + section)
        bringup = get_package_share_directory('pb2025_nav_bringup')
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'rm_navigation_reality_launch.py')), launch_arguments={
                'namespace': namespace, 'map': value('map'), 'prior_pcd_file': value('prior_pcd_file'),
                'params_file': value('nav_params'), 'use_sim_time': 'false',
                'use_robot_state_pub': 'false', 'start_fake_vel_transform': 'false',
                'start_joystick': 'false', 'start_lidar': value('start_lidar'),
                'use_composition': value('use_composition'), 'autostart': value('autostart'),
                'use_rviz': value('use_rviz'), 'rviz_config_file': value('rviz_config_file'),
            }.items()))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='red_standard_robot1'),
        DeclareLaunchArgument('connect', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('mounting_config'),
        DeclareLaunchArgument('nav_params'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(
            get_package_share_directory('pb2025_nav_bringup'), 'rviz', 'nav2_default_view.rviz')),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('prior_pcd_file', default_value=''),
        OpaqueFunction(function=setup),
    ])
