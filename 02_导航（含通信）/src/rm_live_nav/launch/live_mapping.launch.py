"""New live SLAM map -> Nav2 -> bounded frontier exploration, fixed gimbal only."""
import ipaddress
import json
import math
import subprocess
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rm_live_nav.geometry import validate_mounting


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    nav = value('navigation').lower() == 'true'
    root, runtime = Path(value('source_root')), Path(value('runtime'))
    footprint = None
    if nav:
        with open(value('mounting_config')) as stream:
            xyz, rpy, footprint = validate_mounting(yaml.safe_load(stream))
    with open(root/'local/chassis_nav2_params.yaml') as stream:
        params = yaml.safe_load(stream)
    lio = params['point_lio']['ros__parameters']
    lio['publish'].update(tf_send_en=False, path_en=True)
    lio['pcd_save']['pcd_save_en'] = False
    # IMU initialization aligns the observed tilted gravity with the world -Z.
    lio['mapping'].update(gravity=[0., 0., -9.81], gravity_init=[0., 0., -9.81])
    lio['common'].update(lid_topic='livox/lidar', imu_topic='livox/imu')
    slam = params['slam_toolbox']['ros__parameters']
    base = 'base_footprint' if nav else 'mapping_base'
    slam.update(odom_frame='camera_init', base_frame=base, scan_topic='/live/scan',
                transform_publish_period=0.0, use_scan_matching=False, do_loop_closing=False,
                min_laser_range=0.1, minimum_travel_distance=0.0,
                minimum_travel_heading=0.0, minimum_time_interval=0.2)
    nodes = []
    if value('start_lidar').lower() == 'true':
        host, lidar = str(ipaddress.IPv4Address(value('host_ip'))), str(ipaddress.IPv4Address(value('lidar_ip')))
        interfaces = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show'], text=True))
        assigned = [address['local'] for interface in interfaces for address in interface.get('addr_info', [])]
        if host not in assigned:
            raise RuntimeError(f'Radar host IP {host} is not assigned to a local interface. Check ip -br -4 addr and the radar Ethernet adapter; no network settings were changed.')
        with open(root/'外部依赖源码/livox_ros_driver2/config/MID360_config.json') as stream:
            driver = json.load(stream)
        for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            driver['Mid360s']['host_net_info'][key] = host
        driver['lidar_configs'][0]['ip'] = lidar
        runtime.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile('w', prefix='live-mid360-', suffix='.json', dir=runtime, delete=False) as config:
            json.dump(driver, config)
        nodes.append(Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
            name='livox_ros_driver2', output='screen', parameters=[{
                'xfer_format': 4, 'multi_topic': 0, 'data_src': 0,
                'publish_freq': 20., 'frame_id': 'front_mid360', 'user_config_path': config.name}]))
    nodes += [
        Node(package='point_lio', executable='pointlio_mapping', name='point_lio', output='screen', parameters=[lio]),
        Node(package='rm_live_nav', executable='lio_frames', output='screen', parameters=[{
            'mapping_only': not nav, 'mounting_config': value('mounting_config'),
            'imu_lidar_t': lio['mapping']['extrinsic_T'], 'imu_lidar_r': lio['mapping']['extrinsic_R']}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='live_map_origin',
             arguments=['--frame-id', 'map', '--child-frame-id', 'camera_init']),
        Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
             name='live_scan', output='screen', parameters=[{
                 'target_frame': base, 'transform_tolerance': 0.1,
                 'min_height': 0.08 if nav else -0.30, 'max_height': 1.5,
                 'min_intensity': 0., 'max_intensity': 255., 'angle_min': -math.pi,
                 'angle_max': math.pi, 'angle_increment': math.pi/360.,
                 'scan_time': .05, 'range_min': .1, 'range_max': 10., 'use_inf': True}],
             remappings=[('cloud_in', 'live/cloud'), ('scan', 'live/scan')]),
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[slam]),
        Node(executable='/usr/bin/python3', name='live_map_recorder', output='screen',
             arguments=[str(root/'scripts/mid360_map_recorder.py')],
             parameters=[{'output_dir': value('output_dir')}],
             remappings=[('cloud_registered', 'live/cloud')]),
    ]
    if nav:
        share = Path(get_package_share_directory('rm_live_nav'))
        with open(share/'config/nav2.yaml') as stream:
            nav_params = yaml.safe_load(stream)
        for name in ('local_costmap', 'global_costmap'):
            nav_params[name][name]['ros__parameters']['footprint'] = json.dumps(footprint.tolist())
        nav_params['bt_navigator']['ros__parameters'].update(
            default_nav_to_pose_bt_xml=str(share/'config/navigate.xml'),
            default_nav_through_poses_bt_xml=str(share/'config/navigate.xml'))
        # File parameters keep the nested costmap node selectors intact.
        with tempfile.NamedTemporaryFile('w', prefix='live-nav2-', suffix='.yaml', dir=runtime, delete=False) as output:
            yaml.safe_dump(nav_params, output)
        lifecycle = []
        for package, executable in [('nav2_controller', 'controller_server'),
                                    ('nav2_planner', 'planner_server'), ('nav2_bt_navigator', 'bt_navigator')]:
            lifecycle.append(executable)
            nodes.append(Node(package=package, executable=executable, name=executable,
                output='screen', parameters=[output.name], remappings=[('cmd_vel', 'live/cmd_vel_raw')]))
        radius = float(max(math.hypot(*point) for point in footprint))+.15
        nodes += [
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='live_lifecycle', parameters=[{'autostart': True, 'node_names': lifecycle}]),
            Node(package='rm_chassis_comm', executable='chassis_node', name='chassis_comm', output='screen',
                 parameters=[{'connect': value('connect').lower() == 'true', 'enabled': False,
                              'route': 2, 'serial_port': value('serial_port'), 'send_hz': 50,
                              'command_timeout_ms': 150, 'feedback_timeout_ms': 250,
                              'max_linear_speed': .1, 'max_angular_speed': .25,
                              'frame_id': 'base_footprint', 'use_stamped_commands': True,
                              'require_frame_ready': True, 'require_gimbal_state': False}],
                 remappings=[('cmd_vel_stamped', 'live/cmd_vel'), ('cmd_vel', 'unused_cmd_vel')]),
            Node(package='rm_live_nav', executable='motion_gate', output='screen', parameters=[{'stop_radius': radius}]),
            Node(package='rm_live_nav', executable='frontier_explorer', output='screen', parameters=[{'clearance': radius}]),
            Node(package='tf2_ros', executable='static_transform_publisher', name='live_lidar_mount',
                 arguments=['--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                            '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                            '--frame-id', 'base_footprint', '--child-frame-id', 'front_mid360']),
        ]
    if value('rviz').lower() == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', arguments=['-d', str(
            Path(get_package_share_directory('rm_live_nav'))/'config/live.rviz')]))
    return nodes


def generate_launch_description():
    defaults = {'navigation': 'false', 'connect': 'false', 'start_lidar': 'true',
                'serial_port': '/dev/unused', 'host_ip': '192.168.1.50', 'lidar_ip': '192.168.1.106',
                'rviz': 'false'}
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]+[
        DeclareLaunchArgument(key) for key in ('source_root', 'runtime', 'mounting_config', 'output_dir')]+[
        OpaqueFunction(function=setup)])
