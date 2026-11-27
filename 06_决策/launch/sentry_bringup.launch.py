"""Compose onboard decision, live navigation, and the current auto-aim entry."""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))
from bringup_defaults import (fill_live_paths, mounting_refused_message,
                              resolve_vision_project, vision_command)


def include_launch(package, filename, arguments):
    source = Path(get_package_share_directory(package)) / 'launch' / filename
    return GroupAction(actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)), launch_arguments=arguments.items())])


def record_diagnostics(path, topics, simulated):
    output = Path(path).expanduser().resolve()
    if output.exists():
        raise RuntimeError(f'Recording destination already exists: {output}')
    # Small default topic set and bounded segment size for field triage.
    command = ['ros2', 'bag', 'record', '-o', str(output), '--include-hidden-topics',
               '--max-bag-size', str(256 * 1024 * 1024)]
    if simulated:
        command += ['--use-sim-time']
    return ExecuteProcess(cmd=command + list(dict.fromkeys(topics)), output='screen')


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('rm2027_decision'))
    live = value('mode') == 'live'
    execute = value('execute_navigation') == 'true'
    connect = value('connect_chassis') == 'true'
    simulated = value('use_sim_time') == 'true'
    description = value('robot_urdf')
    offline_model = value('robot_model') == 'offline'
    port = value('serial_port')
    referee = value('referee_uart')
    tasks = value('tasks_file') or str(share / 'config' / 'approved_tasks.yaml')
    namespace = value('nav_namespace').strip('/')
    prefix = '' if live or not namespace else '/' + namespace
    keepout_map = value('keepout_map')
    map_topic = '/live/keepout_mask' if live and keepout_map else (
        value('route_map_topic') or prefix + '/map')
    tf_topic = value('tf_topic') or prefix + '/tf'
    tf_static_topic = value('tf_static_topic') or prefix + '/tf_static'
    protected_map = value('protected_map_confirmed') == 'true'
    clearance = float(value('clearance_radius_m'))
    planner = value('planner_action') or prefix + '/decision_route/compute_path_to_pose'
    start_vision = value('start_vision') == 'true'
    vision_usb = value('vision_usb') == 'true'

    if not live and (execute or connect or referee or keepout_map):
        raise RuntimeError('External preview accepts navigation and vision observations only')
    if live and (simulated or description or offline_model):
        raise RuntimeError('Live navigation publishes its own measured transforms and uses system time')
    if description and offline_model:
        raise RuntimeError('Select either robot_urdf or robot_model:=offline')
    if live and value('planner_action'):
        raise RuntimeError('Live evidence uses its dedicated planner')
    if execute and not (connect and protected_map and keepout_map):
        raise RuntimeError('Execution requires chassis connection and a reviewed keepout_map')
    if connect and (not port or not Path(port).is_char_device()):
        raise RuntimeError('Set serial_port to the chassis gateway serial device')
    if connect and referee and Path(referee).resolve() == Path(port).resolve():
        raise RuntimeError('The referee UART and chassis gateway must be separate serial devices')
    if start_vision and simulated:
        raise RuntimeError('start_vision launches the live auto_aim entry and cannot use use_sim_time')
    if start_vision and vision_usb and value('vision_status') != 'true':
        raise RuntimeError(
            'start_vision with vision_usb requires vision_status:=true so decision can receive '
            'tracker status while auto_aim subscribes to /decision2027/vision_mode_request')

    nodes = []
    ros_time = ParameterValue(simulated, value_type=bool)
    if live:
        import math
        import yaml
        from rm_live_nav.geometry import validate_mounting

        paths = fill_live_paths(
            {name: value(name) for name in ('source_root', 'runtime', 'mounting_config', 'output_dir')})
        if not all(paths.values()):
            raise RuntimeError(
                'Live mode requires source_root, runtime, mounting_config and output_dir. '
                'source 02_导航（含通信）/scripts/local_env.sh, or run 06_决策/scripts/run_sentry_bringup.sh')
        try:
            _, _, footprint = validate_mounting(
                yaml.safe_load(Path(paths['mounting_config']).read_text(encoding='utf-8')))
        except (ValueError, OSError, KeyError, TypeError) as exc:
            raise RuntimeError(mounting_refused_message(exc)) from exc
        clearance = max(math.hypot(*point) for point in footprint) + 0.05
        nodes.append(include_launch('rm_live_nav', 'live_mapping.launch.py', {
            **paths,
            'navigation': 'true',
            'goal_owner': 'decision',
            'connect': str(connect).lower(),
            'serial_port': port or '/dev/unused',
            'start_lidar': value('start_lidar'),
            'host_ip': value('host_ip'),
            'lidar_ip': value('lidar_ip'),
            'rviz': value('rviz'),
            'keepout_map': keepout_map,
        }))
    elif description or offline_model:
        if offline_model:
            import xacro

            model = share / 'robot_description'
            model_xml = xacro.process_file(str(model / 'sentry_preview.urdf.xacro')).toxml()
            nodes.append(Node(package='joint_state_publisher', executable='joint_state_publisher',
                              namespace=namespace, parameters=[{'use_sim_time': ros_time}], output='screen'))
        else:
            model_xml = Path(description).read_text(encoding='utf-8')
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
            parameters=[{'robot_description': ParameterValue(model_xml, value_type=str), 'use_sim_time': ros_time}],
            remappings=[('/tf', tf_topic), ('/tf_static', tf_static_topic)], output='screen'))

    nodes.append(Node(
        package='rm2027_decision', executable='navigation_evidence.py', namespace='decision2027',
        parameters=[{
            'use_sim_time': ros_time,
            'tasks_file': tasks,
            'base_frame': value('base_frame'),
            'planner_action': planner,
            'route_map_topic': map_topic,
            'protected_map_confirmed': protected_map,
            'clearance_radius_m': clearance,
            'nominal_speed_mps': 0.1 if live else 0.15,
        }], remappings=[('/tf', tf_topic), ('/tf_static', tf_static_topic)], output='screen'))
    nodes.append(include_launch('rm2027_decision', 'rm_11_13.launch.py', {
        'enabled': value('enabled'),
        'execute_navigation': value('execute_navigation'),
        'use_sim_time': value('use_sim_time'),
        'robot_id': value('robot_id'),
        'referee_uart': referee,
        'vision_status': value('vision_status'),
        'tasks_file': tasks,
        'tree_file': value('tree_file') or str(share / 'behavior_trees' / 'intents.xml'),
        'navigation_backend': 'live' if live else 'fixed_map',
        'navigation_action': prefix + '/navigate_to_pose',
    }))
    if start_vision:
        try:
            command = vision_command(
                resolve_vision_project(value('vision_project')), usb=vision_usb)
        except FileNotFoundError as exc:
            raise RuntimeError(str(exc)) from exc
        nodes.append(ExecuteProcess(cmd=command, output='screen', name='live_usb_sp_vision'))
    if bag := value('record_bag'):
        topics = [tf_topic, tf_static_topic, prefix + '/map', map_topic,
                  value('odometry_topic') or ('/live/odometry' if live else prefix + '/odometry'),
                  prefix + '/navigate_to_pose/_action/status']
        topics += ['/decision2027/' + name for name in (
            'state', 'navigation_evidence', 'intent', 'navigation_status', 'vision_mode_request')]
        if live:
            topics += ['/live/scan', '/diagnostics', '/measured_velocity']
        if value('record_lidar') == 'true':
            topics += [prefix + '/livox/lidar', prefix + '/livox/imu']
        nodes.append(record_diagnostics(bag, topics, simulated))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='external', choices=['external', 'live']),
        DeclareLaunchArgument('enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('execute_navigation', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('connect_chassis', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('start_lidar', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_vision', default_value='false', choices=['true', 'false'],
                              description='Launch the current auto_aim_test entry (scripts/run_live_usb_sp.sh)'),
        DeclareLaunchArgument('vision_usb', default_value='true', choices=['true', 'false'],
                              description='true: official USB SP entry with decision-status; false: --preview only'),
        DeclareLaunchArgument('vision_project', default_value='',
                              description='Path to 01_自瞄（含通信）. Empty uses RM_11_13_ROOT or sibling of this repo'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('protected_map_confirmed', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('vision_status', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('nav_namespace', default_value='red_standard_robot1'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('route_map_topic', default_value=''),
        DeclareLaunchArgument('keepout_map', default_value=''),
        DeclareLaunchArgument('planner_action', default_value=''),
        DeclareLaunchArgument('clearance_radius_m', default_value='0.0'),
        DeclareLaunchArgument('tf_topic', default_value=''),
        DeclareLaunchArgument('tf_static_topic', default_value=''),
        DeclareLaunchArgument('robot_urdf', default_value=''),
        DeclareLaunchArgument('robot_model', default_value='none', choices=['none', 'offline']),
        DeclareLaunchArgument('record_bag', default_value='', description='New directory for diagnostic ROS bag'),
        DeclareLaunchArgument('record_lidar', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('odometry_topic', default_value=''),
        DeclareLaunchArgument('tasks_file', default_value=''),
        DeclareLaunchArgument('tree_file', default_value=''),
        DeclareLaunchArgument('robot_id', default_value='0', choices=['0', '7', '107']),
        DeclareLaunchArgument('referee_uart', default_value=''),
        DeclareLaunchArgument('serial_port', default_value='',
                              description='Chassis gateway serial device; required only when connect_chassis:=true'),
        DeclareLaunchArgument('source_root', default_value=''),
        DeclareLaunchArgument('runtime', default_value=''),
        DeclareLaunchArgument('mounting_config', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.50'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.106'),
        OpaqueFunction(function=setup),
    ])
