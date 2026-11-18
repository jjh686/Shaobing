"""Direct USB chassis endpoint. Defaults to preview without opening hardware."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rm_chassis_comm')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('connect', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('namespace', default_value='chassis'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/chassis/manual_body_cmd_vel'),
        Node(package='rm_chassis_comm', executable='chassis_node',
             namespace=arg('namespace'), output='screen',
             remappings=[('cmd_vel', arg('cmd_vel_topic'))],
             parameters=[os.path.join(share, 'config', 'chassis.yaml'), {
                 'route': 1,
                 'connect': ParameterValue(arg('connect'), value_type=bool),
                 'serial_port': arg('serial_port'),
             }]),
    ])
