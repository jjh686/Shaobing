"""Standalone intent inspection. Does not launch navigation, serial or simulation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rm2027_decision')
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='rmul_3v3'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='decision2027'),
        Node(package='rm2027_decision', executable='rm2027_tree',
             namespace=arg('namespace'), output='screen',
             parameters=[os.path.join(share, 'config', 'decision.yaml'), {
                 'profile': arg('profile'),
                 'enabled': ParameterValue(arg('enabled'), value_type=bool),
                 'use_sim_time': ParameterValue(arg('use_sim_time'), value_type=bool),
                 'tree_file': os.path.join(share, 'behavior_trees', 'intents.xml'),
             }]),
    ])
