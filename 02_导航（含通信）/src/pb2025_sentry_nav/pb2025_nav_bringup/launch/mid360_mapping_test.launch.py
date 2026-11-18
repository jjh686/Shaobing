"""Standalone, manually carried MID-360S mapping; no chassis nodes required."""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def mapping_setup(context):
    # Keep ASCII aliases: this Humble installation cannot parse Unicode YAML paths.
    bringup = get_package_share_directory("pb2025_nav_bringup")
    params_path = LaunchConfiguration("params_file").perform(context)
    recorder_path = LaunchConfiguration("recorder_script").perform(context)
    with open(params_path) as stream:
        existing = yaml.safe_load(stream)
    lio = existing["point_lio"]["ros__parameters"]
    lio["mapping"].update(
        gravity=[0.0, 0.0, -9.81],
        gravity_init=[0.0, 0.0, -9.81],
        lidar_time_inte=0.05,
    )
    lio["odometry"]["publish_odometry_without_downsample"] = False
    lio["publish"].update(path_en=True, tf_send_en=True)
    # Save a voxel map with the recorder instead of retaining every raw scan.
    lio["pcd_save"]["pcd_save_en"] = False

    slam = existing["slam_toolbox"]["ros__parameters"]
    slam.update(
        odom_frame="camera_init",
        base_frame="aft_mapped",
        scan_topic="mid360_test/scan",
        transform_publish_period=0.0,
        use_scan_matching=False,
        do_loop_closing=False,
        min_laser_range=0.5,
        # Livox scans are sparse: also accumulate scans during the static test.
        minimum_travel_distance=0.0,
        minimum_travel_heading=0.0,
        minimum_time_interval=0.5,
    )

    return [
        Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node",
            name="livox_ros_driver2", output="screen",
            condition=IfCondition(LaunchConfiguration("start_lidar")),
            parameters=[{
                "xfer_format": 4, "multi_topic": 0, "data_src": 0,
                "publish_freq": 20.0, "frame_id": "front_mid360",
                "user_config_path": existing["livox_ros_driver2"]["ros__parameters"]["user_config_path"],
            }],
        ),
        Node(
            package="point_lio", executable="pointlio_mapping", name="point_lio",
            output="screen", parameters=[lio],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="mid360_test_map_tf", output="screen",
            arguments=["--frame-id", "map", "--child-frame-id", "camera_init"],
        ),
        # Point-LIO calls its odometry child 'body' but broadcasts 'aft_mapped'.
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="mid360_test_body_tf", output="screen",
            arguments=["--frame-id", "aft_mapped", "--child-frame-id", "body"],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="mid360_test_scan", output="screen",
            parameters=[{
                "target_frame": "aft_mapped", "transform_tolerance": 0.1,
                "min_height": LaunchConfiguration("min_height"),
                "max_height": LaunchConfiguration("max_height"),
                "min_intensity": 0.0, "max_intensity": 255.0,
                "angle_min": -3.1415926, "angle_max": 3.1415926,
                "angle_increment": 0.00872665, "scan_time": 0.05,
                "range_min": 0.5, "range_max": 10.0, "use_inf": True,
            }],
            remappings=[("cloud_in", "cloud_registered"),
                        ("scan", "mid360_test/scan")],
        ),
        Node(
            package="slam_toolbox", executable="sync_slam_toolbox_node",
            name="slam_toolbox", output="screen", parameters=[slam],
        ),
        Node(
            executable="/usr/bin/python3", name="mid360_test_recorder",
            arguments=[recorder_path],
            parameters=[{"output_dir": LaunchConfiguration("output_dir")}],
            output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz_mid360_test",
            arguments=["-d", os.path.join(bringup, "rviz/mid360_mapping_test.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")), output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("recorder_script"),
        DeclareLaunchArgument("min_height", default_value="0.05"),
        DeclareLaunchArgument("max_height", default_value="1.5"),
        OpaqueFunction(function=mapping_setup),
    ])
