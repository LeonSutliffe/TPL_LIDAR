"""Bring up the VLP-16 driver + pointcloud transform with this scanner's config.

    ros2 launch scanner_bringup velodyne.launch.py
    ros2 launch scanner_bringup velodyne.launch.py enable_pointcloud:=false

enable_pointcloud:=false skips velodyne_transform_node -- the per-point
XYZ conversion, which is the single biggest CPU cost in this whole stack.
Meant for a small field unit (e.g. a Pi) that only records raw
/velodyne_packets via rosbag2 and defers point-cloud conversion + the
tf2 merge in scan_aggregator to post-processing on a real PC later.
scan_aggregator still runs fine without it -- its move/home/settle state
machine doesn't depend on /velodyne_points, only its own end-of-run PCD
write does (which will just report "no points captured", harmlessly,
since that step is meant to happen later anyway in this mode).
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_pointcloud_arg = DeclareLaunchArgument("enable_pointcloud", default_value="true")

    config_path = os.path.join(
        get_package_share_directory("scanner_bringup"), "config", "vlp16.yaml"
    )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    driver_params = config["velodyne_driver_node"]["ros__parameters"]
    transform_params = config["velodyne_transform_node"]["ros__parameters"]
    transform_params["calibration"] = os.path.join(
        get_package_share_directory("velodyne_pointcloud"), "params", "VLP16db.yaml"
    )

    driver_node = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        output="screen",
        parameters=[driver_params],
    )

    transform_node = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_transform_node",
        output="screen",
        parameters=[transform_params],
        condition=IfCondition(LaunchConfiguration("enable_pointcloud")),
    )

    return LaunchDescription([enable_pointcloud_arg, driver_node, transform_node])
