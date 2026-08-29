"""Full scanner bringup: tilt axis + rosbridge, tf, VLP-16, the
step-and-stare aggregator, and VLP-16 hardware config (vlp16_config), with
optional rosbag2 recording.

    ros2 launch scanner_bringup bringup.launch.py
    ros2 launch scanner_bringup bringup.launch.py serial_port:=/dev/ttyACM1
    ros2 launch scanner_bringup bringup.launch.py record_bag:=true

record_bag follows the project brief's "log raw driver output + joint
states via rosbag2 so calibration/registration can be re-run as
post-processing" -- it records /velodyne_packets (not the already-computed
pointcloud) plus tf and the tilt axis's joint state/status, so a run can be
replayed through a different calibration or lever-arm offset later.

For a small field unit (e.g. a Pi) that only records raw data and defers
point-cloud conversion to a real PC afterward, combine both:

    ros2 launch scanner_bringup bringup.launch.py enable_pointcloud:=false record_bag:=true
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0")
    enable_pointcloud_arg = DeclareLaunchArgument("enable_pointcloud", default_value="true")
    record_bag_arg = DeclareLaunchArgument("record_bag", default_value="false")
    # WSLg makes this a normal Windows-desktop window with no extra setup,
    # and tying it into the launch tree means Shutdown Everything / Ctrl-C
    # closes it along with everything else -- no orphaned window to hunt
    # down separately. rviz:=false to skip it (e.g. a headless/SSH session
    # with no WSLg).
    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")
    bag_dir_arg = DeclareLaunchArgument(
        "bag_dir", default_value=os.path.expanduser("~/lidar_scans/bags")
    )

    tilt_axis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tilt_axis_bridge"), "launch", "bringup.launch.py"
            )
        ),
        launch_arguments={"serial_port": LaunchConfiguration("serial_port")}.items(),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scanner_description"),
                "launch",
                "description.launch.py",
            )
        )
    )

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scanner_bringup"), "launch", "velodyne.launch.py"
            )
        ),
        launch_arguments={"enable_pointcloud": LaunchConfiguration("enable_pointcloud")}.items(),
    )

    aggregator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("scan_aggregator"), "launch", "aggregator.launch.py"
            )
        )
    )

    vlp16_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("vlp16_config"), "launch", "bringup.launch.py"
            )
        )
    )

    bag_name = "scan_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    mkdir_bag_dir = ExecuteProcess(
        cmd=["mkdir", "-p", LaunchConfiguration("bag_dir")],
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )
    bag_record = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            [LaunchConfiguration("bag_dir"), os.sep + bag_name],
            "/velodyne_packets",
            "/tilt_axis_bridge/joint_state",
            "/tilt_axis_bridge/status",
            "/scan_aggregator/status",
            "/tf",
            "/tf_static",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("record_bag")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("scanner_bringup"), "config", "scanner.rviz"
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            serial_port_arg,
            enable_pointcloud_arg,
            record_bag_arg,
            bag_dir_arg,
            rviz_arg,
            tilt_axis_launch,
            description_launch,
            velodyne_launch,
            aggregator_launch,
            vlp16_config_launch,
            mkdir_bag_dir,
            bag_record,
            rviz_node,
        ]
    )
