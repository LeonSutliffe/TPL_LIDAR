"""Start scan_aggregator with its default params.

    ros2 launch scan_aggregator aggregator.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_path = os.path.join(
        get_package_share_directory("scan_aggregator"), "config", "params.yaml"
    )
    node = Node(
        package="scan_aggregator",
        executable="scan_aggregator_node",
        name="scan_aggregator",
        output="screen",
        parameters=[params_path],
    )
    return LaunchDescription([node])
