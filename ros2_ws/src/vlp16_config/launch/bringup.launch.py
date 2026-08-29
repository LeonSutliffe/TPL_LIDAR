"""Start vlp16_config with its default params.

    ros2 launch vlp16_config bringup.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package="vlp16_config",
        executable="vlp16_config_node",
        name="vlp16_config",
        output="screen",
    )
    return LaunchDescription([node])
