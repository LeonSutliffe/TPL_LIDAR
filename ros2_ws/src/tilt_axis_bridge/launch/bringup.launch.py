"""Bring up the tilt axis bridge node plus rosbridge_server (websocket, for
the web GUI in D:\\Downloads\\LIDAR\\web\\tilt_axis_gui) with one command:

    ros2 launch tilt_axis_bridge bringup.launch.py
    ros2 launch tilt_axis_bridge bringup.launch.py serial_port:=/dev/ttyUSB1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0")

    tilt_axis_node = Node(
        package="tilt_axis_bridge",
        executable="tilt_axis_bridge_node",
        name="tilt_axis_bridge",
        output="screen",
        parameters=[{"serial_port": LaunchConfiguration("serial_port")}],
    )

    # rosbridge_server's stock rosbridge_websocket_launch.xml also starts
    # rosapi_node unconditionally (~80MB RSS by itself, measured) -- our
    # hand-rolled GUI client only ever does plain topic pub/sub and direct
    # service calls, never anything through /rosapi/*, so it's dead weight
    # here. Worth the RAM back on a memory-constrained field unit (Pi),
    # so this launches rosbridge_websocket directly instead of including
    # that bundle.
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
    )

    return LaunchDescription([serial_port_arg, tilt_axis_node, rosbridge])
