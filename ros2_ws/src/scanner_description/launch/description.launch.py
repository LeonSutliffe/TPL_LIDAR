"""Broadcast the scanner's tf tree from the tilt axis JointState.

    ros2 launch scanner_description description.launch.py

robot_state_publisher's default input topic is "joint_states"; the driver
node publishes on ~/joint_state (i.e. /tilt_axis_bridge/joint_state), so
that gets remapped below.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory("scanner_description"), "urdf", "scanner.urdf.xacro"
    )
    # Deferred import + xacro.process_file at launch-description-generation
    # time is fine here: this file is only evaluated once per `ros2 launch`.
    import xacro

    robot_description = xacro.process_file(xacro_path).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        # publish_frequency defaults to 20.0 Hz -- exactly the VLP-16's
        # point cloud rate. That tie made most /velodyne_points messages
        # land ahead of the newest /tf sample by the time RViz looked it
        # up, throwing "extrapolation into the future" and dropping most
        # clouds (looked like "a lot of error status" + a slow update
        # rate, even though joint_state itself and /velodyne_points were
        # both healthy). robot_state_publisher republishes /tf on its own
        # timer at this rate regardless of how often joint_state arrives,
        # so raising it here (not the joint_state rate) is what actually
        # gives every cloud's stamp a fresh-enough transform to resolve
        # against.
        parameters=[{"robot_description": robot_description, "publish_frequency": 60.0}],
        remappings=[("joint_states", "/tilt_axis_bridge/joint_state")],
    )

    return LaunchDescription([robot_state_publisher])
