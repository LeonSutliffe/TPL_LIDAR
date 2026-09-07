"""Start scan_aggregator with its default params.

    ros2 launch scan_aggregator aggregator.launch.py

No static params file is passed here -- the node's own declare_parameter
calls already read persisted values from ~/.lidar_scanner_settings.json
(see node.py's _load_settings_section/_default), falling back to
hardcoded literals if nothing's been saved yet. A `parameters=[<yaml>]`
launch override here used to duplicate those same literals in a
packaged config/params.yaml -- which, being an explicit launch-time
override, unconditionally beat the settings-file value on every restart
regardless of what had been persisted (found 2026-09-07 investigating
why a Pi field deployment's output_dir kept reverting after every
reboot -- this is also the likely explanation for the previously-
unexplained "mount_yaw_deg silently lost to a settings-file reset"
symptom from earlier project history). Removed rather than fixed in
place, since the node's own defaults already matched the YAML's values
exactly and duplicating them was the entire problem.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package="scan_aggregator",
        executable="scan_aggregator_node",
        name="scan_aggregator",
        output="screen",
    )
    return LaunchDescription([node])
