#!/bin/bash
# Launches the full scanner ROS2 stack. Kept as its own file (rather than
# inlined as a quoted string passed through PowerShell -> wsl.exe -> bash)
# specifically to avoid multi-layer quote-escaping bugs -- confirmed this
# session that passing the equivalent command as a Start-Process
# -ArgumentList string corrupts it before it reaches micromamba.
exec ~/micromamba-bin/micromamba run -r ~/micromamba -n ros2 bash -c "source /mnt/d/Downloads/LIDAR/ros2_ws/install/setup.bash && ros2 launch scanner_bringup bringup.launch.py"
