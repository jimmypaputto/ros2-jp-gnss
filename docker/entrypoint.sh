#!/bin/bash
# Sources ROS and jp_gnss_hat, then runs the command (default: ros2 launch, see Dockerfile).
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /opt/jp_gnss_hat/setup.bash
exec "$@"
