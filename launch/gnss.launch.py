# Jimmy Paputto 2026
#
# GnssNode is a rclcpp_lifecycle::LifecycleNode.
# Auto-transitions (configure → activate) are handled in Main.cpp
# so the node works out of the box with both `ros2 run` and `ros2 launch`.
#
# For manual lifecycle control (node name is jp_gnss_hat, or
# jp_gnss_hat_<node_id> when node_id is set):
#   ros2 lifecycle list /jp_gnss_hat
#   ros2 lifecycle set /jp_gnss_hat configure
#   ros2 lifecycle set /jp_gnss_hat activate
#
# Multi-node RTK setup — launch two nodes with separate config files:
#   ros2 launch jp_gnss_hat gnss.launch.py config_file:=/path/to/base.yaml
#   ros2 launch jp_gnss_hat gnss.launch.py config_file:=/path/to/rover.yaml
#
# Each config should have a unique "node_id" field (e.g. "base", "rover").
# The node name and all topics are derived from it automatically.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    pkg_share = get_package_share_directory('jp_gnss_hat')
    default_config = os.path.join(pkg_share, 'config', 'gnss_config.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to GNSS configuration YAML file'
    )

    config_path = LaunchConfiguration('config_file')

    gnss_node = Node(
        package='jp_gnss_hat',
        executable='gnss_node',
        output='screen',
        parameters=[{
            'config_file': config_path
        }],
        # Also pass config_file as a CLI-style ROS arg so Main.cpp can read
        # the YAML (and thus node_id) BEFORE constructing the LifecycleNode.
        # Without this, parameters arrive only via --params-file, which is
        # parsed by rclcpp after node construction — too late to influence
        # the node name and topic prefixes derived from node_id.
        arguments=['--ros-args', '-p', ['config_file:=', config_path]],
    )

    return LaunchDescription([
        config_arg,
        gnss_node,
    ])
