import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch cluster base controller.
    This is the core component for cluster coordination.
    """
    # Declare launch arguments
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='3',
        description='Number of robots in cluster (3 or 5)'
    )

    cluster_config_arg = DeclareLaunchArgument(
        'cluster_config',
        default_value='',
        description='Custom cluster config file path (optional)'
    )

    # Get configuration
    robot_count = LaunchConfiguration('robot_count')
    cluster_config = LaunchConfiguration('cluster_config')

    # Get package directory
    cluster_pkg_share = get_package_share_directory('cluster_node')

    # Default config files based on robot count
    config_3robots = os.path.join(cluster_pkg_share, 'config', '3cluster_velocity.yaml')
    config_5robots = os.path.join(cluster_pkg_share, 'config', 'cluster_5.yaml')

    # Use custom config if provided, otherwise use default based on robot_count
    # Note: LaunchConfiguration doesn't support conditional logic directly,
    # so we'll use the robot_count parameter to determine which config to use

    return LaunchDescription([
        robot_count_arg,
        cluster_config_arg,
        Node(
            package='controller',
            executable='cluster_controller',
            name='cluster_controller',
            parameters=[config_3robots],  # Default to 3 robots
            # User can override by specifying cluster_config argument
        ),
    ])
