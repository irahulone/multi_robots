import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch adaptive navigation for autonomous cluster control.
    """
    # Declare launch arguments
    cluster_config_arg = DeclareLaunchArgument(
        'cluster_config',
        default_value='',
        description='Cluster config file path'
    )

    # Get configuration
    cluster_config = LaunchConfiguration('cluster_config')

    # Get package directory
    cluster_pkg_share = get_package_share_directory('cluster_node')

    # Default config file
    default_config = os.path.join(cluster_pkg_share, 'config', '3cluster_velocity.yaml')

    return LaunchDescription([
        cluster_config_arg,
        Node(
            package='adaptive_nav',
            executable='adaptive_nav',
            name='adaptive_nav_node',
            parameters=[default_config],
        ),
    ])
