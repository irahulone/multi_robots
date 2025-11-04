import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Integrated launch file for 3-robot cluster with hardware.
    This combines all necessary components for full cluster operation.

    Components:
    - Cluster base controller
    - Visualization (hardware mode)
    - Simulation/Hardware interface
    - Teleoperation
    - Data logging
    """
    # Get package directory
    base_launch_share = get_package_share_directory('base_launch')

    # Component launch files
    cluster_base_launch = os.path.join(base_launch_share, 'launch', 'cluster_base.launch.py')
    cluster_viz_launch = os.path.join(base_launch_share, 'launch', 'cluster_visualization.launch.py')
    cluster_sim_launch = os.path.join(base_launch_share, 'launch', 'cluster_simulation.launch.py')
    cluster_teleop_launch = os.path.join(base_launch_share, 'launch', 'cluster_teleop.launch.py')
    cluster_logger_launch = os.path.join(base_launch_share, 'launch', 'cluster_logger.launch.py')

    return LaunchDescription([
        # Core controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_base_launch),
            launch_arguments={'robot_count': '3'}.items(),
        ),
        # Visualization (hardware mode)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_viz_launch),
            launch_arguments={
                'use_hardware': 'true',
                'robot_count': '3'
            }.items(),
        ),
        # Hardware interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_sim_launch),
            launch_arguments={'use_hardware': 'true'}.items(),
        ),
        # Teleoperation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_teleop_launch),
            launch_arguments={'robot_count': '3'}.items(),
        ),
        # Data logging
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_logger_launch),
        ),
    ])
