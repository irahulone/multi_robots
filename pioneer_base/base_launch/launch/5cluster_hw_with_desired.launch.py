import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

    # Get the directory of this launch file
    cluster_package_name = 'cluster_node'
    cluster_pkg_share = get_package_share_directory(cluster_package_name)
    # Construct paths to the parameter files relative to the launch file directory
    cluster_file = os.path.join(cluster_pkg_share, 'config', 'cluster_5.yaml')

    # Check if parameter files exist
    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    display_launch_file = os.path.join(get_package_share_directory('rover_description'), 'launch', 'display_with_hw_desired.launch.py')
    pioneer_launch_file = os.path.join(get_package_share_directory('sim_launch'), 'pioneer_with_hw_desired.launch.py')
    teleop_launch_file = os.path.join(get_package_share_directory("teleop_core"), 'gui.launch.py')

    return LaunchDescription([
        Node(
            package="controller",
            executable="cluster_controller",
            parameters=[cluster_file],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_file)
        ),
    ])

