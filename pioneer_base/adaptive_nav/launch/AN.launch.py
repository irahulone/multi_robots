import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    # Get the directory of this launch file
    package_name = 'cluster_node'
    pkg_share = get_package_share_directory(package_name)
    # Construct paths to the parameter files relative to the launch file directory
    cluster_file = os.path.join(pkg_share, 'config', '3cluster.yaml')
    display_launch_file = os.path.join(get_package_share_directory('rover_description'), 'launch', 'display.launch.py')
    pioneer_launch_file = os.path.join(get_package_share_directory('rover_description'), 'launch', 'pioneer.launch.py')
    # Check if parameter files exist
    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    return LaunchDescription([
        Node(
            package="adaptive_nav",
            executable="adaptive_nav",
            parameters=[cluster_file],
        ),
        Node(
            package="rf_sim",
            executable="rf_field",
        ),
        Node(
            package="virtual_joy",
            executable="virtual_joy",
        ),
        Node(
        package="teleop_core",
        executable="cmd_demux",
        parameters=["pioneer_base/teleop_core/config/demux.yaml"],
        ),
        Node(
        package="teleop_core",
        executable="joywithgui",
        parameters=["pioneer_base/teleop_core/config/joy-assign.yaml"],
        ),   
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_launch_file),
            launch_arguments={
                'robot_id': 'p2',
                'x': '5.0', 
                'y': '-5.0',
                't': '0.0',
                'desired': 'desired',
                'a': '0.2'
            }.items()
        ),
    ])