import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch visualization for cluster robots.
    Supports both simulation and hardware modes with different RViz configs.
    """
    # Declare launch arguments
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Use hardware RViz config instead of simulation config'
    )

    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='3',
        description='Number of robots (3 or 5)'
    )

    # Get configuration
    use_hardware = LaunchConfiguration('use_hardware')
    robot_count = LaunchConfiguration('robot_count')

    # Get package directories
    rover_desc_share = get_package_share_directory('rover_description')

    # Launch file paths
    display_launch_file = os.path.join(rover_desc_share, 'launch', 'display.launch.py')
    display_hw_launch_file = os.path.join(rover_desc_share, 'launch', 'display_with_hw_desired.launch.py')

    # RViz config paths
    sim_rviz_config = os.path.join(rover_desc_share, 'rviz/clusterp1-p3withdesired.rviz')
    hardware_rviz_config = os.path.join(rover_desc_share, 'rviz/clusterp1-p3withhw.rviz')

    return LaunchDescription([
        use_hardware_arg,
        robot_count_arg,
        # Simulation visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_file),
            launch_arguments={'rvizconfig': sim_rviz_config}.items(),
            condition=UnlessCondition(use_hardware),
        ),
        # Hardware visualization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_hw_launch_file),
            condition=IfCondition(use_hardware),
        ),
    ])
