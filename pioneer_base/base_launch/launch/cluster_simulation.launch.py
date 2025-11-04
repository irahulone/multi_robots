import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch simulation components for cluster robots.
    Includes RF field simulation and hardware interface launchers.
    """
    # Declare launch arguments
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Use hardware instead of simulation'
    )

    use_rf_sim_arg = DeclareLaunchArgument(
        'use_rf_sim',
        default_value='true',
        description='Launch RF field simulation (only in simulation mode)'
    )

    # Get configuration
    use_hardware = LaunchConfiguration('use_hardware')
    use_rf_sim = LaunchConfiguration('use_rf_sim')

    # Get package directory
    sim_launch_share = get_package_share_directory('sim_launch')

    # Launch file paths
    pioneer_hw_launch = os.path.join(sim_launch_share, 'pioneer_with_hw_desired.launch.py')

    return LaunchDescription([
        use_hardware_arg,
        use_rf_sim_arg,
        # RF field simulation (only in simulation mode)
        Node(
            package='rf_sim',
            executable='rf_field',
            name='rf_field_node',
            condition=UnlessCondition(use_hardware),
        ),
        # Hardware pioneer interface (only in hardware mode)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pioneer_hw_launch),
            condition=IfCondition(use_hardware),
        ),
    ])
