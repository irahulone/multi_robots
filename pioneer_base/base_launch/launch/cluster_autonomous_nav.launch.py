import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Integrated launch file for autonomous navigation with cluster robots.
    This combines controller, autonomous navigation, visualization, and teleoperation.

    Supports both simulation and hardware modes via use_hardware argument.

    Components:
    - Cluster base controller
    - Adaptive navigation
    - Visualization (switches between sim/hardware)
    - Simulation/Hardware interface
    - Teleoperation
    """
    # Declare launch arguments
    use_hardware_arg = DeclareLaunchArgument(
        'use_hardware',
        default_value='false',
        description='Toggle hardware launch instead of simulation components'
    )

    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='3',
        description='Number of robots in cluster (3 or 5)'
    )

    # Get configuration
    use_hardware = LaunchConfiguration('use_hardware')
    robot_count = LaunchConfiguration('robot_count')

    # Get package directory
    base_launch_share = get_package_share_directory('base_launch')

    # Component launch files
    cluster_base_launch = os.path.join(base_launch_share, 'launch', 'cluster_base.launch.py')
    cluster_autonomous_launch = os.path.join(base_launch_share, 'launch', 'cluster_autonomous.launch.py')
    cluster_viz_launch = os.path.join(base_launch_share, 'launch', 'cluster_visualization.launch.py')
    cluster_sim_launch = os.path.join(base_launch_share, 'launch', 'cluster_simulation.launch.py')
    cluster_teleop_launch = os.path.join(base_launch_share, 'launch', 'cluster_teleop.launch.py')

    return LaunchDescription([
        use_hardware_arg,
        robot_count_arg,
        # Core controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_base_launch),
            launch_arguments={'robot_count': robot_count}.items(),
        ),
        # Autonomous navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_autonomous_launch),
        ),
        # Visualization (switches based on use_hardware)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_viz_launch),
            launch_arguments={
                'use_hardware': use_hardware,
                'robot_count': robot_count
            }.items(),
        ),
        # Simulation/Hardware interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_sim_launch),
            launch_arguments={'use_hardware': use_hardware}.items(),
        ),
        # Teleoperation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cluster_teleop_launch),
            launch_arguments={'robot_count': robot_count}.items(),
        ),
    ])
