import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch teleoperation components for cluster control.
    Includes virtual joystick, command demux, and GUI.
    """
    # Declare launch arguments
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='3',
        description='Number of robots (3 or 5)'
    )

    use_virtual_joy_arg = DeclareLaunchArgument(
        'use_virtual_joy',
        default_value='true',
        description='Launch virtual joystick node'
    )

    # Get configuration
    robot_count = LaunchConfiguration('robot_count')
    use_virtual_joy = LaunchConfiguration('use_virtual_joy')

    # Get package directories
    teleop_share = get_package_share_directory('teleop_core')

    # Config file paths
    teleop_config_dir = os.path.join(teleop_share, 'config')
    demux_config = os.path.join(teleop_config_dir, 'demux.yaml')
    joy_assign_config = os.path.join(teleop_config_dir, 'joy-assign.yaml')

    # Launch file paths
    gui_3_launch = os.path.join(teleop_share, 'gui_3.launch.py')
    gui_launch = os.path.join(teleop_share, 'gui.launch.py')

    # Check if config files exist
    if not os.path.isfile(demux_config):
        raise FileNotFoundError(f"Parameter file not found: {demux_config}")
    if not os.path.isfile(joy_assign_config):
        raise FileNotFoundError(f"Parameter file not found: {joy_assign_config}")

    return LaunchDescription([
        robot_count_arg,
        use_virtual_joy_arg,
        # Virtual joystick node (when no hardware joystick available)
        Node(
            package='virtual_joy',
            executable='virtual_joy',
            name='virtual_joy_node',
        ),
        # Command demultiplexer
        Node(
            package='teleop_core',
            executable='cmd_demux',
            name='cmd_demux_node',
            parameters=[demux_config],
        ),
        # Joystick with GUI (supports both 3 and 5 robot modes)
        Node(
            package='teleop_core',
            executable='joywithgui3',
            name='joywithgui_node',
            parameters=[joy_assign_config],
        ),
        # GUI launch for 3 robots (default)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gui_3_launch),
        ),
    ])
