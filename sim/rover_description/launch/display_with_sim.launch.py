import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
import launch_ros
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')

    # RViz config file path
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/cluster3withhw.rviz')

    return LaunchDescription([
        # Specify RViz config file path
        DeclareLaunchArgument("rvizconfig", default_value=default_rviz_config_path, description="Absolute path to rviz config file"),

        # Enable time for Gazebo and simulation
        DeclareLaunchArgument("use_sim_time", default_value="True", description="Flag to enable use_sim_time"),

        # Launch RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
        Node(
            package="register_service",
            executable="register_service",
            name="robot_register_server",
            output="screen",
        ),
        Node(
            package="fake_rover_state_controller",
            executable="fake_rover",
            name="fake_rover",
            output="screen",
            parameters=[{
                'robot_id': "p2",
                'x': 0.0,
                'y': 0.0,
                't': 0.0,
                'prefix': '/sim'
            }]
        ),
        Node(
            package="fake_rover_state_controller",
            executable="fake_rover",
            name="fake_rover",
            output="screen",
            parameters=[{
                'robot_id': "p3",
                'x': 5.0,
                'y': 5.0,
                't': 0.0,
                'prefix': '/sim'
            }]
        ),
        Node(
            package="fake_rover_state_controller",
            executable="fake_rover",
            name="fake_rover",
            output="screen",
            parameters=[{
                'robot_id': "p4",
                'x': 5.0,
                'y': -5.0,
                't': 0.0,
                'prefix': '/sim'
            }]
        ),
        Node(
            package="fake_rover_state_controller",
            executable="jointstate_publisher",
            name="jointstate_publisher",
            output="screen",
            parameters=[{
                'robot_id': "p2, p3, p4",
            }]
        ),
    ])
