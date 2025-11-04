import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch data logging for cluster robots.
    Logs live data to CSV files for analysis.
    """
    # Get package directory
    adaptive_nav_util_share = get_package_share_directory('adaptive_navigation_utilities')

    # Launch file path
    csv_logger_launch = os.path.join(adaptive_nav_util_share, 'live_data_to_csv.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(csv_logger_launch)
        ),
    ])
