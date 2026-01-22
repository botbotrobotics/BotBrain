import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bot_navigation_dir = get_package_share_directory('bot_navigation')

    nav_utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_navigation_dir, 'launch', 'nav_utils.launch.py')
        )
    )

    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_navigation_dir, 'launch', 'nav2.launch.py')
        )
    )

    return LaunchDescription([
        nav_utils_launch,
        navigation2_launch,
    ])
