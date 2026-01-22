import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro
import yaml


def generate_launch_description():

    # Include the RealSense launch file
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_localization'),
                'launch',
                'realsense.launch.py'
            )
        )
    )

    # Include the RTAB-Map launch file
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_localization'),
                'launch',
                'rtabmap.launch.py'
            )
        )
    )

    # Include the RTAB-Map manager launch file
    rtabmap_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_localization'),
                'launch',
                'rtab_manager.launch.py'
            )
        )
    )

    # Include map odometry launch file
    map_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_localization'),
                'launch',
                'map_odom.launch.py'
            )
        )
    )

    return LaunchDescription([
        realsense_launch,
        rtabmap_launch,
        rtabmap_manager_launch,
        map_odom_launch,
    ])
