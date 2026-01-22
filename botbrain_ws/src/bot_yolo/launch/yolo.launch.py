#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bot_yolo')
    default_params = os.path.join(pkg_share, 'config', 'yolo.yaml')

    pt_arg = DeclareLaunchArgument('pt_path', default_value='/root/.cache/ultralytics/yolo11n.pt')
    eng_arg = DeclareLaunchArgument('engine_path', default_value='/root/.cache/ultralytics/yolo11n.engine')

    params_arg = DeclareLaunchArgument('params_file', default_value=default_params)

    yolo = LifecycleNode(
        package='bot_yolo',
        executable='yolo_node',
        namespace='',
        name='yolo_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'pt_path': LaunchConfiguration('pt_path'),
                'engine_path': LaunchConfiguration('engine_path'),
            }
        ],
    )

    return LaunchDescription([
        pt_arg,
        eng_arg,
        params_arg,
        yolo,
    ])