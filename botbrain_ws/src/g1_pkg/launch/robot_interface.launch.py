#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
import yaml
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import EnvironmentVariable, TextSubstitution
from launch.events import matches_action
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    
    robot_name = config['robot_name']
    prefix = robot_name + '/' if robot_name != '' else ''

    pkg_share = get_package_share_directory('g1_pkg')
    params_file = os.path.join(pkg_share, 'config', 'g1_params.yaml')

    g1_write_node = LifecycleNode(
        package = 'g1_pkg',
        executable = 'g1_write_node',
        parameters=[params_file, {'prefix': (prefix)}],
        name='robot_write_node',
        namespace=robot_name,
        output='screen',
        additional_env={
            "LD_LIBRARY_PATH": "/usr/local/lib:/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:"
                              + os.environ.get("LD_LIBRARY_PATH", ""),
            "LD_PRELOAD": "/usr/local/lib/libddsc.so.0:/usr/local/lib/libddscxx.so.0",
        },
    )

    g1_read_node = LifecycleNode(
        package = 'g1_pkg',
        executable = 'g1_read.py',
        parameters=[{'prefix': (prefix)}],
        name='robot_read_node',
        namespace=robot_name,
        output='screen'
    )

    g1_controller_commands_node = LifecycleNode(
        package = 'g1_pkg',
        executable = 'g1_controller_commands.py',
        parameters=[{'prefix': (prefix)}],
        name='controller_commands_node',
        namespace=robot_name,
        output='screen'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'livox_MID360.launch.py')),
        launch_arguments={'prefix': prefix}.items()
    )

    pointcloud_to_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("g1_pkg"), "launch"),
                "/pc2ls.launch.py",
            ]
        ),
    )

    return LaunchDescription(
        [
            g1_write_node,
            g1_read_node,
            g1_controller_commands_node,
            livox_launch,
            pointcloud_to_scan
        ]
    )