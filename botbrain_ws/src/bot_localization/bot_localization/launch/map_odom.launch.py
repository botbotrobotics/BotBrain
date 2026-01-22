#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# --- Required imports ---
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
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
    
    # Create the node
    map_odom_node = LifecycleNode(
        package='bot_localization',
        executable='map_odom.py',
        name='map_odom_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            "prefix": prefix
        }]
    )    
    return LaunchDescription([
        map_odom_node,
    ]) 