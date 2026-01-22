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
    robot_model = config['robot_model']

    # Create the node
    rtab_manager_node = LifecycleNode(
        package='bot_localization',
        executable='rtab_manager.py',
        namespace=robot_name,
        name='rtab_manager',
        output='screen',
        parameters=[{
            'robot_model': robot_model,
        }]
    )
    
    return LaunchDescription([
        rtab_manager_node,
    ]) 