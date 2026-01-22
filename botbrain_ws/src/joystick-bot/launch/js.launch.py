#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

# --- Required imports ---
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('joystick_bot')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the Joystick interface node'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'js_config.yaml'),
        description='Path to the configuration file'
    )
    
    # Create the node
    joystick_node = LifecycleNode(
        package='joystick_bot',
        executable='js_node.py',
        name='joystick_interface',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    configure_handler_for_commands = RegisterEventHandler(
        OnProcessStart(
            target_action=joystick_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(joystick_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_commands = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=joystick_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(joystick_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )
    
    return LaunchDescription([
        namespace_arg,
        config_file_arg,
        joystick_node,
    ]) 