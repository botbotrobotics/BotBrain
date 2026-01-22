#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
import yaml
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
    network_interface = config['network_interface']
    prefix = robot_name + '/' if robot_name != '' else ''

    controller_commands_node = LifecycleNode(
        package = 'go2_pkg',
        executable = 'go2_controller_commands.py',
        name='controller_commands_node',
        namespace=robot_name,
        output='screen'
    )

    go2_read_node = LifecycleNode(
        package = 'go2_pkg',
        executable = 'go2_read.py',
        parameters=[{'prefix': (prefix)}],
        name='robot_read_node',
        namespace=robot_name,
        output='screen'
    )

    go2_write_node = LifecycleNode(
        package = 'go2_pkg',
        executable = 'go2_write.py',
        parameters=[{'prefix': (prefix)}],
        name='robot_write_node',
        namespace=robot_name,
        output='screen'
    )

    go2_video_stream_node = LifecycleNode(
        package = 'go2_pkg',
        executable = 'go2_video_stream.py',
        parameters=[{'prefix': (prefix), 'network_interface': network_interface}],
        name='robot_video_stream',
        namespace=robot_name,
        output='screen'
    )

    configure_handler_for_write = RegisterEventHandler(
        OnProcessStart(
            target_action=go2_write_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(go2_write_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_write = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=go2_write_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(go2_write_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    configure_handler_for_read = RegisterEventHandler(
        OnProcessStart(
            target_action=go2_read_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(go2_read_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_read = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=go2_read_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(go2_read_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    configure_handler_for_commands = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_commands_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(controller_commands_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_commands = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=controller_commands_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(controller_commands_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    configure_handler_for_video_stream = RegisterEventHandler(
        OnProcessStart(
            target_action=go2_video_stream_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(go2_video_stream_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_video_stream = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=go2_video_stream_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(go2_video_stream_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    return LaunchDescription(
        [
            go2_read_node ,
            go2_write_node,
            controller_commands_node,
            go2_video_stream_node,
            
            # Handlers
            # configure_handler_for_write,
            # activate_handler_for_write,
            # configure_handler_for_read,
            # activate_handler_for_read,
            # configure_handler_for_commands,
            # activate_handler_for_commands,
            # configure_handler_for_video_stream,
            # activate_handler_for_video_stream,
        ]
    )