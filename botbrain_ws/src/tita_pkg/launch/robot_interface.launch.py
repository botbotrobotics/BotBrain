#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
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
    tita_namespace = config.get('tita_namespace', '')

    # --- Lifecycle nodes ---
    tita_write_node = LifecycleNode(
        package='tita_pkg',
        executable='tita_write.py',
        name='robot_write_node',
        namespace=robot_name,
        parameters=[{'tita_namespace': tita_namespace}],
        output='screen'
    )

    tita_read_node = LifecycleNode(
        package='tita_pkg',
        executable='tita_read.py',
        name='robot_read_node',
        namespace=robot_name,
        parameters=[{'tita_namespace': tita_namespace}],
        output='screen'
    )

    controller_commands_node = LifecycleNode(
        package='tita_pkg',
        executable='tita_controller_commands.py',
        name='controller_commands_node',
        namespace=robot_name,
        parameters=[{'tita_namespace': tita_namespace}],
        output='screen'
    )

    # -- Handlers for tita_write_node --
    configure_handler_for_write = RegisterEventHandler(
        OnProcessStart(
            target_action=tita_write_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(tita_write_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_write = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=tita_write_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(tita_write_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    # -- Handlers for tita_read_node --
    configure_handler_for_read = RegisterEventHandler(
        OnProcessStart(
            target_action=tita_read_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(tita_read_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_read = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=tita_read_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(tita_read_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    # -- Handlers for controller_commands_node --
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

    return LaunchDescription([
        # Nodes
        tita_write_node,
        tita_read_node,
        controller_commands_node,
        # Handlers
        #configure_handler_for_write,
        #activate_handler_for_write,
        #configure_handler_for_read,
        #activate_handler_for_read,
        #configure_handler_for_commands,
        #activate_handler_for_commands,
    ])