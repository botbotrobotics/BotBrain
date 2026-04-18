#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
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

    k1_read_node = LifecycleNode(
        package = 'k1_pkg',
        executable = 'k1_read.py',
        parameters=[{'prefix': (prefix)}],
        name='robot_read_node',
        namespace=robot_name,
        output='screen'
    )

    k1_write_node = LifecycleNode(
        package = 'k1_pkg',
        executable = 'k1_write.py',
        parameters=[{'prefix': (prefix)}],
        name='robot_write_node',
        namespace=robot_name,
        output='screen'
    )

    configure_handler_for_write = RegisterEventHandler(
        OnProcessStart(
            target_action=k1_write_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(k1_write_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_write = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=k1_write_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(k1_write_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    configure_handler_for_read = RegisterEventHandler(
        OnProcessStart(
            target_action=k1_read_node,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(k1_read_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )
    activate_handler_for_read = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=k1_read_node,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(k1_read_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    # Convert depth image to LaserScan for Nav2 costmaps and collision monitor
    depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        namespace=robot_name,
        name='depthimage_to_laserscan_front',
        remappings=[
            ('depth', '/depth/image'),
            ('depth_camera_info', '/rgb/camera_info'),
            ('scan', 'front_camera/scan'),
        ],
        parameters=[{
            'range_max': 3.5,
            'range_min': 0.3,
            'scan_height': 3,
            'output_frame': 'camera_link',
        }]
    )

    # Bridge odom→base_link (from k1_read) to Trunk (URDF root)
    base_link_to_trunk = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_trunk',
        arguments=['0', '0', '0', '0', '0', '0', f'{prefix}base_link', 'trunk_link']
    )

    # Depth camera frame offset from head pitch link (85 mm up)
    head_pitch_to_camera_depth = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_pitch_to_camera_depth_frame',
        arguments=['0', '0', '0.085', '0', '0', '0', 'head_pitch_link', 'camera_link']
    )

    return LaunchDescription(
        [
            k1_read_node,
            k1_write_node,
            depth_to_scan_node,
            base_link_to_trunk,
            head_pitch_to_camera_depth,

            # Handlers
            # configure_handler_for_write,
            # activate_handler_for_write,
            # configure_handler_for_read,
            # activate_handler_for_read
        ]
    )