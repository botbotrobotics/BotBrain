import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition
import xacro
import yaml


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    
    robot_name = config['robot_name']

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time')
    
    # Configure and launch twist mux
    config_file = os.path.join(get_package_share_directory('bot_bringup'),
                                         'config', 'twist_mux.yaml')
    
    twist_mux_launch = Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                config_file
            ],
            namespace=robot_name
    )
    
    zero_vel_publisher = LifecycleNode(
        package='bot_bringup',
        executable='zero_vel_publisher.py',
        output='screen',
        name='zero_vel_publisher',
        namespace=robot_name
    )

    configure_handler_for_zero_vel = RegisterEventHandler(
        OnProcessStart(
            target_action=zero_vel_publisher,
            on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(zero_vel_publisher),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))]
        )
    )

    activate_handler_for_zero_vel = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=zero_vel_publisher,
            goal_state='inactive',
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(zero_vel_publisher),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        twist_mux_launch,
        zero_vel_publisher,
    ])
