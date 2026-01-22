import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    robot_name = config['robot_name']
    prefix = robot_name + '/' if robot_name != '' else ''

    nodes_config = os.path.join(
        get_package_share_directory('bot_state_machine'),
        'config'
    )

    state_machine = LifecycleNode(
            package='bot_state_machine',
            executable='state_machine_node',
            name='state_machine',
            namespace=prefix,
            output='screen',
            parameters=[{'nodes_json_path': nodes_config}]
    )

    configure_state_machine = RegisterEventHandler(
        OnProcessStart(
            target_action=state_machine,
            on_start=[  
                TimerAction(  
                    period=20.0, 
                    actions=[  
                        EmitEvent(event=ChangeState(  
                            lifecycle_node_matcher=matches_action(state_machine),  
                            transition_id=Transition.TRANSITION_CONFIGURE,  
                        ))  
                    ],  
                )  
            ],  
        )
    )


    activate_state_machine = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=state_machine,
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(state_machine),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))
            ],
        )
    )

    return LaunchDescription([
        state_machine,  
        configure_state_machine,
        activate_state_machine,
    ])