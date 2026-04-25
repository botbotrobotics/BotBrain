import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    
    robot_name = config['robot_name']
    robot_model = config['robot_model']
    has_own_robot_state_publisher = config.get('has_own_robot_state_publisher', False)

    launch_actions = []

    if not has_own_robot_state_publisher:
        robot_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('bot_description'),
                    'launch',
                    'robot_description.launch.py'
                )
            )
        )
        launch_actions.append(robot_description_launch)

    # k1 has no interface_link in its URDF, so skipping the botbrain description
    # avoids a second disconnected TF tree that breaks rtabmap's odom→base_link lookup.
    if robot_model != 'k1':
        botbrain_description_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('bot_description'),
                    'launch',
                    'botbrain_description.launch.py'
                )
            )
        )
        botbrain_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_botbrain_base',
            namespace=robot_name,
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', f'{robot_name}/interface_link', f'{robot_name}/botbrain_base'],
            output='screen'
        )
        launch_actions += [botbrain_description_launch, botbrain_static_tf_node]

    return LaunchDescription(launch_actions)
