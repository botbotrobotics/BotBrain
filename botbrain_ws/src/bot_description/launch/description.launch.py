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

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_description'),
                'launch',
                'robot_description.launch.py'
            )
        )
    )

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
    
    return LaunchDescription([
        robot_description_launch,
        botbrain_description_launch,
        botbrain_static_tf_node,
    ])
