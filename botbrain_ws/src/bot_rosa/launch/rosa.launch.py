import launch
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']

    robot_name = config['robot_name']
    robot_model = config['robot_model']
    openai_api_key = config['openai_api_key']

    rosa_service_node = LifecycleNode(
        package='bot_rosa',
        executable='rosa_service.py',
        namespace=robot_name,
        name='rosa_service_node',
        output='screen',
        parameters=[{
            'robot_model': robot_model
        }]
    )

    set_openai_api_key = SetEnvironmentVariable(
        name='OPENAI_API_KEY',
        value=openai_api_key
    )

    return LaunchDescription([
        set_openai_api_key,
        rosa_service_node,
    ])