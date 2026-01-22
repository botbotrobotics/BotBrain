import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml


def generate_launch_description():
    # Read robot configuration
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']

    robot_name = config['robot_name']
    prefix = robot_name + '/' if robot_name != '' else ''

    description_file_type = 'xacro'
    
    # Set description package name
    description_package_name = "bot_description"
    description_package_path = os.path.join(get_package_share_directory(description_package_name))

    # Set file paths based on description type
    description_file = f"botbrain.{description_file_type}"
    description_file_path = os.path.join(description_package_path, description_file_type, description_file)


    mappings = {'prefix': prefix}
    robot_description = xacro.process_file(description_file_path, mappings=mappings).toxml()

    
    params = {'robot_description': robot_description, 'use_sim_time': False}

    # Runs the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='botbrain_state_publisher',
        output='screen',
        parameters=[params],
        namespace=robot_name,
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static"), ("robot_description", "botbrain_description")]
    )
    
    return LaunchDescription([
        robot_state_publisher,
    ])
