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
    
    robot_model = config['robot_model']
    robot_name = config['robot_name']
    prefix = robot_name + '/' if robot_name != '' else ''
    description_file_type = config['description_file_type']
    
    # Set description package name
    description_package_name = f"{robot_model}_pkg"
    description_package_path = os.path.join(get_package_share_directory(description_package_name))
    
    # Set file paths based on description type
    description_file = f"robot.{description_file_type}"
    description_file_path = os.path.join(description_package_path, description_file_type, description_file)

    # Handle different description file types
    if description_file_type == 'xacro':
        mappings = {'prefix': prefix}
        robot_description = xacro.process_file(description_file_path, mappings=mappings).toxml()
    else:  # urdf
        with open(description_file_path, 'r') as f:
            robot_description = f.read()
    
    params = {'robot_description': robot_description, 'use_sim_time': False}

    # Runs the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
        namespace=robot_name,
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")]
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
