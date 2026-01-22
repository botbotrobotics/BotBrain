import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import yaml


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    
    robot_name = config['robot_name']
    robot_model = config['robot_model']
    print(f"Robot name from config: {robot_name}")

    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time')
    
    # Include the description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_description'),
                'launch',
                'description.launch.py'
            )
        )
    )

    # Include the robot interface launch file
    robot_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(robot_model+'_pkg'),
                'launch',
                'robot_interface.launch.py'
            )
        )
    )

    # Include twist mux launch file
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bot_bringup'),
                'launch',
                'twist_mux.launch.py'
            )
        )
    )

    # Include joystick launch file
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('joystick_bot'),
                'launch',
                'js.launch.py'
            )
        ),
        launch_arguments={
            'namespace': robot_name
        }.items()
    )

    # Include rosbridge launch file
    rosbridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        ),
        launch_arguments={
            'cbor_compression': 'true',
            'namespace': robot_name
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        description_launch,
        robot_interface_launch,
        twist_mux_launch,
        joystick_launch,
        rosbridge_launch,
    ])
