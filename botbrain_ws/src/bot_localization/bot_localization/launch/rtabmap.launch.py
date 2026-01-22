from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')

    with open(config_file, 'r') as f:
        _raw_robot = yaml.safe_load(f)['robot_configuration']

    robot_name  = _raw_robot['robot_name']
    robot_model = _raw_robot['robot_model']
    robot_package_name = f"{robot_model}_pkg"
    default_map = _raw_robot.get('default_map', 'rtabmap.db')

    # Define the database path centrally
    database_path = os.path.join(
        get_package_share_directory(robot_package_name),
        'maps',
        default_map
    )

    camera_cfg_file = os.path.join(
        get_package_share_directory(robot_package_name),
        "config",
        "camera_config.yaml",
    )

    with open(camera_cfg_file, "r") as f:
        _raw_cam = yaml.safe_load(f)["camera_configuration"]

    front_type = (_raw_cam.get('front') or {}).get('type', '')
    back_type  = (_raw_cam.get('back')  or {}).get('type', '')

    num_cameras = 0
    if front_type:
        num_cameras += 1
    if back_type:
        num_cameras += 1

    nodes = []

    if robot_model == "g1":
        lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "rtabmap_lidar.launch.py")
            ),
            launch_arguments={'database_path': database_path}.items(),
        )
        nodes.append(lidar)
    else:
        if num_cameras == 0:
            pass

        elif num_cameras == 1:
            single_camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rtabmap_single_camera.launch.py")
                ),
                launch_arguments={'database_path': database_path}.items(),
            )
            nodes.append(single_camera)

        else:
            double_camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "rtabmap_double_camera.launch.py")
                ),
                launch_arguments={'database_path': database_path}.items(),
            )
            nodes.append(double_camera)

    return LaunchDescription(nodes)
