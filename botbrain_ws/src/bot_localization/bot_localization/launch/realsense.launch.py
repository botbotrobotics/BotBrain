#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node   # <-- importe Node também
from ament_index_python.packages import get_package_share_directory
import os
import yaml

from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch_ros.event_handlers import OnStateTransition
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from lifecycle_msgs.msg import Transition

def make_camera_params(serial: str | None, camera_side: str, tf_prefix: str, rtsp_port: int):
    # Sensors run at 15 fps for the browser stream; ROS topics stay at the 6 Hz RTAB-Map is tuned for.
    return {
        "serial_no": serial or "",
        "camera_name": f"{camera_side}_camera",
        "tf_prefix": tf_prefix,
        "initial_reset": True,
        "depth_width": 640,
        "depth_height": 480,
        "depth_fps": 15,
        "color_width": 640,
        "color_height": 480,
        "color_fps": 15,
        "color_rate": 6.0,
        "depth_rate": 6.0,
        "align_depth": True,
        "stream_rtsp_port": rtsp_port,
    }

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')

    with open(config_file, 'r') as f:
        _raw_robot = yaml.safe_load(f)['robot_configuration']
   
    robot_name  = _raw_robot['robot_name']
    robot_model = _raw_robot['robot_model']

    prefix = robot_name + '/' if robot_name != '' else ''
    
    camera_cfg_file = os.path.join(
        get_package_share_directory(f"{robot_model}_pkg"),
        "config",
        "camera_config.yaml",    
    )

    with open(camera_cfg_file, "r") as f:
        _raw_cam = yaml.safe_load(f)["camera_configuration"]

    front_camera = (_raw_cam.get('front') or {}).get('type', '')
    front_serial = (_raw_cam.get('front') or {}).get('serial_number', '')
    back_camera  = (_raw_cam.get('back')  or {}).get('type', '')
    back_serial  = (_raw_cam.get('back') or {}).get('serial_number', '')

    nodes = []

    if front_camera and front_camera.strip():
        front_camera_node = LifecycleNode(
            name='front_camera',
            namespace=robot_name,
            package='bot_realsense',
            executable='bot_realsense_node',
            parameters=[make_camera_params(serial=front_serial, camera_side="front", tf_prefix=prefix, rtsp_port=8560)],
            output='screen'
        )
        nodes.append(front_camera_node)

        front_cfg = _raw_cam.get('front', {}) or {}
        tf = front_cfg.get('tf', {}) or {}

        x = str(tf.get('x', 0))
        y = str(tf.get('y', 0))
        z = str(tf.get('z', 0))
        roll  = str(tf.get('roll', 0))
        pitch = str(tf.get('pitch', 0))
        yaw   = str(tf.get('yaw', 0))

        parent_frame = front_cfg.get('parent_frame', f'{prefix}base_link')
        child_frame  = front_cfg.get('child', f'{prefix}front_camera_link')

        front_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='front_static_tf_base_to_camera',
            namespace=robot_name,
            arguments=[x, y, z, yaw, pitch, roll, parent_frame, child_frame],
            output='screen',
        )
        nodes.append(front_static_tf_node)

        scan_node_front = Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            namespace=robot_name,
            name='depthimage_to_laserscan_front',
            remappings=[
                ('depth', 'front_camera/depth/image_rect_raw'),
                ('depth_camera_info', 'front_camera/depth/camera_info'),
                (('scan', 'front_camera/scan'))
            ],
            parameters=[{
                'range_max': 5.0,
                'range_min': 0.3,
                'scan_height': 3,
                'output_frame': f'{prefix}front_camera_link',
            }]
        )
        nodes.append(scan_node_front)

    if back_camera and back_camera.strip():
        back_camera_node = LifecycleNode(
            name='back_camera',
            namespace=robot_name,
            package='bot_realsense',
            executable='bot_realsense_node',
            parameters=[make_camera_params(serial=back_serial, camera_side="back", tf_prefix=prefix, rtsp_port=8561)],
            output='screen'
        )
        nodes.append(back_camera_node)

        back_cfg = _raw_cam.get('back', {}) or {}
        tf = back_cfg.get('tf', {}) or {}

        x = str(tf.get('x', 0))
        y = str(tf.get('y', 0))
        z = str(tf.get('z', 0))
        roll  = str(tf.get('roll', 0))
        pitch = str(tf.get('pitch', 0))
        yaw   = str(tf.get('yaw', 0))

        parent_frame = back_cfg.get('parent_frame', f'{prefix}base_link')
        child_frame  = back_cfg.get('child', f'{prefix}back_camera_link')

        back_static_tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='back_static_tf_base_to_camera',
            namespace=robot_name,
            arguments=[x, y, z, yaw, pitch, roll, parent_frame, child_frame],
            output='screen',
        )
        nodes.append(back_static_tf_node)

    scan_node_back = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        namespace=robot_name,
        name='depthimage_to_laserscan_back',
        remappings=[
            ('depth', 'back_camera/depth/image_rect_raw'),
            ('depth_camera_info', 'back_camera/depth/camera_info'),
            ('scan', 'back_camera/scan')
        ],
        parameters=[{
            'range_max': 5.0,
            'range_min': 0.3,
            'scan_height': 10,
            'output_frame': f'{prefix}back_camera_link',
        }]
    )
    nodes.append(scan_node_back)

    return LaunchDescription(nodes)
