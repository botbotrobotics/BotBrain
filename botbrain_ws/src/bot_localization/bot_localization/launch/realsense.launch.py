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

def make_camera_params(serial: str | None, camera_name: str, tf_prefix: str, camera_side: str):
    name_prefix = f"{camera_name}_" if camera_name else ""
    params = {
        "serial_no": serial,
        "pointcloud.enable": False,
        "enable_infra": False,
        "enable_infra1": False,
        "enable_infra2": False,
        "enable_gyro": False,
        "enable_accel": False,
        "enable_motion": False,
        "initial_reset": True,
        "accelerate_gpu_with_glsl": True,
        "depth_module.depth_profile": "640x480x6",
        "rgb_camera.color_profile": "640x480x6",
        "rgb_camera.color_format": "MJPEG",
        "unite_imu_method": 2,
        "camera_name": f"{camera_name}{camera_side}_camera",
        'tf_prefix': tf_prefix,
        "align_depth.enable": True,
        "spatial_filter.enable": False,
        "spatial_filter.parameters.filter_magnitude": 5.0,
        "spatial_filter.parameters.filter_smooth_alpha": 0.5,
        "spatial_filter.parameters.filter_smooth_delta": 20.0,
        "temporal_filter.enable": False,
        "temporal_filter.parameters.filter_smooth_alpha": 0.1,
        "temporal_filter.parameters.filter_smooth_delta": 20.0,
        "enable_sync": True,
    }
    return params

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
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[
                make_camera_params(
                    serial=front_serial,      
                    camera_name=robot_name,
                    tf_prefix=robot_name,
                    camera_side="front"
                )
            ],
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
            package='realsense2_camera',
            executable='realsense2_camera_node',
            parameters=[
                make_camera_params(
                    serial=back_serial,
                    camera_name=robot_name,
                    tf_prefix=robot_name,
                    camera_side="back"
                )
            ],
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

    compressed_realsense_node = LifecycleNode(
        package='bot_localization',
        executable='compressed_realsense.py',
        name='realsense_compressed_node',
        namespace=robot_name,
        output='screen'
    )
    nodes.append(compressed_realsense_node)

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
