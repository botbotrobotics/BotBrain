from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(launch_dir)))))
    config_file = os.path.join(workspace_dir, 'robot_config.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['robot_configuration']
    robot_name = config['robot_name']
    prefix = robot_name + '/' if robot_name != '' else ''

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        description='Path to the RTAB-Map database file'
    )
    database_path = LaunchConfiguration('database_path')

    # Stereo camera publishes to absolute topics via <robot>_read.py:
    #   /rgb/image        (sensor_msgs/Image)
    #   /depth/image      (sensor_msgs/Image)
    #   /rgb/camera_info  (sensor_msgs/CameraInfo)
    #   /odom             (nav_msgs/Odometry)
    rtabmap_localization_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=robot_name,
        output='screen',
        parameters=[
            {
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "frame_id": f'{prefix}base_link',
                "map_frame_id": f'{prefix}map',
                "odom_frame_id": f'{prefix}odom',
                "approx_sync": True,
                "wait_for_transform": 0.5,
                'approx_sync_max_interval': 0.1,
                'sync_queue_size': 10,
                'use_sim_time': False,
                'Rtabmap/DetectionRate': '1.0',
                'Reg/Force3DoF': 'true',
                'Grid/RayTracing': 'true',
                'Grid/3D': 'false',
                'Grid/RangeMax': '3.0',
                'Grid/CellSize': '0.10',
                'Grid/NormalsSegmentation': 'true',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '0.8',
                'Optimizer/GravitySigma': '0.0',
                "delete_db_on_start": False,
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True',
                'Mem/DepthAsMask': 'False',
                'Vis/MaxFeatures': '500',
                'Vis/MinInliers': '10',
            },
            {
                'database_path': database_path,
            },
        ],
        remappings=[
            # Publishes to absolute topics — remap with leading /
            ("rgb/image", "/rgb/image"),
            ("depth/image", "/depth/image"),
            ("rgb/camera_info", "/rgb/camera_info"),
            ("odom", "/odom"),
        ]
    )

    return LaunchDescription([
        database_path_arg,
        rtabmap_localization_node,
    ])
