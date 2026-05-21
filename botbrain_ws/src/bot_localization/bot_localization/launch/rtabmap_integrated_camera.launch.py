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

    # rgbd_sync subscribes to separate rgb/depth/camera_info topics and publishes
    # a single synchronized RGBDImage topic consumed by rtabmap.
    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'queue_size': 10,
            'use_sim_time': False,
            'qos_image': 2,        # BEST_EFFORT (matches k1_read.py stereo_qos)
            'qos_camera_info': 2,  # BEST_EFFORT
        }],
        remappings=[
            ('rgb/image', '/rgb/image'),
            ('depth/image', '/depth/image'),
            ('rgb/camera_info', '/rgb/camera_info'),
        ]
    )

    # Camera publishes to absolute topics via <robot>_read.py:
    #   /rgb/image        (sensor_msgs/Image)
    #   /depth/image      (sensor_msgs/Image)
    #   /rgb/camera_info  (sensor_msgs/CameraInfo)
    #   /odom             (nav_msgs/Odometry)
    # rtabmap now consumes the synced rgbd_image from rgbd_sync.
    rtabmap_localization_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace=robot_name,
        output='screen',
        parameters=[
            {
                "subscribe_rgbd": True,
                "subscribe_depth": False,
                "subscribe_rgb": False,
                "frame_id": f'{prefix}base_link',
                "map_frame_id": f'{prefix}map',
                "odom_frame_id": f'{prefix}odom',
                "approx_sync": True,
                "wait_for_transform": 0.5,
                'approx_sync_max_interval': 0.1,
                'sync_queue_size': 10,
                'use_sim_time': False,
                'Rtabmap/DetectionRate': '0.5',
                'Reg/Force3DoF': 'true',
                'Grid/RayTracing': 'false',
                'Grid/3D': 'false',
                'Grid/RangeMax': '3.0',
                'Grid/CellSize': '0.10',
                'Grid/NormalsSegmentation': 'false',
                'Grid/MaxGroundHeight': '0.05',
                'Grid/MaxObstacleHeight': '0.8',
                'Optimizer/GravitySigma': '0.0',
                "delete_db_on_start": False,
                'Mem/IncrementalMemory': 'False',
                'Mem/InitWMWithAllNodes': 'True',
                'Mem/DepthAsMask': 'False',
                # 'Grid/NormalsSegmentation': 'false',  # avoid PCL segfault on degenerate depth
                'Vis/MaxFeatures': '200',
                'Vis/MinInliers': '10',
            },
            {
                'database_path': database_path,
            },
        ],
        remappings=[
            ("rgbd_image", "/rgbd_image"),
            ("odom", "/odom"),
        ]
    )

    return LaunchDescription([
        database_path_arg,
        rgbd_sync_node,
        rtabmap_localization_node,
    ])
