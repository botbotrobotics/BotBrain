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

  fixed_frame_id = f'{prefix}odom'
  frame_id = f'{prefix}base_link'

  odom_topic = f'{prefix}/odom'
  lidar_topic = f'{prefix}/pointcloud'
  lidar_topic_deskewed = lidar_topic + '/deskewed'

  use_sim_time = False
  rgbd_image_used = False
  deskewing = True
  deskewing_slerp = True

  voxel_size_value = 0.1

  database_path_arg = DeclareLaunchArgument(
      'database_path',
      description='Path to the RTAB-Map database file'
  )
  database_path = LaunchConfiguration('database_path')

  if not fixed_frame_id or not deskewing:
    lidar_topic_deskewed = lidar_topic     

  max_correspondence_distance = voxel_size_value * 10.0  

  shared_parameters = {
    'use_sim_time': use_sim_time,
    'frame_id': frame_id,
    'qos': 1,
    'approx_sync': True,
    'wait_for_transform': 0.5,

    'Icp/PointToPlane': 'true',
    'Icp/Iterations': '10',
    'Icp/VoxelSize': str(voxel_size_value),         
    'Icp/Epsilon': '0.001',                         
    'Icp/PointToPlaneK': '20',
    'Icp/PointToPlaneRadius': '0',
    'Icp/MaxTranslation': '3',
    'Icp/MaxCorrespondenceDistance': str(max_correspondence_distance),  
    'Icp/Strategy': '1',
    'Icp/OutlierRatio': '0.7',                    
  }

  rtabmap_parameters = {
    'subscribe_depth': False,
    'subscribe_rgb': False,
    'subscribe_odom_info': False,
    'subscribe_scan_cloud': True,
    'map_frame_id': 'map',
    'database_path': database_path,

    'RGBD/ProximityMaxGraphDepth': '0',
    'RGBD/ProximityPathMaxNeighbors': '1',
    'RGBD/AngularUpdate': '0.05',
    'RGBD/LinearUpdate': '0.05',
    'RGBD/CreateOccupancyGrid': 'false',
    'Mem/NotLinkedNodesKept': 'false',
    'Mem/STMSize': '30',
    'Reg/Strategy': '1',
    'Icp/CorrespondenceRatio': '0.2',
    'Mem/IncrementalMemory': 'False',
    'Mem/InitWMWithAllNodes': 'True'
  }
  
  nodes = [
    Node(
      package='rtabmap_slam', executable='rtabmap', output='screen',
      parameters=[shared_parameters, rtabmap_parameters, {'subscribe_rgbd': rgbd_image_used}],
      remappings=[
        ('odom', odom_topic),
        ('scan_cloud', lidar_topic_deskewed)
      ]
    )
  ]
  
  if fixed_frame_id and deskewing:
    nodes.append(
      Node(
        package='rtabmap_util', executable='lidar_deskewing', output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'fixed_frame_id': fixed_frame_id,
          'wait_for_transform': 0.5,  
          'slerp': deskewing_slerp}],
        remappings=[
            ('input_cloud', lidar_topic),
            ('output_cloud', lidar_topic_deskewed)
        ])
    )
      
  return LaunchDescription([database_path_arg] + nodes)
