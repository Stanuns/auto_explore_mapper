"""
launch all the node when using autonomous exploration mapping
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    

    slam_launch_file_dir = os.path.join(get_package_share_directory('open_source_slam_launch'), 'launch')
    robot_pose_publisher_launch_file_dir = os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch')
    auto_explore_mapper_launch_file_dir = os.path.join(get_package_share_directory('auto_explore_mapper'), 'launch')
    nav2_map_server_launch_file_dir = os.path.join(get_package_share_directory('nav2_map_server'), 'launch')
    ekf_carto_config = os.path.join(get_package_share_directory('base_driver'), 'config', 'ekf_carto.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch_file_dir, '/luxsharerobot_cartographer_mapping.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch_file_dir, '/luxsharerobot_nav2_auto_explore_mapping.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_pose_publisher_launch_file_dir, '/pose_publisher.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([auto_explore_mapper_launch_file_dir, '/auto_explore_mapper.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_map_server_launch_file_dir, '/map_saver_server.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),

        # Node(
        #     #condition=UnlessCondition(carto_slam),
        #     package='robot_localization', 
        #     executable='ekf_node', 
        #     name = 'carto_ekf_filter_node',
        #     parameters=[ekf_carto_config]
        #     # remappings=[("/odom", "/odom")]
        # ),
    ])
