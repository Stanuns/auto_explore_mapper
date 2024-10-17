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

import launch_ros.actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    

    nav2_launch_file_dir = os.path.join(get_package_share_directory('open_source_slam_launch'), 'launch')
    robot_pose_publisher_launch_file_dir = os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch')
    auto_explore_mapper_launch_file_dir = os.path.join(get_package_share_directory('auto_explore_mapper'), 'launch')
    nav2_map_server_launch_file_dir = os.path.join(get_package_share_directory('nav2_map_server'), 'launch')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/wheeltec_cartographer_mapping.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/wheeltec_nav2_autonomous_exploration.launch.py']),
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
            PythonLaunchDescriptionSource([auto_explore_mapper_launch_file_dir, '/auto_explore_mapper_d2_cartographer.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),

        #launch navigation2 nav2_map_server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_map_server_launch_file_dir, '/map_saver_server.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                }.items(),
        ),
    ])
