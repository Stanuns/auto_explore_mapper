# auto_explore_mapper

Autonomous exploration when mapping

## 一. 在gazebo仿真环境中运行自动建图

```bashrc
ros2 launch robot_description gazebo_robot_world.launch.py 

ros2 launch open_source_slam_launch cartographer_mapping.launch.py 

ros2 launch open_source_slam_launch nav2_autonomous_exploration.launch.py 

ros2 launch robot_pose_publisher pose_publisher.launch.py 

ros2 launch auto_explore_mapper auto_explore_mapper_d2_cartographer.launch.py

ros2 launch nav2_map_server map_saver_server.launch.py
```

## 二.使用wheeltec diff robot在真实环境下运行

```bashrc
ros2 launch wheeltec_auto_explore_mapper auto_explorer_mapping_whole.launch.py
```
