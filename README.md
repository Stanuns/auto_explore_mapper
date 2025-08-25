# auto_explore_mapper

Autonomous exploration when mappingCurrently compatible：

- **cartographer**

## 一. 在gazebo仿真环境中运行自动建图

```bashrc
ros2 launch robot_description gazebo_robot_world.launch.py 

ros2 launch open_source_slam_launch cartographer_mapping.launch.py 

ros2 launch open_source_slam_launch nav2_autonomous_exploration.launch.py 

ros2 launch robot_pose_publisher pose_publisher.launch.py 

ros2 launch auto_explore_mapper auto_explore_mapper.launch.py

ros2 launch nav2_map_server map_saver_server.launch.py
```

## ~~二.使用wheeltec diff robot在真实环境下运行~~

### ~~启动底盘~~

```bashrc
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

### ~~启动auto_explore_mapper相关算法~~

```bashrc
ros2 launch auto_explore_mapper wheeltec_auto_explore_mapping_whole.launch.py
```

## 三.使用luxshare robot在真实环境下运行

### 启动底盘相关节点，包含激光雷达、里程计、以及各个tf关系

```
ros2 launch base_driver base_driver.launch.py
ros2 launch ldlidar_driver_ros2 ldlidar_driver.launch.py
```

### 启动auto_explore_mapper相关算法

```bashrc
ros2 launch auto_explore_mapper luxsharerobot_auto_explore_mapping_whole.launch.py
```

## 启动之后查看当前自动建图的状态

```bashrc
ros2 topic echo /auto_explore_mapping/state
```

- 状态解释： 参考robot_interfaces/msg/AutoExploreMappingState.msg

```
当前状态，-1:未初始化; 0:已初始化未开始; 1:开始自动建图; 2:建图中; 3:停止自动建图;
```

- 激活/停止自动建图, 保存当前地图, 参考robot_interfaces/msg/AutoExploreMappingTrigger.msg
 **1:开始自动建图; 3:停止自动建图; 5：保存当前地图;**

```
ros2 topic pub -1 /auto_explore_mapping/trigger robot_interfaces/msg/AutoExploreMappingTrigger "to_state: 1"
```

- 保存地图名称及目录：

名称: AutoExploreMap_当前时间戳.pgm 与 AutoExploreMap_当前时间戳.yaml文件
目录: 当前home目录

- 地图topic名称: /map