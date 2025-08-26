# auto_explore_mapper

Autonomous exploration when mapping Currently compatible：

- **Cartographer**
- **Navigation2**

涉及到的包：
1.auto_explore_mapper
2.base_driver
3.ld_lidar_ros2
4.open_source_slam_launch
5.robot_pose_publisher
6.robot_interfaces

## ~~一. 在gazebo仿真环境中运行自动建图~~

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

## 三. 使用luxshare robot在真实环境下运行

### 3.1 启动底盘相关节点，包含激光雷达、里程计、以及各个tf关系

```
ros2 launch base_driver base_driver.launch.py
ros2 launch ldlidar_driver_ros2 ldlidar_driver.launch.py
```

### 3.2 启动auto_explore_mapper相关算法

```bashrc
ros2 launch auto_explore_mapper luxsharerobot_auto_explore_mapping_whole.launch.py
```

## 3.3 启动之后查看当前自动建图的状态

```bashrc
ros2 topic echo /auto_explore_mapping/state
```

- 状态解释： 参考robot_interfaces/msg/AutoExploreMappingState.msg

```
当前状态，-1:未初始化; 0:已初始化未开始; 1:开始自动建图; 2:建图中; 3:停止自动建图;
```

## 3.4 激活/停止自动建图,保存当前地图
- 激活/停止自动建图, 保存当前地图, 参考robot_interfaces/msg/AutoExploreMappingTrigger.msg
  **1:开始自动建图; 3:停止自动建图; 5：保存当前地图;**
- **注意，只有在开始自动建图之后，停止自动建图之前 才能保存当前地图**
- 开始自动建图

```
ros2 topic pub -1 /auto_explore_mapping/trigger robot_interfaces/msg/AutoExploreMappingTrigger "to_state: 1"
```

- 保存当前地图

```
ros2 topic pub -1 /auto_explore_mapping/trigger robot_interfaces/msg/AutoExploreMappingTrigger "to_state: 5"
```

- 停止自动建图

```
ros2 topic pub -1 /auto_explore_mapping/trigger robot_interfaces/msg/AutoExploreMappingTrigger "to_state: 3"
```

- 保存地图名称及目录：
名称: AutoExploreMap_当前时间戳.pgm 与 AutoExploreMap_当前时间戳.yaml文件
目录: 当前home目录  

- 地图topic名称: /map 

- 自动探索建图停止条件
  - 手动停止，发送停止自动建图topic
  - 自动停止, 查看auto_explore_mapper.cpp以下代码部分
  ```bashrc
  void Explore() {
    ....
    if(frontiers.size()<=5){
      .....
    }
  }
  ```