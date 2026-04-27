# Lightning-LM for DORA

将ROS2环境下的lightning-lm增加DORA框架 API接口

![dora_lightning_lm_slam](doc/dora_lightning_lm_slam.gif)

## 修改说明
1、 把slam.cc中所有与ROS相关的代码(比如函数接口、ROS参数)移动到run_slam_online.cc、run_slam_offline.cc、run_loc_offine.cc 和 run_loc_online.cc中， 

2、 把loc_system.cc中所有与ROS相关的代码(比如函数接口、ROS参数)移动到 run_loc_online.cc run_loc_offline.cc

3、 localization/localization.cpp localization/localization.h 注释对ros 点云和livox点云消息类型支持，改为统一的输入接口   void Localization::ProcessLidarMsg(CloudPtr cloud) 

4、 在 run_loc_online.cc 增加了TF发布、轨迹、odom发布


5、 注释 core/lio/laser_mapping.h 中 preprocess_，以及处理处理ROS2和livox的点云函数 ProcessPointCloud2()


6、 添加ROS2和DORA编译脚本。编译时若需要编译为DORA环境下的节点，修改Lightning-LM/CMakeLists.txt 中的第3行代码，将 BUILD_FRAMEWORK 设置为DORA，反之设置为ROS2

7、去ROS化（修改了以下文件）

    src/common/options.h:14:10:  注释 rclcpp/rclcpp.hpp
    src/core/lightning_math.hpp  注释 头文件、函数 inline PoseRPYD SE3ToRollPitchYaw(const SE3& pose)  RpyToRotM(const T r, const T p, const T y)  . SE3ToRollPitchYaw函数被其他文件调用，这里对这个函数进行了修改
    src/wrapper/ros_utils.h
    src/core/g2p5/g2p5_map.h  注释 函数 nav_msgs::msg::OccupancyGrid G2P5Map::ToROS()
    src/core/localization/localization_result.cc
    slam.cc 注释 函数  存为ROS兼容的模式地图 if (options_.with_gridmap_) {xxxx}



## DORA框架下启动节点

1、 修改Lightning-LM/CMakeLists.txt 中的第3行代码，将 BUILD_FRAMEWORK 设置为DORA

2、新建build文件夹，编译代码

```
mkdir build
cd build 
cmake ..
make
```

3、启动DORA框架下的建图节点

```
dora up
dora run  dora_slam_online_node.yml
```

需要修改 lightning_lm.yml 文件中pointcloud_imu_pub_dora_node节点的路径，指向 pointcloud_imu_pub_dora_node 的可执行文件

4、启动DORA框架下的定位节点

```
dora up
dora run  dora_loc_online_node.yml
```

注意修改 dora_loc_online_node.yml 中指向的配置文件 default_nclt_dora.yaml， 在 default_nclt_dora.yaml 文件中设置 “map_path:”地图存储的路径

 