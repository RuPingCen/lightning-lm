# Lightning-LM for DORA
 
将ROS2环境下的lightning-lm增加DORA框架 API接口
 
## 修改说明
1、 把slam.cc中所有与ROS相关的代码(比如函数接口、ROS参数)移动到run_slam_online.cc、run_slam_offline.cc、run_loc_offine.cc 和 run_loc_online.cc中， 

2、 把loc_system.cc中所有与ROS相关的代码(比如函数接口、ROS参数)移动到 run_loc_online.cc run_loc_offline.cc

3、 localization/localization.cpp localization/localization.h 注释对ros 点云和livox点云消息类型支持，改为统一的输入接口   void Localization::ProcessLidarMsg(CloudPtr cloud) 

4、 在 run_loc_online.cc 增加了TF发布、轨迹、odom发布


5、 注释 core/lio/laser_mapping.h 中 preprocess_，以及处理处理ROS2和livox的点云函数 ProcessPointCloud2()


5、 添加ROS2和DORA编译脚本。编译时若需要编译为DORA环境下的节点，修改Lightning-LM/CMakeLists.txt 中的第3行代码，将 BUILD_FRAMEWORK 设置为DORA，反之设置为ROS2

TODO:
cmake/packages.cmake 下还有ROS2的依赖库没有处理掉

## ROS框架
### 建图
```
ros2 run lightning run_slam_online --config src/lightning-lm/config/default_nclt.yaml 
ros2 bag  play 20120115/
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: new_map}"
pcl_viewer ./data/new_map/global.pcd
```
### 定位
```

```
## DORA框架下启动节点

1、 修改Lightning-LM/CMakeLists.txt 中的第3行代码，将 BUILD_FRAMEWORK 设置为DORA

2、新建build文件夹，编译代码

```
source ~/ros2_ws/install/setup.bash
mkdir build
cd build 
cmake ..
make
```

3、启动DORA节点

```
dora up
dora run  lightning_lm.yml
```
 
需要修改 lightning_lm.yml 文件中pointcloud_imu_pub_dora_node节点的路径，指向 pointcloud_imu_pub_dora_node 的可执行文件
 
