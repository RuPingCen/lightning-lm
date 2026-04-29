# rosbag_to_dora

读取rosbag文件，将bag文件中的点云数据数据发布到dora中，在yml文件中通过

修改数据包路径
```
  BAG_PATH: /home/mickrobot/dataset/rosbag2_2024_11_02-15_26_53
```
修改数据发布频率
```
  tick: dora/timer/millis/200
```


## TODO

增加图像、RTK、数据的发布

模块化功能，转化为点云、图像、RTK、IMU、轮速路程计的功能要分开，分别创建一个文件或者函数模块

增加对典型数据集的支持、KITTI、NUCLT等，以读取文件进行数据发布的example