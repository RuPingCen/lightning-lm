//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "core/system/slam.h"
#include "core/lio/pointcloud_preprocess.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

#include "io/yaml_io.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");

/// 运行一个LIO前端，带可视化
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);
    if (FLAGS_input_bag.empty()) {
        LOG(ERROR) << "未指定输入数据";
        return -1;
    }

    using namespace lightning;

    RosbagIO rosbag(FLAGS_input_bag);

    SlamSystem::Options options;
    options.online_mode_ = false;

    SlamSystem slam(options);

    /// 实时模式好像掉帧掉的比较厉害？

    if (!slam.Init(FLAGS_config)) {
        LOG(ERROR) << "failed to init slam";
        return -1;
    }

    PointCloudPreprocess preprocess;
    auto yaml_node = YAML::LoadFile(FLAGS_config);
    preprocess.Blind() = yaml_node["fasterlio"]["blind"].as<double>();
    preprocess.TimeScale() = yaml_node["fasterlio"]["time_scale"].as<double>();
    int lidar_type = yaml_node["fasterlio"]["lidar_type"].as<int>();
    preprocess.NumScans() = yaml_node["fasterlio"]["scan_line"].as<int>();
    preprocess.PointFilterNum() = yaml_node["fasterlio"]["point_filter_num"].as<int>();
    float height_max = yaml_node["roi"]["height_max"].as<float>();
    float height_min = yaml_node["roi"]["height_min"].as<float>();
    preprocess.SetHeightROI(height_max, height_min);

    if (lidar_type == 1) {
        preprocess.SetLidarType(LidarType::AVIA);
    } else if (lidar_type == 2) {
        preprocess.SetLidarType(LidarType::VELO32);
    } else if (lidar_type == 3) {
        preprocess.SetLidarType(LidarType::OUST64);
    } else if (lidar_type == 4) {
        preprocess.SetLidarType(LidarType::ROBOSENSE);
    }

    slam.StartSLAM("new_map");

    std::string lidar_topic = yaml_node["common"]["lidar_topic"].as<std::string>();
    std::string imu_topic = yaml_node["common"]["imu_topic"].as<std::string>();

    rosbag
        /// IMU 的处理
        .AddImuHandle(imu_topic,
                      [&slam](IMUPtr imu) {
                          slam.ProcessIMU(imu);
                          return true;
                      })

        /// lidar 的处理
        .AddPointCloud2Handle(lidar_topic,
                              [&slam, &preprocess](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                  CloudPtr cloud(new PointCloudType());
                                  preprocess.Process(msg, cloud);
                                  cloud->header.stamp = ToNanoSec(msg->header.stamp);
                                  slam.ProcessLidar(cloud);
                                  return true;
                              })
        /// livox 的处理
        .AddLivoxCloudHandle("/livox/lidar",
                             [&slam, &preprocess](livox_ros_driver2::msg::CustomMsg::SharedPtr cloud_msg) {
                                 CloudPtr cloud(new PointCloudType());
                                 preprocess.Process(cloud_msg, cloud);
                                 cloud->header.stamp = ToNanoSec(cloud_msg->header.stamp);
                                 slam.ProcessLidar(cloud);
                                 return true;
                             })
        .Go();

    slam.SaveMap("");
    Timer::PrintAll();

    LOG(INFO) << "done";

    return 0;
}