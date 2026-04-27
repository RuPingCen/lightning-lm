//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "core/lio/pointcloud_preprocess.h"
#include "core/localization/localization.h"
#include "ui/pangolin_window.h"
#include "utils/timer.h"
#include "wrapper/bag_io.h"
#include "wrapper/ros_utils.h"

DEFINE_string(input_bag, "", "输入数据包");
DEFINE_string(config, "./config/default.yaml", "配置文件");
DEFINE_string(map_path, "./data/new_map/", "地图路径");

/// 运行定位的测试
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

    loc::Localization::Options options;
    options.online_mode_ = false;

    loc::Localization loc(options);
    loc.Init(FLAGS_config, FLAGS_map_path);

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

    std::string lidar_topic = yaml_node["common"]["lidar_topic"].as<std::string>();
    std::string imu_topic = yaml_node["common"]["imu_topic"].as<std::string>();

    rosbag
        .AddImuHandle(imu_topic,
                      [&loc](IMUPtr imu) {
                          loc.ProcessIMUMsg(imu);
                          usleep(1000);
                          return true;
                      })
        .AddPointCloud2Handle(lidar_topic,
                              [&loc, &preprocess](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                                  CloudPtr cloud(new PointCloudType());
                                  preprocess.Process(msg, cloud);
                                  cloud->header.stamp = ToNanoSec(msg->header.stamp);
                                  loc.ProcessLidarMsg(cloud);
                                  usleep(1000);
                                  return true;
                              })
        .AddLivoxCloudHandle("/livox/lidar",
                             [&loc, &preprocess](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                                 CloudPtr cloud(new PointCloudType());
                                 preprocess.Process(msg, cloud);
                                 cloud->header.stamp = ToNanoSec(msg->header.stamp);
                                 loc.ProcessLidarMsg(cloud);
                                 usleep(1000);
                                 return true;
                             })
        .Go();

    Timer::PrintAll();
    loc.Finish();

    LOG(INFO) << "done";

    return 0;
}