//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "core/system/slam.h"
#include "core/lio/pointcloud_preprocess.h"
#include "utils/timer.h"
#include "wrapper/ros_utils.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <lightning/srv/save_map.hpp>

DEFINE_string(config, "./config/default.yaml", "配置文件");

using namespace lightning;

class RosSlamNode : public rclcpp::Node {
   public:
    using SaveMapService = srv::SaveMap;

    RosSlamNode(const std::string& yaml_path) : Node("lightning_slam"), yaml_path_(yaml_path) {
        // Init SLAM
        SlamSystem::Options options;
        options.online_mode_ = true;
        slam_ = std::make_shared<SlamSystem>(options);
        if (!slam_->Init(yaml_path_)) {
            LOG(ERROR) << "failed to init slam";
            return;
        }

        // Init Preprocess
        preprocess_ = std::make_shared<PointCloudPreprocess>();
        if (!InitPreprocess()) {
            LOG(ERROR) << "failed to init pointcloud preprocess";
            return;
        }

        slam_->StartSLAM("new_map");

        auto yaml = YAML::LoadFile(yaml_path_);
        std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
        std::string cloud_topic = yaml["common"]["lidar_topic"].as<std::string>();
        std::string livox_topic = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                slam_->ProcessIMU(imu);
            });

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                Timer::Evaluate([&]() { 
                    CloudPtr cloud(new PointCloudType());
                    preprocess_->Process(msg, cloud);
                    cloud->header.stamp = ToNanoSec(msg->header.stamp);
                    slam_->ProcessLidar(cloud); 
                }, "Proc Lidar", true);
            });

        livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic, qos, [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                Timer::Evaluate([&]() { 
                    CloudPtr cloud(new PointCloudType());
                    preprocess_->Process(msg, cloud);
                    cloud->header.stamp = ToNanoSec(msg->header.stamp);
                    slam_->ProcessLidar(cloud); 
                }, "Proc Lidar", true);
            });

        savemap_service_ = this->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) {
                std::string save_path = "./data/" + req->map_id + "/";
                slam_->SaveMap(save_path);
                res->response = 0;
            });

        LOG(INFO) << "online slam node has been created.";
    }

    ~RosSlamNode() {
        Timer::PrintAll();
    }

   private:
    bool InitPreprocess() {
        try {
            auto yaml = YAML::LoadFile(yaml_path_);
            preprocess_->Blind() = yaml["fasterlio"]["blind"].as<double>();
            preprocess_->TimeScale() = yaml["fasterlio"]["time_scale"].as<double>();
            int lidar_type = yaml["fasterlio"]["lidar_type"].as<int>();
            preprocess_->NumScans() = yaml["fasterlio"]["scan_line"].as<int>();
            preprocess_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

            float height_max = yaml["roi"]["height_max"].as<float>();
            float height_min = yaml["roi"]["height_min"].as<float>();
            preprocess_->SetHeightROI(height_max, height_min);

            if (lidar_type == 1) {
                preprocess_->SetLidarType(LidarType::AVIA);
            } else if (lidar_type == 2) {
                preprocess_->SetLidarType(LidarType::VELO32);
            } else if (lidar_type == 3) {
                preprocess_->SetLidarType(LidarType::OUST64);
            } else if (lidar_type == 4) {
                preprocess_->SetLidarType(LidarType::ROBOSENSE);
            } else {
                LOG(WARNING) << "unknown lidar_type";
                return false;
            }
        } catch (...) {
            LOG(ERROR) << "bad conversion in preprocess init";
            return false;
        }
        return true;
    }

    std::string yaml_path_;
    std::shared_ptr<SlamSystem> slam_;
    std::shared_ptr<PointCloudPreprocess> preprocess_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Service<SaveMapService>::SharedPtr savemap_service_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;
    google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RosSlamNode>(FLAGS_config);
    rclcpp::spin(node);

    rclcpp::shutdown();
    LOG(INFO) << "done";

    return 0;
}