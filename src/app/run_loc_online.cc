//
// Created by xiang on 25-3-18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <tf2_ros/transform_broadcaster.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "core/lio/pointcloud_preprocess.h"
#include "core/localization/localization.h"
#include "core/system/loc_system.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

DEFINE_string(config, "./config/default.yaml", "配置文件");

using namespace lightning;

class RosLocNode : public rclcpp::Node {
   public:
    RosLocNode(const std::string& yaml_path) : Node("lightning_loc"), yaml_path_(yaml_path) {
        LocSystem::Options opt;
        loc_system_ = std::make_shared<LocSystem>(opt);

        if (!loc_system_->Init(yaml_path_)) {
            LOG(ERROR) << "failed to init loc system";
            return;
        }

        // Init Preprocess
        preprocess_ = std::make_shared<PointCloudPreprocess>();
        if (!InitPreprocess()) {
            LOG(ERROR) << "failed to init pointcloud preprocess";
            return;
        }

        auto yaml = YAML::LoadFile(yaml_path_);
        std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
        std::string cloud_topic = yaml["common"]["lidar_topic"].as<std::string>();
        std::string livox_topic = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("localization/pose", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("localization/path", 10);

        pub_tf_ = yaml["system"]["pub_tf"].as<bool>();
        if (pub_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                loc_system_->ProcessIMU(imu);
            });

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                Timer::Evaluate(
                    [&]() {
                        CloudPtr cloud(new PointCloudType());
                        preprocess_->Process(msg, cloud);
                        cloud->header.stamp = ToNanoSec(msg->header.stamp);
                        loc_system_->ProcessLidar(cloud);

                        // pub
                        std::shared_ptr<lightning::loc::Localization> loc_value = loc_system_->GetLoc();
                        Sophus::SE3d pose = loc_value->GetLocalizationResult()->pose_;
                        PublishResult(pose, msg->header.stamp, pub_tf_);
                    },
                    "Proc Lidar", true);
            });

        livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic, qos, [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
                Timer::Evaluate(
                    [&]() {
                        CloudPtr cloud(new PointCloudType());
                        preprocess_->Process(msg, cloud);
                        cloud->header.stamp = ToNanoSec(msg->header.stamp);
                        loc_system_->ProcessLidar(cloud);

                        // pub
                        std::shared_ptr<lightning::loc::Localization> loc_value = loc_system_->GetLoc();
                        Sophus::SE3d pose = loc_value->GetLocalizationResult()->pose_;
                        PublishResult(pose, msg->header.stamp, pub_tf_);
                    },
                    "Proc Lidar", true);
            });

        loc_system_->SetInitPose(SE3());
        LOG(INFO) << "online loc node has been created.";
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
    /**
     * @brief 封装结果发布函数
     * @param pose 算法输出的位姿 (SE3)
     * @param timestamp 时间戳 (秒)
     * @param pub_tf 是否发布 TF 变换
     */
    void PublishResult(const Sophus::SE3d& pose, rclcpp::Time stamp, bool pub_tf) {
        Eigen::Quaterniond q = pose.unit_quaternion();
        Eigen::Vector3d t = pose.translation();

        // 1. 发布 TF (map -> base_link)
        if (pub_tf) {
            Eigen::Quaterniond q_inv = pose.inverse().unit_quaternion();
            Eigen::Vector3d t_inv = pose.inverse().translation();
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = "map";
            tf_msg.child_frame_id = "base_link";
            tf_msg.transform.translation.x = t_inv.x();
            tf_msg.transform.translation.y = t_inv.y();
            tf_msg.transform.translation.z = t_inv.z();
            tf_msg.transform.rotation.x = q_inv.x();
            tf_msg.transform.rotation.y = q_inv.y();
            tf_msg.transform.rotation.z = q_inv.z();
            tf_msg.transform.rotation.w = q_inv.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }

        // 2. 发布 PoseStamped
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = t.x();
        pose_msg.pose.position.y = t.y();
        pose_msg.pose.position.z = t.z();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        pose_pub_->publish(pose_msg);

        // 3. 发布 Path (轨迹)
        path_msg_.header.stamp = stamp;
        path_msg_.header.frame_id = "map";
        path_msg_.poses.push_back(pose_msg);
        // 限制轨迹长度，防止内存溢出 (例如保留最近 10000 个点)
        if (path_msg_.poses.size() > 10000) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }
        path_pub_->publish(path_msg_);
    }

    std::string yaml_path_;
    std::shared_ptr<LocSystem> loc_system_;
    std::shared_ptr<PointCloudPreprocess> preprocess_;

    bool pub_tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> path_pub_;
    nav_msgs::msg::Path path_msg_;  // 用于累积轨迹

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
};

/// 运行定位的测试
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_colorlogtostderr = true;
    FLAGS_stderrthreshold = google::INFO;

    google::ParseCommandLineFlags(&argc, &argv, true);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<RosLocNode>(FLAGS_config);
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}