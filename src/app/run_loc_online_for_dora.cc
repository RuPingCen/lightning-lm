extern "C" {
#include "node_api.h"
}

#include <yaml-cpp/yaml.h>
#include <atomic>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include "common/eigen_types.h"
#include "common/point_def.h"
#include "core/system/loc_system.h"
#include "utils/timer.h"
// #include "wrapper/ros_utils.h"

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

#include "core/localization/localization.h"
#include "core/system/loc_system.h"

using json = nlohmann::json;
using namespace lightning;

std::atomic<bool> to_exit_process{false};

/**
 * 模仿 PointCloudPreprocess 的 PCL -> PCL 预处理器
 */
class PointCloudPreprocessPCL {
   public:
    PointCloudPreprocessPCL() = default;

    double& Blind() { return blind_; }
    int& PointFilterNum() { return point_filter_num_; }
    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }

    int point_filter_num_ = 1;
    double blind_ = 0.01;
    float height_max_ = 1.0;
    float height_min_ = -1.0;
};

class DoraLocNode {
   public:
    DoraLocNode(const std::string& yaml_path) : yaml_path_(yaml_path) {
        // 初始化定位系统
        LocSystem::Options options;
        loc_system_ = std::make_shared<LocSystem>(options);
        if (!loc_system_->Init(yaml_path_)) {
            LOG(ERROR) << "failed to init localization system";
            return;
        }

        // 初始化 PCL 预处理器
        preprocess_pcl_ = std::make_shared<PointCloudPreprocessPCL>();
        if (!InitPreprocess()) {
            LOG(ERROR) << "failed to init pointcloud preprocess";
            return;
        }

        // 设置初始位姿，默认单位阵
        loc_system_->SetInitPose(SE3());
        LOG(INFO) << "DORA Localization Node (PCL Preprocess) initialized.";
    }

    void Run(void* dora_context) {
        LOG(INFO) << "DORA event loop started.";

        while (!to_exit_process) {
            void* event = dora_next_event(dora_context);
            if (event == NULL) {
                printf("[c node] ERROR: unexpected end of event\n");
                continue;
            }

            enum DoraEventType ty = read_dora_event_type(event);

            if (ty == DoraEventType_Input) {
                char* id_ptr = nullptr;
                size_t id_len = 0;
                read_dora_input_id(event, &id_ptr, &id_len);
                std::string input_id(id_ptr, id_len);

                if (id_ptr == nullptr || id_len == 0) {
                    continue;
                }

                if (input_id == "imu") {
                    HandleImu(event);
                } else if (input_id == "pointcloud") {
                    HandleLidar(event);
                    PublishPose(dora_context, pose_, timestamp_);  // 发布状态
                } else if (input_id == "init_pose") {
                    HandleInitPose(event);
                }
            } else if (ty == DoraEventType_Stop) {
                printf("[c node] received stop event\n");
                to_exit_process = true;
            } else {
                printf("[c node] received unexpected event: %d\n", ty);
            }

            free_dora_event(event);
        }
    }

   private:
    bool InitPreprocess() {
        try {
            auto yaml = YAML::LoadFile(yaml_path_);
            preprocess_pcl_->Blind() = yaml["fasterlio"]["blind"].as<double>();
            preprocess_pcl_->PointFilterNum() = yaml["fasterlio"]["point_filter_num"].as<int>();

            float height_max = yaml["roi"]["height_max"].as<float>();
            float height_min = yaml["roi"]["height_min"].as<float>();
            preprocess_pcl_->SetHeightROI(height_max, height_min);
        } catch (...) {
            LOG(ERROR) << "Exception during preprocess init from YAML.";
            return false;
        }
        return true;
    }

    void HandleImu(void* input_event) {
        char* data_ptr = nullptr;
        size_t data_len = 0;
        read_dora_input_data(input_event, &data_ptr, &data_len);

        if (data_ptr == nullptr || data_len == 0) return;

        std::string_view json_view(data_ptr, data_len);

        try {
            auto data = json::parse(json_view);
            IMUPtr imu = std::make_shared<IMU>();

            double sec = data["header"]["stamp"]["sec"].get<double>();
            double nanosec = data["header"]["stamp"]["nanosec"].get<double>();

            imu->timestamp = sec + (nanosec * 1e-9);
            imu->linear_acceleration = Vec3d(data["linear_acceleration"]["x"], data["linear_acceleration"]["y"],
                                             data["linear_acceleration"]["z"]);
            imu->angular_velocity =
                Vec3d(data["angular_velocity"]["x"], data["angular_velocity"]["y"], data["angular_velocity"]["z"]);

            if (loc_system_) {
                loc_system_->ProcessIMU(imu);
            }
        } catch (const json::exception& e) {
            std::cerr << "IMU JSON Error: " << e.what() << " | Raw: " << json_view << std::endl;
        }
    }

    bool ParsePointCloud(const uint8_t* data, size_t len, double& timestamp, pcl::PointCloud<PointType>::Ptr& cloud) {
        if (len < 16) {
            std::cerr << "[lightning-lm] ParsePointCloud: buffer too small (" << len << " bytes)\n";
            return false;
        }

        std::memcpy(&timestamp, data + 8, sizeof(double));

        const size_t point_bytes = len - 16;
        if (point_bytes % 16 != 0) {
            std::cerr << "[lightning-lm] ParsePointCloud: unexpected payload size\n";
            return false;
        }
        const size_t points_num = point_bytes / 16;

        cloud->clear();
        cloud->reserve(points_num);

        size_t count = 0;
        for (size_t i = 0; i < points_num; ++i) {
            count++;
            if (count % preprocess_pcl_->point_filter_num_ != 0) continue;

            float x, y, z, intensity;
            const size_t base = 16 + i * 16;
            std::memcpy(&x, data + base + 0, sizeof(float));
            std::memcpy(&y, data + base + 4, sizeof(float));
            std::memcpy(&z, data + base + 8, sizeof(float));
            std::memcpy(&intensity, data + base + 12, sizeof(float));

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            PointType added_pt;
            added_pt.x = x;
            added_pt.y = y;
            added_pt.z = z;
            added_pt.intensity = intensity;
            added_pt.time = (double)i / points_num * 100.0f;

            double range_sq = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range_sq < preprocess_pcl_->blind_ * preprocess_pcl_->blind_) continue;

            if (added_pt.z > preprocess_pcl_->height_max_ || added_pt.z < preprocess_pcl_->height_min_) continue;

            cloud->points.push_back(added_pt);
        }

        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->header.stamp = timestamp * 1e9;
        return true;
    }

    void HandleLidar(void* input_event) {
        char* data_ptr = nullptr;
        size_t data_len = 0;
        read_dora_input_data(input_event, &data_ptr, &data_len);

        if (data_ptr == nullptr || data_len == 0) return;

        const uint8_t* data = reinterpret_cast<const uint8_t*>(data_ptr);
        // double timestamp = 0;  // 点云的时间戳
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        if (ParsePointCloud(data, data_len, timestamp_, cloud)) {
            if (loc_system_) {
                loc_system_->ProcessLidar(cloud);

                std::shared_ptr<lightning::loc::Localization> loc_value = loc_system_->GetLoc();
                if (!loc_value || !loc_value->GetLocalizationResult()) {
                    return;  // 或者根据业务逻辑返回错误码
                }

                pose_ = loc_value->GetLocalizationResult()->pose_;

                //  printf
                //  std::shared_ptr<lightning::loc::Localization> loc_value = loc_system_->GetLoc();
                //  Sophus::SE3d pose = loc_value->GetLocalizationResult()->pose_;
                //   std::cout << "loc pose x:" << pose.translation().x << "  y:" << pose.translation().y
                //             << "  z:" << pose.translation().z << std::endl;
            }
        }
    }
    /**
     * @brief 将 Sophus 位姿转换为 JSON 并通过 DORA 发布
     * @param dora_context DORA 上下文指针
     * @return int 0 成功, 1 失败
     */
    int PublishPose(void* dora_context, Sophus::SE3d pose, double timestamp_sec) {
        // 1. 获取位姿数据 (你的原始逻辑)

        // 2. 提取平移和旋转（四元数）
        auto translation = pose.translation();
        auto q = pose.unit_quaternion();

        // 3. 构造符合 ROS 2 geometry_msgs/msg/Pose 结构的 JSON
        json ros_pose;
        ros_pose["position"] = {{"x", translation.x()}, {"y", translation.y()}, {"z", translation.z()}};
        ros_pose["orientation"] = {{"x", q.x()}, {"y", q.y()}, {"z", q.z()}, {"w", q.w()}};

        json ros_pose_stamped;
        int32_t sec = static_cast<int32_t>(timestamp_sec);
        int32_t nanosec = static_cast<int32_t>((timestamp_sec - sec) * 1e9);
        ros_pose_stamped["header"] = {{"frame_id", "map"}, {"stamp", {{"sec", sec}, {"nanosec", nanosec}}}};
        ros_pose_stamped["pose"] = ros_pose;

        // 4. 序列化为 JSON 字符串
        std::string payload = ros_pose_stamped.dump();

        // 5. 调用你提供的 DORA C API 发送数据
        std::string out_id = "pose";  // 必须与 dataflow.yml 中的 output id 一致
        char* output_data = const_cast<char*>(payload.data());
        size_t output_data_len = payload.size();

        int result = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);

        if (result != 0) {
            std::cerr << "failed to send pose output" << std::endl;
            return 1;
        }

        return 0;
    }

    void HandleInitPose(void* input_event) {
        char* data_ptr = nullptr;
        size_t data_len = 0;
        read_dora_input_data(input_event, &data_ptr, &data_len);

        if (data_ptr == nullptr || data_len == 0) return;

        std::string_view json_view(data_ptr, data_len);
        try {
            auto data = json::parse(json_view);
            // 假设输入 JSON 包含 pose [x, y, z, qw, qx, qy, qz]
            Vec3d t(data["translation"]["x"], data["translation"]["y"], data["translation"]["z"]);
            Quatd q(data["rotation"]["w"], data["rotation"]["x"], data["rotation"]["y"], data["rotation"]["z"]);
            SE3 pose(q, t);

            if (loc_system_) {
                loc_system_->SetInitPose(pose);
                LOG(INFO) << "Received init_pose: " << t.transpose();
            }
        } catch (const json::exception& e) {
            std::cerr << "InitPose JSON Error: " << e.what() << std::endl;
        }
    }

    std::string yaml_path_;
    std::shared_ptr<LocSystem> loc_system_;
    std::shared_ptr<PointCloudPreprocessPCL> preprocess_pcl_;
    Sophus::SE3d pose_;
    double timestamp_;
};

int main(int argc, char** argv) {
    std::cout << "lightning localization node for dora" << std::endl;

    auto dora_context = init_dora_context_from_env();

    const char* FLAGS_config = std::getenv("FLAGS_config");
    if (FLAGS_config == nullptr) {
        std::cerr << "ERROR: Environment variable FLAGS_config is not set!" << std::endl;
        return -1;
    }
    std::cout << "FLAGS_config: " << std::string(FLAGS_config) << std::endl;

    DoraLocNode lightning_node(FLAGS_config);
    lightning_node.Run(dora_context);

    return 0;
}
