extern "C" {
#include "node_api.h"
// #include "operator_api.h"
// #include "operator_types.h"
}

#include <yaml-cpp/yaml.h>
#include <atomic>  // 仅用于安全退出，不破坏原有结构
#include <cstring>
#include <iostream>
#include <thread>  // 仅用于安全退出，不破坏原有结构
#include <vector>

#include "common/eigen_types.h"  // IMUPtr imu type
#include "common/point_def.h"
#include "core/system/slam.h"
#include "utils/timer.h"
// #include "wrapper/ros_utils.h"

// IMU 仍然可能需要 ROS 格式，除非用户也要求去掉
// #include <sensor_msgs/msg/imu.hpp>

// DEFINE_string(config, "./config/default.yaml", "配置文件");

#include <iostream>
#include <nlohmann/json.hpp>
#include <string>

using json = nlohmann::json;

using namespace lightning;

std::atomic<bool> to_exit_process{false};
/**
 * 模仿 PointCloudPreprocess 的 PCL -> PCL 预处理器
 */
class PointCloudPreprocessPCL {
   public:
    PointCloudPreprocessPCL() = default;

    /// 模仿原 PointCloudPreprocess 的参数名称
    double& Blind() { return blind_; }
    int& PointFilterNum() { return point_filter_num_; }
    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }
    // 与 PointCloudPreprocess 保持一致的参数名
    int point_filter_num_ = 1;
    double blind_ = 0.01;
    float height_max_ = 1.0;
    float height_min_ = -1.0;
};

class DoraSlamNode {
   public:
    DoraSlamNode(const std::string& yaml_path) : yaml_path_(yaml_path) {
        // 初始化 SLAM 系统
        SlamSystem::Options options;
        options.online_mode_ = true;
        slam_ = std::make_shared<SlamSystem>(options);
        if (!slam_->Init(yaml_path_)) {
            LOG(ERROR) << "failed to init slam";
            return;
        }

        // 初始化 PCL 预处理器
        preprocess_pcl_ = std::make_shared<PointCloudPreprocessPCL>();
        if (!InitPreprocess()) {
            LOG(ERROR) << "failed to init pointcloud preprocess";
            return;
        }

        slam_->StartSLAM("new_map");
        LOG(INFO) << "DORA SLAM Node (PCL Preprocess) initialized and started.";
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
                    // printf("[lightning-lm node] received imu event\n");
                    HandleImu(event);
                } else if (input_id == "pointcloud") {
                    // printf("[lightning-lm node] received pointcloud event\n");
                    HandleLidar(event);
                } else if (input_id == "save_map") {
                    printf("[lightning-lm node] received save_map event\n");
                    HandleSaveMap();
                }
            } else if (ty == DoraEventType_Stop) {
                printf("[c node] received stop event\n");
                to_exit_process = true;
            } else {
                printf("[c node] received unexpected event: %d\n", ty);
            }

            free_dora_event(event);
        }

        // 退出时保存地图
        HandleSaveMap();
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

        // 建议：直接构造 string_view 可以减少一次拷贝（C++17）
        std::string_view json_view(data_ptr, data_len);

        try {
            auto data = json::parse(json_view);

            IMUPtr imu = std::make_shared<IMU>();

            // std::cout << "sec: " << data["header"]["stamp"]["sec"] << "  nanosec:" <<
            // data["header"]["stamp"]["nanosec"]
            //           << std::endl;

            double sec = data["header"]["stamp"]["sec"].get<double>();
            double nanosec = data["header"]["stamp"]["nanosec"].get<double>();

            imu->timestamp = sec + (nanosec * 1e-9);

            imu->linear_acceleration = Vec3d(data["linear_acceleration"]["x"], data["linear_acceleration"]["y"],
                                             data["linear_acceleration"]["z"]);

            imu->angular_velocity =
                Vec3d(data["angular_velocity"]["x"], data["angular_velocity"]["y"], data["angular_velocity"]["z"]);

            if (slam_) {
                slam_->ProcessIMU(imu);
            }

            // std::printf("[IMU] ts: %.6f, acc_z: %.3f\n", imu->timestamp, imu->linear_acceleration.z());
        } catch (const json::exception& e) {
            std::cerr << "IMU JSON Error: " << e.what() << " | Raw: " << json_view << std::endl;
        }
    }

    // ---------------------------------------------------------------------------
    // ParsePointCloud
    // ---------------------------------------------------------------------------
    bool ParsePointCloud(const uint8_t* data, size_t len, double& timestamp, pcl::PointCloud<PointType>::Ptr& cloud) {
        // Minimum: 16-byte header
        if (len < 16) {
            std::cerr << "[" << "lightning-lm" << "] ParsePointCloud: buffer too small (" << len << " bytes)\n";
            return false;
        }

        // Parse header
        // uint32_t seq;
        // std::memcpy(&seq, data + 0, sizeof(uint32_t));
        std::memcpy(&timestamp, data + 8, sizeof(double));

        const size_t point_bytes = len - 16;
        if (point_bytes % 16 != 0) {
            std::cerr << "[" << "lightning-lm" << "] ParsePointCloud: unexpected payload size\n";
            return false;
        }
        const size_t points_num = point_bytes / 16;

        cloud->clear();
        cloud->reserve(points_num);

        size_t count = 0;

        for (size_t i = 0; i < points_num; ++i) {
            count++;
            // 1. PointFilterNum 采样过滤
            if (count % preprocess_pcl_->point_filter_num_ != 0) continue;

            float x, y, z, intensity;
            const size_t base = 16 + i * 16;
            std::memcpy(&x, data + base + 0, sizeof(float));
            std::memcpy(&y, data + base + 4, sizeof(float));
            std::memcpy(&z, data + base + 8, sizeof(float));
            std::memcpy(&intensity, data + base + 12, sizeof(float));

            // Skip invalid (NaN / Inf) points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            PointType added_pt;
            added_pt.x = x;
            added_pt.y = y;
            added_pt.z = z;
            added_pt.intensity = intensity;
            // added_pt.time = timestamp * 1e3;  //  / 1e6;  // curvature unit: ms
            // 模拟 RoboSense 的行为：将 0~100ms 的偏移量分给各个点
            added_pt.time = (double)i / points_num * 100.0f;

            // 2. Blind 盲区过滤
            double range_sq = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
            if (range_sq < preprocess_pcl_->blind_ * preprocess_pcl_->blind_) continue;

            // 3. Height ROI 高度过滤
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

        if (data_ptr == nullptr || data_len == 0) {
            std::cerr << "Warn: Received input event but ID pointer is null" << std::endl;
            return;
        }
        const uint8_t* data = reinterpret_cast<const uint8_t*>(data_ptr);

        double timestamp = 0;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        if (ParsePointCloud(data, data_len, timestamp, cloud)) {
            // std::printf("[pointcould] ts: %.6f, points: %d\n", timestamp, cloud->points.size());
        }

        // Timer::Evaluate(
        //     [&]()
        //     {
        //         CloudPtr cloud_raw(new PointCloudType());

        //         // 1. 映射 DORA 数据到原始 PCL 点云
        //         size_t num_points = input.data.size() / sizeof(PointType);
        //         if (num_points == 0) return;
        //         cloud_raw->points.resize(num_points);
        //         memcpy(cloud_raw->points.data(), input.data.data(), input.data.size());

        //         // TODO: 设置时间戳，例如从 metadata 中获取
        //         // cloud_raw->header.stamp = ...;

        //         // 2. 预处理 (过滤)
        //         CloudPtr cloud_filtered(new PointCloudType());
        //         preprocess_pcl_->Process(cloud_raw, cloud_filtered);

        // 3. 送入 SLAM
        slam_->ProcessLidar(cloud);
        //     },
        //     "Proc Lidar DORA (PCL Preprocess)", true);
    }

    void HandleSaveMap(void) {
        std::string map_id = "map";
        std::string save_path = "./data/" + map_id + "/";
        slam_->SaveMap(save_path);
    }

    std::string yaml_path_;
    std::shared_ptr<SlamSystem> slam_;
    std::shared_ptr<PointCloudPreprocessPCL> preprocess_pcl_;
};

int main(int argc, char** argv) {
    //
    std::cout << "lightning node for dora " << std::endl;

    auto dora_context = init_dora_context_from_env();

    const char* FLAGS_config = std::getenv("FLAGS_config");
    if (FLAGS_config == nullptr) {
        std::cerr << "ERROR: Environment variable FLAGS_config is not set!" << std::endl;
        return -1;
    }
    std::cout << "FLAGS_config: " << std::string(FLAGS_config) << std::endl;

    DoraSlamNode lightning_node(FLAGS_config);
    lightning_node.Run(dora_context);

    std::cout << "exit rslidar driver ..." << std::endl;
    return 0;
}
