import dora
import rerun as rr
import numpy as np
import pyarrow as pa
import datetime
import json

# ==========================================
# 全局配置参数 / Global Configurations
# ==========================================
ENABLE_DEBUG = False # True: 开启调试打印 | False: 关闭调试打印
POINT_COLOR = [255, 0, 0]    # RGB 颜色设置, [255, 0, 0] 为纯红色

def debug_print(message):
    """辅助函数：根据全局变量决定是否打印调试信息"""
    if ENABLE_DEBUG:
        print(message)

def main():
    
    # 1. Initialize Rerun
    rr.init("dora_ros2bag_visualizer", spawn=True)

    # === 强制声明 world 是一个 Z 轴朝上的 3D 空间 ===
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    
    # 为 IMU 创建一个坐标系可视化
    rr.log("world/imu", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # 2. 初始化 Dora 节点
    node = dora.Node()
    print("[INFO] Rerun visualizer node started. Listening for 'pointcloud' and 'imu'...")

    for event in node:
        if event["type"] == "INPUT":
            # ---- 处理点云数据 (Binary Payload) ----
            if event["id"] == "pointcloud":
                raw_data = event["value"]
                
                if isinstance(raw_data, (pa.Array, pa.ChunkedArray)):
                    data_bytes = raw_data.to_numpy(zero_copy_only=False).tobytes()
                else:
                    data_bytes = bytes(raw_data) 

                if len(data_bytes) < 16:
                    continue

                try:
                    # 解析头部
                    seq = np.frombuffer(data_bytes, dtype=np.uint32, count=1, offset=0)[0]
                    timestamp_us = np.frombuffer(data_bytes, dtype=np.uint64, count=1, offset=8)[0]

                    # 解析坐标 (x, y, z, i)
                    points_raw = np.frombuffer(data_bytes, dtype=np.float32, offset=16)
                    positions = points_raw.reshape((-1, 4))[:, :3]

                    # 数据清洗
                    valid_mask = np.all(np.isfinite(positions), axis=1) & np.any(positions != 0.0, axis=1)
                    positions = positions[valid_mask]

                    if len(positions) == 0:
                        continue

                    # Rerun 可视化
                    rr.set_time_nanos("timestamp", int(timestamp_us * 1000))
                    rr.log("world/pointcloud", rr.Points3D(positions=positions, colors=POINT_COLOR))
                    
                    debug_print(f"[INFO] pointcloud frame {seq} logged")
                    
                except Exception as e:
                    print(f"[ERROR] PointCloud parsing error: {e}")

            # ---- 处理 IMU 数据 (JSON String) ----
            elif event["id"] == "imu":
                raw_data = event["value"]
                try:
                    # Dora 发送的字符串通常包装在 Arrow 数组中
                    if isinstance(raw_data, (pa.Array, pa.ChunkedArray)):
                        json_str = raw_data[0].as_py()
                    else:
                        json_str = str(raw_data)
                    
                    imu_msg = json.loads(json_str)
                    
                    # 提取时间戳
                    stamp = imu_msg["header"]["stamp"]
                    timestamp_us = int(stamp["sec"] * 1e6 + stamp["nanosec"] / 1e3)
                    
                    # 提取姿态 (Quaternion: x, y, z, w)
                    o = imu_msg["orientation"]
                    orientation = [o["x"], o["y"], o["z"], o["w"]]
                    
                    # 提取线加速度
                    a = imu_msg["linear_acceleration"]
                    accel = [a["x"], a["y"], a["z"]]
                    
                    # 提取角速度
                    v = imu_msg["angular_velocity"]
                    gyro = [v["x"], v["y"], v["z"]]

                    # Rerun 设置时间
                    rr.set_time_nanos("timestamp", int(timestamp_us * 1000))

                    # 1. 可视化 IMU 姿态 (通过旋转坐标系)
                    # 确保 orientation 是 [x, y, z, w] 格式
                    rr.log("world/imu", rr.Transform3D(
                        rotation=rr.Quaternion(xyzw=orientation)
                    ))

                    # 2. 可视化加速度矢量 (绿色箭头)
                    # 注意：有些版本的 Rerun 要求 origins 和 vectors 必须是列表的列表
                    rr.log("world/imu/acceleration", rr.Arrows3D(
                        origins=[[0, 0, 0]],
                        vectors=[accel],
                        colors=[[0, 255, 0]]
                    ))
                    
                    # 3. 记录角速度标量 (解决 Scalar 属性报错问题)
                    # 尝试使用旧版兼容的 log_scalar 或检测属性是否存在
                    try:
                        if hasattr(rr, "Scalar"):
                            rr.log("world/imu/gyro/x", rr.Scalar(gyro[0]))
                            rr.log("world/imu/gyro/y", rr.Scalar(gyro[1]))
                            rr.log("world/imu/gyro/z", rr.Scalar(gyro[2]))
                        else:
                            # 兼容旧版 SDK
                            rr.log_scalar("world/imu/gyro/x", gyro[0])
                            rr.log_scalar("world/imu/gyro/y", gyro[1])
                            rr.log_scalar("world/imu/gyro/z", gyro[2])
                    except Exception:
                        pass # 如果标量日志依然报错，则跳过，优先保证主画面显示

                except Exception as e:
                    print(f"[ERROR] IMU JSON parsing error: {e}")

        elif event["type"] == "STOP":
            print("[INFO] Received STOP event. Exiting...")
            break

if __name__ == "__main__":
    main()
