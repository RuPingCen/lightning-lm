import dora
import rerun as rr
import numpy as np
import pyarrow as pa
import json
import datetime

# ==========================================
# 配置参数
# ==========================================
POINT_COLOR = [255, 0, 0]  # 点云颜色：红色
ACCEL_COLOR = [0, 255, 0]  # 加速度箭头：绿色

def main():
    # 1. 初始化 Rerun
    rr.init("bag_data_visualizer", spawn=True)
    
    # 声明坐标系：Z 轴朝上
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/imu", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # 2. 初始化 Dora 节点
    node = dora.Node()
    print("[INFO] Visualizer started. Listening for pointcloud, imu, and gps...")

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            raw_data = event["value"]

            # ---- 处理点云 (Binary) ----
            if input_id == "pointcloud":
                if isinstance(raw_data, (pa.Array, pa.ChunkedArray)):
                    data_bytes = raw_data.to_numpy(zero_copy_only=False).tobytes()
                else:
                    data_bytes = bytes(raw_data)
                
                if len(data_bytes) < 16: continue
                
                # 解析头和坐标
                timestamp_us = np.frombuffer(data_bytes, dtype=np.uint64, count=1, offset=8)[0]
                points_raw = np.frombuffer(data_bytes, dtype=np.float32, offset=16)
                positions = points_raw.reshape((-1, 4))[:, :3]

                # 清洗无效点
                mask = np.all(np.isfinite(positions), axis=1) & np.any(positions != 0.0, axis=1)
                positions = positions[mask]

                # 可视化
                rr.set_time_nanos("timestamp", int(timestamp_us * 1000))
                rr.log("world/pointcloud", rr.Points3D(positions=positions, colors=POINT_COLOR))

            # ---- 处理 IMU (JSON) ----
            elif input_id == "imu":
                if isinstance(raw_data, (pa.Array, pa.ChunkedArray)):
                    json_str = raw_data[0].as_py()
                else:
                    json_str = str(raw_data)
                
                try:
                    imu_msg = json.loads(json_str)
                    stamp = imu_msg["header"]["stamp"]
                    ts_ns = int(stamp["sec"] * 1e9 + stamp["nanosec"])
                    
                    accel = [imu_msg["linear_acceleration"]["x"], 
                             imu_msg["linear_acceleration"]["y"], 
                             imu_msg["linear_acceleration"]["z"]]
                    
                    gyro = imu_msg["angular_velocity"]
                    
                    # Rerun 时间对齐
                    rr.set_time_nanos("timestamp", ts_ns)

                    # A. 加速度矢量箭头
                    rr.log("world/imu/acceleration", rr.Arrows3D(
                        origins=[[0, 0, 0]],
                        vectors=[accel],
                        colors=[ACCEL_COLOR]
                    ))

                    # B. 陀螺仪三轴曲线 (Scalar)
                    # 自动兼容新旧版本 Rerun API
                    def log_val(path, val):
                        if hasattr(rr, "Scalar"):
                            rr.log(path, rr.Scalar(val))
                        else:
                            rr.log_scalar(path, val)

                    log_val("world/imu/gyro/x", gyro["x"])
                    log_val("world/imu/gyro/y", gyro["y"])
                    log_val("world/imu/gyro/z", gyro["z"])

                except Exception as e:
                    print(f"[ERROR] IMU parsing error: {e}")

            # ---- 处理 GPS/RTK (JSON) ----
            elif input_id == "gps":
                if isinstance(raw_data, (pa.Array, pa.ChunkedArray)):
                    json_str = raw_data[0].as_py()
                else:
                    json_str = str(raw_data)
                
                try:
                    gps_msg = json.loads(json_str)
                    status = gps_msg["status"]["status"]
                    sats = gps_msg["status"].get("satellites_used", 0)
                    lat, lon, alt = gps_msg["latitude"], gps_msg["longitude"], gps_msg["altitude"]

                    # 格式化打印到终端
                    status_map = {-1: "No Fix", 0: "Single", 1: "SBAS", 2: "RTK/GBAS"}
                    status_str = status_map.get(status, f"Unknown({status})")
                    
                    print(f"[GPS] {datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]} | "
                          f"Status: {status_str:<8} | Sats: {sats:>2} | "
                          f"Lat: {lat:.8f}, Lon: {lon:.8f}, Alt: {alt:.2f}m")

                except Exception as e:
                    print(f"[ERROR] GPS parsing error: {e}")

        elif event["type"] == "STOP":
            break

if __name__ == "__main__":
    main()
