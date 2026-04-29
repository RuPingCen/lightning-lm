import json
import numpy as np

def convert_pointcloud2_to_dora_payload(msg, seq):
    """
    将 ROS2 的 PointCloud2 消息转换为 Dora 可接收的二进制 Payload。
    内存布局 (更新后):
      - 0~3: uint32 seq
      - 4~7: padding
      - 8~15: float64 (double) timestamp_sec  <-- 修改处
      - 16~...: point (float32 x, y, z, intensity)
    """
    # 1. 计算 double 类型的时间戳（秒）
    # 小数点前是秒，小数点后是纳秒部分
    timestamp_sec = float(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
    
    num_points = msg.width * msg.height
    
    # 提取字段偏移量
    x_offset, y_offset, z_offset, i_offset = 0, 4, 8, None
    i_datatype = None
    for field in msg.fields:
        if field.name == 'x': x_offset = field.offset
        elif field.name == 'y': y_offset = field.offset
        elif field.name == 'z': z_offset = field.offset
        elif field.name == 'intensity': 
            i_offset = field.offset
            i_datatype = field.datatype

    raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(num_points, msg.point_step)
    
    # 提取坐标与强度 (保持不变)
    x = raw_data[:, x_offset:x_offset+4].copy().view(np.float32).flatten()
    y = raw_data[:, y_offset:y_offset+4].copy().view(np.float32).flatten()
    z = raw_data[:, z_offset:z_offset+4].copy().view(np.float32).flatten()
    
    if i_offset is not None:
        if i_datatype == 7: # FLOAT32
            i = raw_data[:, i_offset:i_offset+4].copy().view(np.float32).flatten()
        elif i_datatype == 2: # UINT8
            i = raw_data[:, i_offset:i_offset+1].copy().view(np.uint8).flatten().astype(np.float32)
        elif i_datatype == 4: # UINT16
            i = raw_data[:, i_offset:i_offset+2].copy().view(np.uint16).flatten().astype(np.float32)
        else:
            i = np.zeros(num_points, dtype=np.float32)
    else:
        i = np.zeros(num_points, dtype=np.float32)

    # 2. 构造头部 (16 字节)
    header = np.zeros(16, dtype=np.uint8)
    # 写入 seq (0~3 字节)
    header[0:4] = np.array([seq], dtype=np.uint32).view(np.uint8)
    
    # 写入 timestamp (8~15 字节)
    # 使用 np.float64 对应 C++ 中的 double 类型
    header[8:16] = np.array([timestamp_sec], dtype=np.float64).view(np.uint8)
    
    # 3. 构造点云数据内容 (保持不变)
    points_payload = np.zeros((num_points, 4), dtype=np.float32)
    points_payload[:, 0], points_payload[:, 1], points_payload[:, 2], points_payload[:, 3] = x, y, z, i
    
    return header.tobytes() + points_payload.tobytes()


def convert_imu_to_json_payload(msg, seq):
    """
    将 ROS2 的 Imu 消息转换为 JSON 字符串。
    """
    imu_dict = {
        "header": {
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "frame_id": msg.header.frame_id, 
            "seq": seq
        },
        "orientation": {"x": msg.orientation.x, "y": msg.orientation.y, "z": msg.orientation.z, "w": msg.orientation.w},
        "orientation_covariance": list(msg.orientation_covariance),
        "angular_velocity": {"x": msg.angular_velocity.x, "y": msg.angular_velocity.y, "z": msg.angular_velocity.z},
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration": {"x": msg.linear_acceleration.x, "y": msg.linear_acceleration.y, "z": msg.linear_acceleration.z},
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance)
    }
    return json.dumps(imu_dict)


def convert_image_to_dora_payload(msg):
    """
    将 ROS2 的 Image 消息转换为 Dora 可接收的格式 (参考 webcam.py)。
    webcam.py 中图像以平铺的 uint8 数组形式发送。
    """
    # 直接将 ROS2 消息中的二进制数据转换为 uint8 数组
    # 注意：webcam.py 发送的是 pa.array(frame.ravel())
    # 这里返回 numpy 数组，在发送端可以使用 pa.array() 包装
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    return frame


def convert_navsatfix_to_json_payload(msg, seq):
    """
    将 ROS2 的 NavSatFix 消息转换为 JSON 字符串。
    """
    nav_dict = {
        "header": {
            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
            "frame_id": msg.header.frame_id,
            "seq": seq
        },
        "status": {
            "status": msg.status.status,
            "satellites_used": msg.status.satellites_used
        },
        "latitude": msg.latitude,
        "longitude": msg.longitude,
        "altitude": msg.altitude,
        "position_covariance": list(msg.position_covariance),
        "position_covariance_type": msg.position_covariance_type
    }
    return json.dumps(nav_dict)
