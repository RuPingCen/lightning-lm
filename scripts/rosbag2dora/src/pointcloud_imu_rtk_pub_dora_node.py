import os
import numpy as np
import pyarrow as pa
from rosbags.rosbag2 import Reader
import dora

# 从自定义模块导入工具
from rosbag_reader import read_bag_metadata, decode_msg
from msg_conversion import (
    convert_pointcloud2_to_dora_payload, 
    convert_imu_to_json_payload,
    convert_navsatfix_to_json_payload
)

def parse_ros2_bag(bag_info_obj):
    """
    解析 ROS2 bag 内容并将提取的消息存入 RosbagInfo 对象
    """
    bag_path = bag_info_obj.path
    if os.path.isfile(bag_path) and bag_path.endswith('.db3'):
        bag_path = os.path.dirname(bag_path)
    
    try:
        with Reader(bag_path) as reader:
            pc_seq, imu_seq, gps_seq = 0, 0, 0
            imu_timestamps, pc_timestamps, gps_timestamps = [], [], []

            for connection, timestamp, rawdata in reader.messages():
                msg_type = connection.msgtype
                
                # 处理 IMU 消息
                if msg_type == 'sensor_msgs/msg/Imu':
                    msg = decode_msg(rawdata, msg_type)
                    json_str = convert_imu_to_json_payload(msg, imu_seq)
                    bag_info_obj.messages.append({'bag_timestamp': timestamp, 'type': 'imu', 'payload': json_str})
                    imu_timestamps.append(timestamp)
                    imu_seq += 1
                    
                # 处理点云消息
                elif msg_type == 'sensor_msgs/msg/PointCloud2':
                    msg = decode_msg(rawdata, msg_type)
                    payload_bytes = convert_pointcloud2_to_dora_payload(msg, pc_seq)
                    bag_info_obj.messages.append({'bag_timestamp': timestamp, 'type': 'pointcloud', 'payload': payload_bytes})
                    pc_timestamps.append(timestamp)
                    pc_seq += 1

                # 处理 GPS (NavSatFix) 消息
                elif msg_type == 'sensor_msgs/msg/NavSatFix':
                    msg = decode_msg(rawdata, msg_type)
                    json_str = convert_navsatfix_to_json_payload(msg, gps_seq)
                    bag_info_obj.messages.append({'bag_timestamp': timestamp, 'type': 'gps', 'payload': json_str})
                    gps_timestamps.append(timestamp)
                    gps_seq += 1

            if not bag_info_obj.messages:
                return False

            # 首帧对齐逻辑
            imu_start = imu_timestamps[0] if imu_timestamps else None
            pc_start = pc_timestamps[0] if pc_timestamps else None
            gps_start = gps_timestamps[0] if gps_timestamps else None
            
            for m in bag_info_obj.messages:
                if m['type'] == 'imu':
                    m['rel_time'] = m['bag_timestamp'] - imu_start
                elif m['type'] == 'pointcloud':
                    m['rel_time'] = m['bag_timestamp'] - pc_start
                elif m['type'] == 'gps':
                    m['rel_time'] = m['bag_timestamp'] - gps_start
            
            bag_info_obj.messages.sort(key=lambda x: x['rel_time'])
            return True
    except Exception as e:
        print(f"Error parsing bag data: {e}")
        return False

def main():
    bag_path = os.environ.get("BAG_PATH")
    if not bag_path:
        print("[ERROR] BAG_PATH environment variable is missing.")
        return

    # 1. 读取元数据
    bag_info = read_bag_metadata(bag_path)

    # 2. 提取并同步消息
    if not parse_ros2_bag(bag_info):
        print("[ERROR] No valid data extracted from bag.")
        return

    # 3. 打印汇总概览
    bag_info.summary()

    # 4. 初始化 Dora 节点与回放
    total_messages = len(bag_info.messages)
    msg_index, virtual_elapsed_ns = 0, 0
    TICK_INCREMENT_NS = 5_000_000 # 200Hz
    
    node = dora.Node()

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            if msg_index >= total_messages:
                print("[INFO] Reached end of bag playback.")
                break

            # 虚拟时间同步回放
            while msg_index < total_messages and bag_info.messages[msg_index]['rel_time'] <= virtual_elapsed_ns:
                msg = bag_info.messages[msg_index]
                if msg['type'] == 'pointcloud':
                    # 点云作为 uint8 数组发送
                    node.send_output(msg['type'], pa.array(np.frombuffer(msg['payload'], dtype=np.uint8)))
                else:
                    # IMU 或 GPS JSON 作为字符串发送
                    node.send_output(msg['type'], pa.array([msg['payload']]))
                msg_index += 1

            virtual_elapsed_ns += TICK_INCREMENT_NS
            
        elif event["type"] == "STOP":
            break

if __name__ == "__main__":
    main()
