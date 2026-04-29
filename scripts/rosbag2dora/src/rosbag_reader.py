import os
import yaml
import datetime
from rosbags.rosbag2 import Reader

# ---- 兼容新老版本 rosbags 的反序列化处理 ----
try:
    from rosbags.serde import deserialize_cdr
    def decode_msg(rawdata, msgtype):
        return deserialize_cdr(rawdata, msgtype)
except ImportError:
    from rosbags.typesys import Stores, get_typestore
    # 默认使用 ROS2_HUMBLE
    typestore = get_typestore(Stores.ROS2_HUMBLE) 
    def decode_msg(rawdata, msgtype):
        return typestore.deserialize_cdr(rawdata, msgtype)
# ---------------------------------------------

class RosbagInfo:
    """
    存储并管理 Rosbag 的完整信息
    """
    def __init__(self, path):
        self.path = path
        self.storage_id = "unknown"
        self.duration_ns = 0
        self.start_ts_ns = 0
        self.total_msg_count = 0
        self.topic_details = {}  # {topic_name: {'type': type, 'count': count}}
        self.messages = []       # 提取出的用于回放的消息列表

    def summary(self):
        """打印汇总信息"""
        start_date = datetime.datetime.fromtimestamp(self.start_ts_ns / 1e9).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        print("\n" + "="*30 + " Rosbag Summary " + "="*30)
        print(f"  Path:          {self.path}")
        print(f"  Storage ID:    {self.storage_id}")
        print(f"  Start Time:    {start_date}")
        print(f"  Duration:      {self.duration_ns / 1e9:.2f}s")
        print(f"  Total Msgs:    {self.total_msg_count}")
        print("  Topics Details:")
        for name, info in self.topic_details.items():
            print(f"    - {name:<25} | Type: {info['type']:<30} | Count: {info['count']}")
        
        target_count = len(self.messages)
        print(f"\n  Extracted Target Messages: {target_count} (IMU & PointCloud)")
        print("="*76 + "\n")

def read_bag_metadata(bag_path):
    """
    从 metadata.yaml 读取并返回 RosbagInfo 实例
    """
    info = RosbagInfo(bag_path)
    metadata_file = os.path.join(bag_path, "metadata.yaml")
    
    if not os.path.exists(metadata_file):
        print(f"[WARNING] metadata.yaml not found at {bag_path}")
        return info

    try:
        with open(metadata_file, 'r') as f:
            content = yaml.safe_load(f)
            
        bag_data = content.get('rosbag2_bagfile_information', {})
        info.storage_id = bag_data.get('storage_identifier', 'unknown')
        info.duration_ns = bag_data.get('duration', {}).get('nanoseconds', 0)
        info.start_ts_ns = bag_data.get('starting_time', {}).get('nanoseconds_since_epoch', 0)
        info.total_msg_count = bag_data.get('message_count', 0)
        
        topics = bag_data.get('topics_with_message_count', [])
        for t in topics:
            t_meta = t.get('topic_metadata', {})
            name = t_meta.get('name', 'N/A')
            info.topic_details[name] = {
                'type': t_meta.get('type', 'N/A'),
                'count': t.get('message_count', 0)
            }
    except Exception as e:
        print(f"[ERROR] Failed to parse metadata.yaml: {e}")
    
    return info
