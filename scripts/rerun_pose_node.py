import dora
from dora import Node
import rerun as rr
import json
import numpy as np
import sys
import time  # <--- 1. 添加这个导入

def main():
    rr.init("dora_rerun_visualizer", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    
    node = Node()
    trajectory_points = []
    receive_count = 0

    print("Rerun node started and waiting for 'pose'...")

    for event in node:
        if event["type"] == "INPUT" and event["id"] == "pose":
            receive_count += 1
            try:
                # 2. 这里的 pyarrow 转换已经成功了
                raw_value = event["value"]
                msg_bytes = raw_value.to_numpy().tobytes()
                msg_content = msg_bytes.decode("utf-8")
                data = json.loads(msg_content)
                
                # 提取数据
                pos = data["pose"]["position"]
                ori = data["pose"]["orientation"]
                translation = [pos["x"], pos["y"], pos["z"]]
                rotation_q = [ori["x"], ori["y"], ori["z"], ori["w"]]

                # 3. 修正时间轴设置
                # 使用系统当前时间
                rr.set_time_seconds("log_time", time.time()) 
                # 同时保留你的模拟时间轴，方便对比
                if "header" in data and "stamp" in data["header"]:
                    stamp = data["header"]["stamp"]
                    sim_ts = stamp["sec"] + stamp["nanosec"] * 1e-9
                    rr.set_time_seconds("sim_time", sim_ts)

                # 4. Log 数据
                rr.log("world/robot_pose", rr.Transform3D(
                    translation=translation,
                    rotation=rr.Quaternion(xyzw=rotation_q)
                ))

                trajectory_points.append(translation)
                rr.log("world/trajectory", rr.LineStrips3D([trajectory_points], colors=[[0, 255, 0]]))
                
                if receive_count % 10 == 1:
                    print(f"[SUCCESS] Recorded pose #{receive_count} at {translation}")

            except Exception as e:
                print(f"[ERROR] {e}", file=sys.stderr)

if __name__ == "__main__":
    main()