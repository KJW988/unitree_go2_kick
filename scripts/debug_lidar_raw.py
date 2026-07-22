#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

from dribblebot.perception.lidar_ball_detector import Go2LidarBallDetector


def pointcloud2_to_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    dtype = ">f4" if msg.is_bigendian else "<f4"
    rows = []
    for row_index in range(msg.height):
        start = row_index * msg.row_step
        end = start + msg.width * msg.point_step
        row_buffer = memoryview(msg.data)[start:end]
        xyz = np.column_stack([
            np.ndarray(
                shape=(msg.width,),
                dtype=dtype,
                buffer=row_buffer,
                offset=fields[name].offset,
                strides=(msg.point_step,),
            )
            for name in ("x", "y", "z")
        ])
        rows.append(xyz)
    if not rows:
        return np.empty((0, 3), dtype=np.float32)
    xyz = np.concatenate(rows, axis=0)
    return xyz[np.isfinite(xyz).all(axis=1)]


class RawLidarDebugNode(Node):
    def __init__(self):
        super().__init__("debug_lidar_raw")
        self.detector = Go2LidarBallDetector()
        self.sub = self.create_subscription(
            PointCloud2, "/utlidar/cloud", self.cb, qos_profile_sensor_data
        )
        print("=== LiDAR Raw & Transformed PointCloud Debugger Started ===", flush=True)

    def cb(self, msg):
        raw_pts = pointcloud2_to_xyz(msg)
        if len(raw_pts) == 0:
            print("[RAW DEBUG] Empty Point Cloud received!", flush=True)
            return

        base_pts = self.detector.transform_utlidar_to_base(raw_pts)
        
        # 전방 0.2m~2.0m 사이의 모든 점군 Z 범위 측정
        front_mask = (base_pts[:, 0] >= 0.2) & (base_pts[:, 0] <= 2.0) & (abs(base_pts[:, 1]) <= 0.5)
        front_pts = base_pts[front_mask]

        if len(front_pts) > 0:
            z_min = np.min(front_pts[:, 2])
            z_max = np.max(front_pts[:, 2])
            z_mean = np.mean(front_pts[:, 2])
            print(
                f"[RAW DEBUG] Total={len(raw_pts)} | FrontPts={len(front_pts)} | "
                f"Front Z-range: min={z_min:.3f}m, max={z_max:.3f}m, mean={z_mean:.3f}m",
                flush=True,
            )
        else:
            print(f"[RAW DEBUG] Total={len(raw_pts)} | FrontPts=0 (Check transform!)", flush=True)


def main():
    rclpy.init()
    node = RawLidarDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
