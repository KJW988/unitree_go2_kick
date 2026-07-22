#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

from dribblebot.perception.lidar_ball_detector import Go2LidarBallDetector


def pointcloud2_to_xyz(msg):
    fields = {field.name: field for field in msg.fields}

    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError(f"missing PointCloud2 field: {name}")
        if fields[name].datatype != PointField.FLOAT32:
            raise ValueError(f"{name} is not FLOAT32")

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


class BallDetectorNode(Node):
    def __init__(self):
        super().__init__("go2_lidar_ball_detector")
        self.detector = Go2LidarBallDetector(ball_radius=0.11)
        self.last_ball_print = 0.0
        self.last_no_ball_print = 0.0

        self.subscription = self.create_subscription(
            PointCloud2,
            "/utlidar/cloud",
            self.callback,
            qos_profile_sensor_data,
        )

        print("Listening on /utlidar/cloud", flush=True)

    def callback(self, msg):
        started = time.perf_counter()

        try:
            points = pointcloud2_to_xyz(msg)
            center = self.detector.detect_ball_3d(points)
        except Exception as error:
            self.get_logger().error(str(error))
            return

        latency_ms = (time.perf_counter() - started) * 1000.0
        now = time.monotonic()

        if center is None:
            if now - self.last_no_ball_print >= 1.0:
                print(
                    f"[NO BALL] points={len(points)} "
                    f"latency={latency_ms:.1f}ms",
                    flush=True,
                )
                self.last_no_ball_print = now
            return

        if now - self.last_ball_print >= 0.2:
            print(
                f"[BALL] x={center[0]:.3f}m "
                f"y={center[1]:.3f}m "
                f"z={center[2]:.3f}m "
                f"frame={msg.header.frame_id} "
                f"points={len(points)} "
                f"latency={latency_ms:.1f}ms",
                flush=True,
            )
            self.last_ball_print = now


def main():
    rclpy.init()
    node = BallDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
