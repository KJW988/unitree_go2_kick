#!/usr/bin/env python3
"""ROS2 `/utlidar/robot_odom`을 localhost JSON으로 read-only bridge한다.

이 bridge는 Unitree LiDAR odometry 구독만 수행한다. Go2 DDS, LowCmd,
SportClient, MotionSwitcher를 생성하지 않는다. D435i process가 공/Tag를 보는 동안
이 odom은 camera visibility가 끝난 마지막 FR docking 구간의 base displacement를
추적하는 데 사용한다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http import server
from typing import Any


class OdomStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {"ready": False}

    def update(self, message: Any) -> None:
        orientation = message.pose.pose.orientation
        siny = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        position = message.pose.pose.position
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        with self._lock:
            self._state = {
                "ready": True,
                "receipt_monotonic_s": time.monotonic(),
                "frame_id": str(message.header.frame_id),
                "child_frame_id": str(message.child_frame_id),
                "position_xyz_m": [float(position.x), float(position.y), float(position.z)],
                "yaw_rad": float(math.atan2(siny, cosy)),
                "linear_xyz_mps": [float(linear.x), float(linear.y), float(linear.z)],
                "angular_xyz_rps": [float(angular.x), float(angular.y), float(angular.z)],
            }

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._state)


class Handler(server.BaseHTTPRequestHandler):
    server: Any

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        if self.path not in ("/", "/state.json"):
            self.send_error(404)
            return
        body = json.dumps(self.server.odom_store.snapshot(), ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/utlidar/robot_odom")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    try:
        import rclpy
        from nav_msgs.msg import Odometry
    except ImportError as error:
        raise RuntimeError("ROS2 Foxy environment의 rclpy/nav_msgs가 필요합니다: {}".format(error)) from error
    store = OdomStore()
    rclpy.init()
    node = rclpy.create_node("go2_kick_lidar_odom_bridge")
    node.create_subscription(Odometry, args.topic, store.update, 10)
    httpd = server.ThreadingHTTPServer((args.host, args.port), Handler)
    httpd.odom_store = store
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print("UTLIDAR_ODOM_BRIDGE_READY topic={} url=http://{}:{}/state.json".format(
        args.topic, args.host, args.port), flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        httpd.shutdown()
        httpd.server_close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
