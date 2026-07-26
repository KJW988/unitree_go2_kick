#!/usr/bin/env python3
"""ROS2 PointCloud2를 구독해 검증 전용 LiDAR 공 후보 통계를 저장한다.

이 노드는 publisher, service, action, robot-control API를 만들지 않는다. 기본 사용은
`NetworkInterfaceAddress=lo`로 고정한 localhost DDS에서 `ros2 bag play`와 함께 하는 오프라인 분석이다.
"""

import argparse
import json
import sys
import time
from pathlib import Path
from statistics import median
from typing import Dict, List

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from dribblebot.perception.pointcloud2 import pointcloud2_to_xyz
from dribblebot.perception.validated_lidar_ball_detector import (
    LidarBallDetectorConfig,
    ValidatedLidarBallDetector,
)


def _stamp_s(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


class TopicAnalyzer:
    """구독 callback만 가지는 분석기; detector 결과는 JSON에만 기록한다."""

    def __init__(self, node, topic: str):
        from sensor_msgs.msg import PointCloud2

        self.detector = ValidatedLidarBallDetector(
            LidarBallDetectorConfig(expected_frame_id="base_link")
        )
        self.stamps: List[float] = []
        self.point_counts: List[int] = []
        self.frame_ids = set()
        self.detections: List[Dict[str, object]] = []
        self.errors: List[str] = []
        self.subscription = node.create_subscription(PointCloud2, topic, self._on_cloud, 10)

    def _on_cloud(self, message) -> None:
        try:
            stamp = _stamp_s(message.header)
            points = pointcloud2_to_xyz(
                message.data, message.fields, message.point_step, message.width,
                message.height, message.row_step, message.is_bigendian,
            )
            self.stamps.append(stamp)
            self.point_counts.append(len(points))
            self.frame_ids.add(message.header.frame_id)
            result = self.detector.detect(
                points, frame_id=message.header.frame_id, stamp_s=stamp,
            )
            if result is not None:
                self.detections.append({
                    "stamp_s": result.stamp_s,
                    "center_base_xyz": result.center_base_xyz,
                    "radius_m": result.radius_m,
                    "confidence": result.confidence,
                    "mean_residual_m": result.mean_residual_m,
                    "inlier_count": result.inlier_count,
                })
        except Exception as error:
            # 한 프레임 오류가 전체 bag 판정을 숨기지 않게 JSON에 기록한다.
            self.errors.append(str(error))

    def result(self, topic: str) -> Dict[str, object]:
        intervals = np.diff(self.stamps) if len(self.stamps) > 1 else np.empty(0)
        median_interval = float(median(intervals)) if len(intervals) else None
        drop_intervals = (
            int(np.count_nonzero(intervals > 1.5 * median_interval))
            if median_interval else 0
        )
        frames = len(self.stamps)
        return {
            "topic": topic,
            "frame_ids": sorted(self.frame_ids),
            "frames": frames,
            "detections": len(self.detections),
            "detection_rate": float(len(self.detections) / frames) if frames else 0.0,
            "point_count_median": float(median(self.point_counts)) if self.point_counts else 0.0,
            "point_count_min": min(self.point_counts) if self.point_counts else 0,
            "point_count_max": max(self.point_counts) if self.point_counts else 0,
            "median_interval_s": median_interval,
            "estimated_rate_hz": float(1.0 / median_interval) if median_interval else 0.0,
            "drop_intervals": drop_intervals,
            "decode_or_frame_errors": self.errors[:20],
            "detection_samples": self.detections[:100],
            "note": (
                "오프라인 후보 통계다. ball/empty 비교와 위치 정확도 검증 전에는 "
                "kick input으로 사용하지 않는다."
            ),
        }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/utlidar/cloud_base")
    parser.add_argument("--max-messages", type=int, required=True)
    parser.add_argument("--timeout-s", type=float, default=90.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.max_messages <= 0 or args.timeout_s <= 0.0:
        parser.error("--max-messages and --timeout-s must be positive")

    import rclpy

    rclpy.init()
    node = rclpy.create_node("validated_lidar_bag_analyzer")
    analyzer = TopicAnalyzer(node, args.topic)
    deadline = time.monotonic() + args.timeout_s
    try:
        while len(analyzer.stamps) < args.max_messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.20)
    finally:
        result = analyzer.result(args.topic)
        result["complete"] = len(analyzer.stamps) >= args.max_messages
        result["timed_out"] = not result["complete"]
        args.output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(
            {key: value for key, value in result.items() if key != "detection_samples"},
            ensure_ascii=False, indent=2,
        ))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
