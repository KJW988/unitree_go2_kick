#!/usr/bin/env python3
"""정지 Go2 rosbag에서 검증 전용 LiDAR 공 후보 통계를 생성한다.

이 스크립트는 sensor topic만 deserialize하며 ROS publisher, supervisor 또는
robot-control API를 만들지 않는다. ball/empty bag의 JSON을 비교해 실제 recall과
false positive를 판단하는 용도다.
"""

import argparse
import json
from pathlib import Path
from statistics import median
from typing import Dict, List

import numpy as np

from dribblebot.perception.pointcloud2 import pointcloud2_to_xyz
from dribblebot.perception.validated_lidar_ball_detector import (
    LidarBallDetectorConfig,
    ValidatedLidarBallDetector,
)


def _stamp_s(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


def analyze_bag(uri: Path, topic: str, max_messages: int = 0) -> Dict[str, object]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            "ROS2 Foxy environment is required: source the isolated Go2 DDS workspace first"
        ) from error

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"bag does not contain {topic!r}; available={sorted(topic_types)}")
    message_type = get_message(topic_types[topic])
    detector = ValidatedLidarBallDetector(LidarBallDetectorConfig(expected_frame_id="base_link"))
    stamps: List[float] = []
    detections: List[Dict[str, object]] = []
    frame_ids = set()
    frames = 0
    point_counts: List[int] = []
    while reader.has_next():
        topic_name, raw, _ = reader.read_next()
        if topic_name != topic:
            continue
        message = deserialize_message(raw, message_type)
        stamp = _stamp_s(message.header)
        frame_ids.add(message.header.frame_id)
        points = pointcloud2_to_xyz(
            message.data, message.fields, message.point_step, message.width,
            message.height, message.row_step, message.is_bigendian,
        )
        point_counts.append(len(points))
        result = detector.detect(points, frame_id=message.header.frame_id, stamp_s=stamp)
        stamps.append(stamp)
        frames += 1
        if result is not None:
            detections.append({
                "stamp_s": result.stamp_s,
                "center_base_xyz": result.center_base_xyz,
                "radius_m": result.radius_m,
                "confidence": result.confidence,
                "mean_residual_m": result.mean_residual_m,
                "inlier_count": result.inlier_count,
            })
        if max_messages and frames >= max_messages:
            break
    intervals = np.diff(stamps) if len(stamps) > 1 else np.empty(0)
    median_interval = float(median(intervals)) if len(intervals) else None
    drop_intervals = int(np.count_nonzero(intervals > 1.5 * median_interval)) if median_interval else 0
    return {
        "bag": str(uri),
        "topic": topic,
        "frame_ids": sorted(frame_ids),
        "frames": frames,
        "detections": len(detections),
        "detection_rate": float(len(detections) / frames) if frames else 0.0,
        "point_count_median": float(median(point_counts)) if point_counts else 0.0,
        "point_count_min": min(point_counts) if point_counts else 0,
        "point_count_max": max(point_counts) if point_counts else 0,
        "median_interval_s": median_interval,
        "estimated_rate_hz": float(1.0 / median_interval) if median_interval else 0.0,
        "drop_intervals": drop_intervals,
        "detection_samples": detections[:100],
        "note": "이 JSON은 후보 통계다. ball bag과 empty bag 비교 전에는 kick input으로 사용하지 않는다.",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/utlidar/cloud_base")
    parser.add_argument("--max-messages", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze_bag(args.bag, args.topic, args.max_messages)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "detection_samples"}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
