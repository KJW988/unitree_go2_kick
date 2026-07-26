#!/usr/bin/env python3
"""정지 rosbag의 dense odom-frame LiDAR cloud를 base-frame으로 검증한다.

`/utlidar/cloud_deskewed`와 `/utlidar/robot_odom`만 구독한다. publisher, service,
action, robot-control API는 만들지 않는다.
"""

import sys
from collections import deque
from pathlib import Path
from statistics import median
from typing import Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from dribblebot.perception.odom_transform import odom_points_to_base
from dribblebot.perception.pointcloud2 import pointcloud2_to_xyz
from dribblebot.perception.temporal_lidar_ball_detector import make_static_ball_validation_detector


def _stamp_s(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


class DeskewedTopicAnalyzer:
    """가장 가까운 이전 robot odom으로 dense cloud를 base_link로 변환한다."""

    def __init__(self, node, cloud_topic: str, odom_topic: str):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2

        self.detector = make_static_ball_validation_detector()
        self.odom: deque[Tuple[float, np.ndarray, np.ndarray]] = deque(maxlen=512)
        self.stamps: List[float] = []
        self.odom_age_s: List[float] = []
        self.point_counts: List[int] = []
        self.detections: List[Dict[str, object]] = []
        self.errors: List[str] = []
        self.field_layout: List[Dict[str, object]] = []
        self.cloud_subscription = node.create_subscription(PointCloud2, cloud_topic, self._on_cloud, 10)
        self.odom_subscription = node.create_subscription(Odometry, odom_topic, self._on_odom, 200)

    def _on_odom(self, message) -> None:
        pose = message.pose.pose
        self.odom.append((
            _stamp_s(message.header),
            np.array((pose.position.x, pose.position.y, pose.position.z), dtype=np.float64),
            np.array((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), dtype=np.float64),
        ))

    def _nearest_odom(self, stamp: float) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
        if not self.odom:
            return None
        return min(self.odom, key=lambda item: abs(item[0] - stamp))

    def _on_cloud(self, message) -> None:
        try:
            stamp = _stamp_s(message.header)
            pose = self._nearest_odom(stamp)
            if pose is None:
                self.errors.append("cloud received before robot_odom")
                return
            odom_stamp, translation, quaternion = pose
            age = abs(odom_stamp - stamp)
            if age > 0.050:
                self.errors.append(f"nearest robot_odom age {age:.6f}s exceeds 0.050s")
                return
            points_odom = pointcloud2_to_xyz(
                message.data, message.fields, message.point_step, message.width,
                message.height, message.row_step, message.is_bigendian,
            )
            points_base = odom_points_to_base(points_odom, translation, quaternion)
            self.stamps.append(stamp)
            self.odom_age_s.append(age)
            self.point_counts.append(len(points_base))
            if not self.field_layout:
                self.field_layout = [{
                    "name": str(field.name), "offset": int(field.offset),
                    "datatype": int(field.datatype), "count": int(field.count),
                } for field in message.fields]
            # dense cloud는 12개마다 한 frame만 평가해 CPU drop을 피한다.
            if len(self.stamps) % 12:
                return
            result = self.detector.detect(points_base, frame_id="base_link", stamp_s=stamp)
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
            self.errors.append(str(error))

    def result(self, cloud_topic: str) -> Dict[str, object]:
        intervals = np.diff(self.stamps) if len(self.stamps) > 1 else np.empty(0)
        median_interval = float(median(intervals)) if len(intervals) else None
        evaluated = len(self.stamps) // 12
        return {
            "topic": cloud_topic,
            "input_frame_id": "odom",
            "output_frame_id": "base_link",
            "frames": len(self.stamps),
            "evaluated_dense_frames": evaluated,
            "detections": len(self.detections),
            "detection_rate": float(len(self.detections) / evaluated) if evaluated else 0.0,
            "point_count_median": float(median(self.point_counts)) if self.point_counts else 0.0,
            "point_field_layout": self.field_layout,
            "median_odom_age_s": float(median(self.odom_age_s)) if self.odom_age_s else None,
            "max_odom_age_s": max(self.odom_age_s) if self.odom_age_s else None,
            "median_interval_s": median_interval,
            "estimated_rate_hz": float(1.0 / median_interval) if median_interval else 0.0,
            "detection_samples": self.detections[:100],
            "decode_or_transform_errors": self.errors[:20],
            "note": "정지 bag 검증 통계이며 kick input에는 연결하지 않는다.",
        }
