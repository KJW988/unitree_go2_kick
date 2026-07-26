"""정지 Go2 rosbag 검증용 temporal LiDAR ball detector.

내장 LiDAR의 단일 65 ms frame은 11 cm 공 표면점이 희소할 수 있다. 이 모듈은
정지 로봇의 연속 base-frame cloud를 non-overlapping window로 합친 뒤 기존
검증 detector를 한 번만 호출한다. 킥·제어 interface는 import하지 않는다.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .validated_lidar_ball_detector import (
    BallDetection,
    LidarBallDetectorConfig,
    ValidatedLidarBallDetector,
)


@dataclass(frozen=True)
class TemporalEvaluation:
    detection: Optional[BallDetection]
    source_frames: int
    source_points: int


def make_static_ball_validation_detector() -> ValidatedLidarBallDetector:
    """1 m 정지 ball bag의 재설계 검증용 보수적 전방 lane profile.

    이 profile은 실공 bag에서 recall/false-positive가 입증되기 전에는 runtime
    search 또는 kick input에 사용하지 않는다.
    """

    return ValidatedLidarBallDetector(LidarBallDetectorConfig(
        expected_frame_id="base_link",
        # 관측된 self-leg sphere 후보를 제외하고, 기록한 1 m 전방 ball lane만 본다.
        roi_x_m=(0.70, 1.45),
        roi_y_m=(-0.30, 0.30),
        roi_z_m=(-0.80, 0.20),
        min_cluster_points=8,
        max_cluster_points=500,
        radius_tolerance_m=0.030,
        sphere_inlier_threshold_m=0.018,
        max_mean_residual_m=0.013,
        min_confidence=0.30,
    ))


class StaticTemporalBallDetector:
    """정지 base-frame cloud를 window 단위로 병합하는 validation-only wrapper."""

    def __init__(
        self,
        detector: Optional[ValidatedLidarBallDetector] = None,
        window_frames: int = 12,
    ):
        if window_frames < 2:
            raise ValueError("window_frames must be at least 2")
        self.detector = detector or make_static_ball_validation_detector()
        self.window_frames = window_frames
        self._frames = []

    def update(
        self,
        points: np.ndarray,
        frame_id: str = "base_link",
        stamp_s: Optional[float] = None,
    ) -> Optional[TemporalEvaluation]:
        """Window이 찼을 때만 병합 검출 결과를 반환한다."""

        base_points = self.detector._to_base_frame(
            np.asarray(points, dtype=np.float64), frame_id,
        )
        self._frames.append(base_points[np.isfinite(base_points).all(axis=1)])
        if len(self._frames) < self.window_frames:
            return None
        merged = np.concatenate(self._frames, axis=0)
        source_frames = len(self._frames)
        self._frames.clear()
        return TemporalEvaluation(
            detection=self.detector.detect(merged, frame_id="base_link", stamp_s=stamp_s),
            source_frames=source_frames,
            source_points=int(len(merged)),
        )
