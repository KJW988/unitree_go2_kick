"""YOLO 2D ball 후보와 Depth/LiDAR를 base-frame 위치로 결합하는 검증 전용 core.

이 모듈은 ROS, OpenCV, YOLO runtime, 킥 supervisor를 import하지 않는다. 실제
frontend는 camera calibration과 image topic 검증 후 이 순수 계산 계층을 호출한다.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

Vec2 = Tuple[float, float]
Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    rotation_base_from_camera: Tuple[Tuple[float, float, float], ...]
    translation_base_from_camera_m: Vec3


@dataclass(frozen=True)
class ImageBallDetection:
    """YOLO frontend가 제공할 class-filtered ball bbox. 좌표는 pixel 단위다."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float
    confidence: float
    stamp_s: float

    @property
    def center_px(self) -> Vec2:
        return ((self.x_min + self.x_max) * 0.5, (self.y_min + self.y_max) * 0.5)


@dataclass(frozen=True)
class FusionConfig:
    bbox_margin_px: float = 12.0
    min_lidar_points: int = 3
    lidar_bin_width_m: float = 0.08
    min_range_m: float = 0.20
    max_range_m: float = 3.00
    max_depth_lidar_delta_m: float = 0.25
    min_confidence: float = 0.45
    max_jump_m: float = 0.30
    max_age_s: float = 0.25


@dataclass(frozen=True)
class FusedBallObservation:
    ball_base_xyz: Vec3
    confidence: float
    stamp_s: float
    range_source: str
    lidar_point_count: int

    @property
    def ball_base_xy(self) -> Vec2:
        return (self.ball_base_xyz[0], self.ball_base_xyz[1])


def _camera_arrays(model: CameraModel) -> Tuple[np.ndarray, np.ndarray]:
    rotation = np.asarray(model.rotation_base_from_camera, dtype=np.float64)
    translation = np.asarray(model.translation_base_from_camera_m, dtype=np.float64)
    if rotation.shape != (3, 3) or translation.shape != (3,):
        raise ValueError("camera extrinsic must be 3x3 rotation and 3-vector translation")
    return rotation, translation


def _pixel_ray_camera(model: CameraModel, pixel: Vec2) -> np.ndarray:
    if model.fx <= 0.0 or model.fy <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    x = (float(pixel[0]) - model.cx) / model.fx
    y = (float(pixel[1]) - model.cy) / model.fy
    ray = np.array((x, y, 1.0), dtype=np.float64)
    return ray / np.linalg.norm(ray)


def _project_base_to_pixels(model: CameraModel, points_base: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rotation, translation = _camera_arrays(model)
    points_camera = (np.asarray(points_base, dtype=np.float64) - translation) @ rotation
    forward = points_camera[:, 2]
    pixels = np.empty((len(points_camera), 2), dtype=np.float64)
    pixels[:, 0] = model.fx * points_camera[:, 0] / np.maximum(forward, 1e-12) + model.cx
    pixels[:, 1] = model.fy * points_camera[:, 1] / np.maximum(forward, 1e-12) + model.cy
    return pixels, forward


def _lidar_range_in_bbox(
    model: CameraModel,
    detection: ImageBallDetection,
    lidar_points_base: np.ndarray,
    config: FusionConfig,
) -> Tuple[Optional[float], int]:
    points = np.asarray(lidar_points_base, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("lidar_points_base must have shape (N, 3)")
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return None, 0
    pixels, forward = _project_base_to_pixels(model, points)
    margin = config.bbox_margin_px
    inside = (
        (forward > 0.0)
        & (pixels[:, 0] >= detection.x_min - margin)
        & (pixels[:, 0] <= detection.x_max + margin)
        & (pixels[:, 1] >= detection.y_min - margin)
        & (pixels[:, 1] <= detection.y_max + margin)
    )
    ranges = np.linalg.norm(points[inside] - np.asarray(model.translation_base_from_camera_m), axis=1)
    ranges = ranges[(ranges >= config.min_range_m) & (ranges <= config.max_range_m)]
    if len(ranges) < config.min_lidar_points:
        return None, int(len(ranges))
    # Ground/background이 섞여도 같은 range band의 지지점이 가장 많은 mode를 고른다.
    bins = np.floor(ranges / config.lidar_bin_width_m).astype(np.int64)
    unique, counts = np.unique(bins, return_counts=True)
    mode_bin = unique[int(np.argmax(counts))]
    mode_ranges = ranges[bins == mode_bin]
    return float(np.median(mode_ranges)), int(len(ranges))


def fuse_image_ball_with_range(
    model: CameraModel,
    detection: ImageBallDetection,
    lidar_points_base: np.ndarray,
    depth_range_m: Optional[float] = None,
    config: Optional[FusionConfig] = None,
) -> Optional[FusedBallObservation]:
    """Depth 우선, LiDAR cross-check/fallback으로 base-frame 공 후보를 만든다.

    ``depth_range_m``은 bbox 중심 ray를 따른 metric range여야 한다. raw axial-depth
    image는 frontend에서 ray range로 변환한 뒤 전달한다.
    """

    cfg = config or FusionConfig()
    if not (0.0 <= detection.confidence <= 1.0) or detection.x_max <= detection.x_min or detection.y_max <= detection.y_min:
        return None
    lidar_range, lidar_count = _lidar_range_in_bbox(model, detection, lidar_points_base, cfg)
    valid_depth = depth_range_m is not None and cfg.min_range_m <= depth_range_m <= cfg.max_range_m
    if valid_depth and lidar_range is not None and abs(float(depth_range_m) - lidar_range) > cfg.max_depth_lidar_delta_m:
        return None
    if valid_depth:
        selected_range, source = float(depth_range_m), "depth+lidar" if lidar_range is not None else "depth"
        range_quality = 1.0
    elif lidar_range is not None:
        selected_range, source = lidar_range, "lidar"
        range_quality = min(1.0, lidar_count / 10.0)
    else:
        return None
    confidence = float(detection.confidence * range_quality)
    if confidence < cfg.min_confidence:
        return None
    rotation, translation = _camera_arrays(model)
    base_point = translation + selected_range * (rotation @ _pixel_ray_camera(model, detection.center_px))
    return FusedBallObservation(
        ball_base_xyz=tuple(float(value) for value in base_point),
        confidence=confidence,
        stamp_s=detection.stamp_s,
        range_source=source,
        lidar_point_count=lidar_count,
    )


class FusedBallStaleGate:
    """공간 jump와 age를 fail-closed로 막는 fusion output gate."""

    def __init__(self, config: Optional[FusionConfig] = None):
        self.config = config or FusionConfig()
        self._latest: Optional[FusedBallObservation] = None

    def update(self, observation: Optional[FusedBallObservation]) -> Optional[FusedBallObservation]:
        if observation is None:
            return None
        if self._latest is not None:
            jump = np.linalg.norm(
                np.asarray(observation.ball_base_xy) - np.asarray(self._latest.ball_base_xy)
            )
            if jump > self.config.max_jump_m:
                return None
        self._latest = observation
        return observation

    def current(self, now_s: float) -> Optional[FusedBallObservation]:
        if self._latest is None or now_s - self._latest.stamp_s > self.config.max_age_s:
            return None
        return self._latest
