"""검증 우선 Go2 LiDAR 축구공 후보 검출기.

기존 ``lidar_ball_detector``와 독립적이다. 이 모듈은 실제 bag 검증이 끝나기
전에는 킥 supervisor나 ``RelativeTargetObservation``을 import하지 않는다.
``/utlidar/cloud_base``처럼 이미 ``base_link``인 점군만 기본으로 허용하며,
raw LiDAR extrinsic은 측정값을 명시적으로 넣은 경우에만 적용한다.
"""

from dataclasses import dataclass
from itertools import product
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class RigidTransform:
    """측정·검증된 sensor->base extrinsic만 표현한다."""

    rotation_base_from_sensor: Tuple[Tuple[float, float, float], ...]
    translation_base_from_sensor_m: Vec3

    def apply(self, points: np.ndarray) -> np.ndarray:
        rotation = np.asarray(self.rotation_base_from_sensor, dtype=np.float64)
        translation = np.asarray(self.translation_base_from_sensor_m, dtype=np.float64)
        if rotation.shape != (3, 3):
            raise ValueError("rotation_base_from_sensor must be 3x3")
        return points @ rotation.T + translation


@dataclass(frozen=True)
class LidarBallDetectorConfig:
    """검증을 통해 조정할 수 있는, 보수적인 base-frame detector 설정."""

    expected_frame_id: str = "base_link"
    sensor_to_base: Optional[RigidTransform] = None
    ball_radius_m: float = 0.11
    radius_tolerance_m: float = 0.025
    roi_x_m: Tuple[float, float] = (0.25, 2.50)
    roi_y_m: Tuple[float, float] = (-0.90, 0.90)
    roi_z_m: Tuple[float, float] = (-0.80, 0.35)
    ground_inlier_threshold_m: float = 0.018
    ground_clearance_m: float = 0.018
    ground_min_inliers: int = 30
    ground_max_tilt_rad: float = 0.30
    voxel_size_m: float = 0.012
    cluster_tolerance_m: float = 0.050
    min_cluster_points: int = 5
    max_cluster_points: int = 500
    sphere_inlier_threshold_m: float = 0.016
    sphere_ransac_iterations: int = 120
    max_mean_residual_m: float = 0.012
    min_confidence: float = 0.55
    random_seed: int = 20260726


@dataclass(frozen=True)
class BallDetection:
    center_base_xyz: Vec3
    radius_m: float
    confidence: float
    mean_residual_m: float
    inlier_count: int
    cluster_point_count: int
    stamp_s: Optional[float] = None


@dataclass(frozen=True)
class _SphereFit:
    center: np.ndarray
    radius: float
    inlier_mask: np.ndarray
    mean_residual_m: float


def _sphere_from_four(points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    """4개 점의 algebraic sphere 해. 공면/퇴화 표본은 거절한다."""

    matrix = np.column_stack((2.0 * points, np.ones(4)))
    rhs = np.sum(points * points, axis=1)
    if np.linalg.matrix_rank(matrix) < 4:
        return None
    result = np.linalg.solve(matrix, rhs)
    center = result[:3]
    radius_squared = result[3] + float(np.dot(center, center))
    if radius_squared <= 0.0 or not np.isfinite(radius_squared):
        return None
    return center, float(np.sqrt(radius_squared))


def _least_squares_sphere(points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    if len(points) < 4:
        return None
    matrix = np.column_stack((2.0 * points, np.ones(len(points))))
    rhs = np.sum(points * points, axis=1)
    result, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
    if rank < 4:
        return None
    center = result[:3]
    radius_squared = result[3] + float(np.dot(center, center))
    if radius_squared <= 0.0 or not np.isfinite(radius_squared):
        return None
    return center, float(np.sqrt(radius_squared))


class ValidatedLidarBallDetector:
    """지면 제거, sparse clustering, sphere fit을 분리해 검증하는 detector."""

    def __init__(self, config: Optional[LidarBallDetectorConfig] = None):
        self.config = config or LidarBallDetectorConfig()

    def _to_base_frame(self, points: np.ndarray, frame_id: str) -> np.ndarray:
        if frame_id == self.config.expected_frame_id:
            return points
        if self.config.sensor_to_base is None:
            raise ValueError(
                "point cloud is not in base_link and no measured sensor_to_base "
                f"transform was configured (got frame_id={frame_id!r})"
            )
        return self.config.sensor_to_base.apply(points)

    def _roi(self, points: np.ndarray) -> np.ndarray:
        cfg = self.config
        mask = (
            (points[:, 0] >= cfg.roi_x_m[0]) & (points[:, 0] <= cfg.roi_x_m[1])
            & (points[:, 1] >= cfg.roi_y_m[0]) & (points[:, 1] <= cfg.roi_y_m[1])
            & (points[:, 2] >= cfg.roi_z_m[0]) & (points[:, 2] <= cfg.roi_z_m[1])
        )
        return points[mask]

    def _ground_plane(self, points: np.ndarray) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
        cfg = self.config
        if len(points) < cfg.ground_min_inliers:
            return None
        rng = np.random.default_rng(cfg.random_seed)
        best: Optional[Tuple[np.ndarray, float, np.ndarray]] = None
        best_count = 0
        min_normal_z = float(np.cos(cfg.ground_max_tilt_rad))
        for _ in range(cfg.sphere_ransac_iterations):
            sample = points[rng.choice(len(points), size=3, replace=False)]
            normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
            magnitude = float(np.linalg.norm(normal))
            if magnitude <= 1e-9:
                continue
            normal /= magnitude
            if normal[2] < 0.0:
                normal = -normal
            if normal[2] < min_normal_z:
                continue
            offset = -float(np.dot(normal, sample[0]))
            inliers = np.abs(points @ normal + offset) <= cfg.ground_inlier_threshold_m
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best = (normal, offset, inliers)
                best_count = count
        if best is None or best_count < cfg.ground_min_inliers:
            return None
        normal, _, inliers = best
        ground_points = points[inliers]
        centroid = np.mean(ground_points, axis=0)
        _, _, vectors = np.linalg.svd(ground_points - centroid, full_matrices=False)
        normal = vectors[-1]
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < min_normal_z:
            return None
        return normal, -float(np.dot(normal, centroid)), inliers

    def _voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        if len(points) == 0:
            return points
        keys = np.floor(points / self.config.voxel_size_m).astype(np.int64)
        _, indices = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(indices)]

    def _clusters(self, points: np.ndarray) -> Iterable[np.ndarray]:
        """spatial hash를 사용해 10k point cloud에서도 O(N)에 가깝게 동작한다."""

        if len(points) == 0:
            return []
        tolerance = self.config.cluster_tolerance_m
        cells = np.floor(points / tolerance).astype(np.int64)
        buckets: Dict[Tuple[int, int, int], List[int]] = {}
        for index, cell in enumerate(cells):
            buckets.setdefault(tuple(int(value) for value in cell), []).append(index)
        visited = np.zeros(len(points), dtype=bool)
        offsets = list(product((-1, 0, 1), repeat=3))
        result: List[np.ndarray] = []
        tolerance_squared = tolerance * tolerance
        for seed in range(len(points)):
            if visited[seed]:
                continue
            visited[seed] = True
            queue = [seed]
            cluster_indices: List[int] = []
            while queue:
                index = queue.pop()
                cluster_indices.append(index)
                cell = cells[index]
                for delta in offsets:
                    key = (int(cell[0] + delta[0]), int(cell[1] + delta[1]), int(cell[2] + delta[2]))
                    for neighbor in buckets.get(key, ()):
                        if visited[neighbor]:
                            continue
                        if float(np.sum((points[neighbor] - points[index]) ** 2)) <= tolerance_squared:
                            visited[neighbor] = True
                            queue.append(neighbor)
            if self.config.min_cluster_points <= len(cluster_indices) <= self.config.max_cluster_points:
                result.append(points[np.asarray(cluster_indices, dtype=np.int64)])
        return result

    def _fit_sphere(self, cluster: np.ndarray) -> Optional[_SphereFit]:
        cfg = self.config
        if len(cluster) < 4:
            return None
        rng = np.random.default_rng(cfg.random_seed + len(cluster))
        best_mask: Optional[np.ndarray] = None
        best_count = 0
        radius_low = cfg.ball_radius_m - cfg.radius_tolerance_m
        radius_high = cfg.ball_radius_m + cfg.radius_tolerance_m
        for _ in range(cfg.sphere_ransac_iterations):
            model = _sphere_from_four(cluster[rng.choice(len(cluster), size=4, replace=False)])
            if model is None:
                continue
            center, radius = model
            if not radius_low <= radius <= radius_high:
                continue
            residuals = np.abs(np.linalg.norm(cluster - center, axis=1) - radius)
            inliers = residuals <= cfg.sphere_inlier_threshold_m
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_mask = inliers
                best_count = count
        if best_mask is None or best_count < cfg.min_cluster_points:
            return None
        refined = _least_squares_sphere(cluster[best_mask])
        if refined is None:
            return None
        center, radius = refined
        residuals = np.abs(np.linalg.norm(cluster - center, axis=1) - radius)
        inliers = residuals <= cfg.sphere_inlier_threshold_m
        if int(np.count_nonzero(inliers)) < cfg.min_cluster_points:
            return None
        refined = _least_squares_sphere(cluster[inliers])
        if refined is None:
            return None
        center, radius = refined
        residuals = np.abs(np.linalg.norm(cluster - center, axis=1) - radius)
        inliers = residuals <= cfg.sphere_inlier_threshold_m
        return _SphereFit(
            center=center,
            radius=radius,
            inlier_mask=inliers,
            mean_residual_m=float(np.mean(residuals[inliers])) if np.any(inliers) else float("inf"),
        )

    def _candidate(self, cluster: np.ndarray, normal: np.ndarray, offset: float, stamp_s: Optional[float]) -> Optional[BallDetection]:
        fit = self._fit_sphere(cluster)
        if fit is None:
            return None
        cfg = self.config
        center = fit.center
        if not (
            cfg.roi_x_m[0] <= center[0] <= cfg.roi_x_m[1]
            and cfg.roi_y_m[0] <= center[1] <= cfg.roi_y_m[1]
            and cfg.roi_z_m[0] <= center[2] <= cfg.roi_z_m[1]
            and abs(fit.radius - cfg.ball_radius_m) <= cfg.radius_tolerance_m
            and fit.mean_residual_m <= cfg.max_mean_residual_m
        ):
            return None
        center_height = float(center @ normal + offset)
        if abs(center_height - fit.radius) > 0.040:
            return None
        inlier_count = int(np.count_nonzero(fit.inlier_mask))
        radius_quality = max(0.0, 1.0 - abs(fit.radius - cfg.ball_radius_m) / cfg.radius_tolerance_m)
        residual_quality = max(0.0, 1.0 - fit.mean_residual_m / cfg.max_mean_residual_m)
        count_quality = min(1.0, inlier_count / 18.0)
        support_quality = max(0.0, 1.0 - abs(center_height - fit.radius) / 0.040)
        confidence = float(radius_quality * residual_quality * count_quality * support_quality)
        if confidence < cfg.min_confidence:
            return None
        return BallDetection(
            center_base_xyz=(float(center[0]), float(center[1]), float(center[2])),
            radius_m=float(fit.radius),
            confidence=confidence,
            mean_residual_m=fit.mean_residual_m,
            inlier_count=inlier_count,
            cluster_point_count=len(cluster),
            stamp_s=stamp_s,
        )

    def detect(self, points: np.ndarray, frame_id: str = "base_link", stamp_s: Optional[float] = None) -> Optional[BallDetection]:
        """한 프레임에서 confidence가 가장 높은 공 후보만 반환한다."""

        array = np.asarray(points, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError("points must have shape (N, 3)")
        array = array[np.isfinite(array).all(axis=1)]
        if len(array) == 0:
            return None
        base_points = self._roi(self._to_base_frame(array, frame_id))
        plane = self._ground_plane(base_points)
        if plane is None:
            return None
        normal, offset, _ = plane
        signed_distance = base_points @ normal + offset
        nonground = base_points[signed_distance > self.config.ground_clearance_m]
        candidates = [
            candidate
            for cluster in self._clusters(self._voxel_downsample(nonground))
            for candidate in (self._candidate(cluster, normal, offset, stamp_s),)
            if candidate is not None
        ]
        return max(candidates, key=lambda item: item.confidence) if candidates else None


class StaleDetectionGate:
    """후속 adapter가 사용할 fail-closed freshness gate. 아직 킥에 연결하지 않는다."""

    def __init__(self, max_age_s: float = 0.20):
        self.max_age_s = max_age_s
        self._latest: Optional[BallDetection] = None

    def update(self, detection: Optional[BallDetection]) -> None:
        if detection is not None:
            self._latest = detection

    def current(self, now_s: float) -> Optional[BallDetection]:
        if self._latest is None or self._latest.stamp_s is None:
            return None
        if now_s - self._latest.stamp_s > self.max_age_s:
            return None
        return self._latest
