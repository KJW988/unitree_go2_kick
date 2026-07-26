"""검증 전용 LiDAR ball detector의 단계별 탈락 원인을 수치로 기록한다.

후보를 만들거나 robot interface에 값을 전달하지 않는다. 정지 rosbag의 detector
재설계를 위한 read-only 진단 보조 모듈이다.
"""

from typing import Dict, Optional

import numpy as np

from .validated_lidar_ball_detector import ValidatedLidarBallDetector


def diagnose_lidar_frame(
    detector: ValidatedLidarBallDetector,
    points: np.ndarray,
    frame_id: str,
) -> Dict[str, object]:
    """한 frame의 ROI/ground/cluster/sphere-fit 통계만 반환한다."""

    array = np.asarray(points, dtype=np.float64)
    array = array[np.isfinite(array).all(axis=1)]
    base_points = detector._roi(detector._to_base_frame(array, frame_id))
    result: Dict[str, object] = {
        "input_points": int(len(array)),
        "roi_points": int(len(base_points)),
    }
    if not len(base_points):
        result["stage"] = "empty_roi"
        return result
    result["roi_xyz_min"] = [float(value) for value in np.min(base_points, axis=0)]
    result["roi_xyz_max"] = [float(value) for value in np.max(base_points, axis=0)]
    plane = detector._ground_plane(base_points)
    if plane is None:
        result["stage"] = "no_ground_plane"
        return result

    normal, offset, ground_inliers = plane
    signed_distance = base_points @ normal + offset
    nonground = base_points[signed_distance > detector.config.ground_clearance_m]
    voxel = detector._voxel_downsample(nonground)
    clusters = list(detector._clusters(voxel))
    result.update({
        "stage": "clusters",
        "ground_normal": [float(value) for value in normal],
        "ground_offset": float(offset),
        "ground_inliers": int(np.count_nonzero(ground_inliers)),
        "nonground_points": int(len(nonground)),
        "voxel_points": int(len(voxel)),
        "cluster_count": int(len(clusters)),
        "largest_cluster_points": int(max((len(cluster) for cluster in clusters), default=0)),
        "cluster_sizes_desc": sorted((int(len(cluster)) for cluster in clusters), reverse=True)[:12],
    })
    fits = []
    for cluster in clusters:
        fit = detector._fit_sphere(cluster)
        if fit is None:
            continue
        center_height = float(fit.center @ normal + offset)
        fits.append({
            "center_base_xyz": [float(value) for value in fit.center],
            "radius_m": float(fit.radius),
            "radius_error_m": float(abs(fit.radius - detector.config.ball_radius_m)),
            "mean_residual_m": float(fit.mean_residual_m),
            "inlier_count": int(np.count_nonzero(fit.inlier_mask)),
            "ground_support_error_m": float(abs(center_height - fit.radius)),
        })
    fits.sort(key=lambda item: (item["radius_error_m"], item["mean_residual_m"]))
    result["sphere_fit_count"] = len(fits)
    result["best_sphere_fits"] = fits[:6]
    return result
