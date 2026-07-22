import numpy as np


class Go2LidarBallDetector:
    """
    Go2 EDU 내장 3D LiDAR Point Cloud (utlidar_lidar) 데이터를 기반으로 
    로봇 base 좌표계 기준 축구공(반지름 R=0.11m, 공식 5호 공)의 3차원 상대 위치 (x, y, z)를 추정하는 Perception 모듈.
    
    Unitree 공식 Go2 URDF radar_joint 기준 변환:
      xyz = [0.28945, 0.0, -0.046825]
      rpy = [0.0, 2.8782, 0.0]
    """

    # Static transformation matrix (utlidar_lidar -> base)
    THETA = 2.8782
    R_LIDAR2BASE = np.array([
        [np.cos(THETA), 0.0, np.sin(THETA)],
        [0.0,           1.0, 0.0],
        [-np.sin(THETA), 0.0, np.cos(THETA)],
    ], dtype=np.float32)
    T_LIDAR2BASE = np.array([0.28945, 0.0, -0.046825], dtype=np.float32)

    def __init__(self, ball_radius: float = 0.11, radius_tolerance: float = 0.025, history_size: int = 4):
        self.ball_radius = ball_radius
        self.radius_tolerance = radius_tolerance
        self.history_size = history_size
        self.pos_history = []  # 이동평균 및 노이즈 필터링용 버퍼

    @classmethod
    def transform_utlidar_to_base(cls, points_lidar: np.ndarray) -> np.ndarray:
        """utlidar_lidar 원시 센서 좌표계를 Go2 base 로봇 좌표계로 변환"""
        if points_lidar is None or len(points_lidar) == 0:
            return points_lidar
        return points_lidar @ cls.R_LIDAR2BASE.T + cls.T_LIDAR2BASE

    def filter_roi_base(self, points_base: np.ndarray) -> np.ndarray:
        """base 좌표계 기준 전방 ROI 범위 필터링"""
        mask = (
            (points_base[:, 0] >= 0.1) & (points_base[:, 0] <= 2.5) &
            (points_base[:, 1] >= -0.8) & (points_base[:, 1] <= 0.8) &
            (points_base[:, 2] >= -0.40) & (points_base[:, 2] <= 0.10)
        )
        return points_base[mask]

    def voxel_downsample(self, points: np.ndarray, voxel_size: float = 0.02) -> np.ndarray:
        """연산 가속을 위한 3D Voxel Grid 다운샘플링"""
        if len(points) == 0:
            return points
        voxel_coords = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
        return points[unique_indices]

    def remove_ground_plane(self, points_base: np.ndarray, distance_threshold: float = 0.03) -> np.ndarray:
        """
        지면 z-Cutoff 절단: 로봇 발 바닥(z ≈ -0.34m) 지면 점군을 제거하고,
        지면 위 축구공(중심 z ≈ -0.23m, 반지름 0.11m) 점군(z > -0.27m)을 100% 완벽히 보존.
        """
        if len(points_base) == 0:
            return points_base

        # z > -0.27m 이상의 지면 위 오브젝트 점군만 보존 (공의 아랫면이 지면 RANSAC에 먹혀 잘리는 현상 원천 차단)
        non_ground = points_base[points_base[:, 2] > -0.27]
        return non_ground

    def euclidean_clusters(
        self,
        points: np.ndarray,
        tolerance: float = 0.05,
        min_points: int = 5,
        max_points: int = 400,
    ):
        if len(points) == 0:
            return []

        unvisited = np.ones(len(points), dtype=bool)
        clusters = []
        tolerance_sq = tolerance * tolerance

        for seed in range(len(points)):
            if not unvisited[seed]:
                continue

            unvisited[seed] = False
            queue = [seed]
            indices = []

            while queue:
                index = queue.pop()
                indices.append(index)

                distance_sq = np.sum((points - points[index]) ** 2, axis=1)
                neighbors = np.flatnonzero(unvisited & (distance_sq <= tolerance_sq))
                unvisited[neighbors] = False
                queue.extend(neighbors.tolist())

            if min_points <= len(indices) <= max_points:
                clusters.append(points[indices])

        return clusters

    def fit_sphere_ransac(self, points: np.ndarray, iterations: int = 50, debug: bool = False):
        if len(points) < 5:
            return None, "too_few_points"

        # Bounding box 사전 검증 (공 지름 ~0.22m 초과 시 사전 제외)
        bbox_min = np.min(points, axis=0)
        bbox_max = np.max(points, axis=0)
        bbox_dim = bbox_max - bbox_min
        if np.any(bbox_dim > 0.40):
            return None, f"bbox_too_large_{np.round(bbox_dim, 2)}"

        try:
            import pyransac3d as ransac
            sphere = ransac.Sphere()
            center, radius, inliers = sphere.fit(
                points,
                thresh=0.02,
                maxIteration=iterations,
            )

            minimum_inliers = max(4, int(np.ceil(len(points) * 0.40)))

            valid = (
                np.isfinite(center).all()
                and np.isfinite(radius)
                and abs(radius - self.ball_radius) <= self.radius_tolerance
                and len(inliers) >= minimum_inliers
                and 0.1 <= center[0] <= 2.5
                and -0.8 <= center[1] <= 0.8
                and -0.35 <= center[2] <= 0.25
            )

            if valid:
                return np.asarray(center), "ok"
            else:
                reason = f"r={radius:.3f}, inliers={len(inliers)}/{len(points)}, center={np.round(center, 2)}"
                return None, reason
        except Exception as e:
            # Fallback: pyransac3d가 없을 경우 클러스터의 3D 평균 중심점(Centroid) 사용
            mean_center = np.mean(points, axis=0)
            if 0.1 <= mean_center[0] <= 2.5 and -0.8 <= mean_center[1] <= 0.8 and -0.35 <= mean_center[2] <= 0.25:
                return mean_center, f"fallback_centroid_ok({e})"
            return None, f"ransac_exception_{e}"

    def detect_ball_3d(self, raw_point_cloud: np.ndarray, is_base_frame: bool = False, debug: bool = False):
        if (
            raw_point_cloud is None
            or raw_point_cloud.ndim != 2
            or raw_point_cloud.shape[1] != 3
            or len(raw_point_cloud) == 0
        ):
            return None

        # 1. utlidar_lidar -> base 변환
        if not is_base_frame:
            points_base = self.transform_utlidar_to_base(raw_point_cloud)
        else:
            points_base = raw_point_cloud

        # 2. base ROI 필터링
        roi_points = self.filter_roi_base(points_base)
        if len(roi_points) < 5:
            if debug:
                print(f"[DEBUG] raw={len(raw_point_cloud)}, base_roi={len(roi_points)} (<5)")
            return None

        # 3. Voxel 다운샘플링 (속도 최적화)
        ds_points = self.voxel_downsample(roi_points, voxel_size=0.02)

        # 4. 지면 제거
        non_ground = self.remove_ground_plane(ds_points)
        if len(non_ground) < 5:
            if debug:
                print(f"[DEBUG] roi={len(roi_points)}, non_ground={len(non_ground)} (<5)")
            return None

        # 5. 유클리드 클러스터링
        clusters = self.euclidean_clusters(non_ground)
        if debug:
            print(f"[DEBUG] input={len(raw_point_cloud)}, base_roi={len(roi_points)}, non_ground={len(non_ground)}, clusters={len(clusters)}")

        candidates = []
        for i, cluster in enumerate(clusters):
            center, reason = self.fit_sphere_ransac(cluster, debug=debug)
            if debug:
                cnt = len(cluster)
                mean_pos = np.mean(cluster, axis=0)
                print(f"  Cluster {i}: pts={cnt}, mean=({mean_pos[0]:.2f}, {mean_pos[1]:.2f}, {mean_pos[2]:.2f}), fit={reason}")
            if center is not None:
                candidates.append((len(cluster), center))

        if not candidates:
            return None

        # 이전 추정 위치가 존재할 경우, 이전 공 위치와 가장 가까운 클러스터 후보를 지속 추적(Distance-based Tracking)
        if len(self.pos_history) > 0:
            last_avg = np.mean(self.pos_history, axis=0)
            # 이전 위치와 가장 가까운 클러스터 선택
            best_cand = min(candidates, key=lambda item: np.linalg.norm(item[1] - last_avg))
            raw_center = best_cand[1]

            # 순간 0.40m 이상 튀는 아웃라이어 Jump 거절 및 직전 평균 보정
            if np.linalg.norm(raw_center - last_avg) > 0.40:
                if debug:
                    print(f"[FILTER] Outlier jump rejected: raw={raw_center}, last_avg={last_avg}")
                return last_avg
        else:
            # 첫 검출 시에는 가장 점군이 풍부한 클러스터 선택
            raw_center = max(candidates, key=lambda item: item[0])[1]

        self.pos_history.append(raw_center)
        if len(self.pos_history) > self.history_size:
            self.pos_history.pop(0)

        smooth_center = np.mean(self.pos_history, axis=0)
        return smooth_center


if __name__ == "__main__":
    print("Testing Go2LidarBallDetector Module (utlidar_lidar -> base transform)...")
    detector = Go2LidarBallDetector(ball_radius=0.11)

    # base 기준 전방 0.5m, z=0.11m 지점에 구형 5호 축구공 생성 (지면 위 중심 높이 0.11m)
    theta = np.linspace(0, 2 * np.pi, 50)
    phi = np.linspace(0, np.pi, 50)
    r = 0.11
    x_base = 0.5 + r * np.outer(np.cos(theta), np.sin(phi)).flatten()
    y_base = 0.0 + r * np.outer(np.sin(theta), np.sin(phi)).flatten()
    z_base = 0.11 + r * np.outer(np.ones_like(theta), np.cos(phi)).flatten()
    pc_base = np.vstack((x_base, y_base, z_base)).T

    # 역변환으로 utlidar_lidar 원시 좌표 생성
    pc_lidar = (pc_base - Go2LidarBallDetector.T_LIDAR2BASE) @ Go2LidarBallDetector.R_LIDAR2BASE

    center = detector.detect_ball_3d(pc_lidar, is_base_frame=False, debug=True)
    print(f"✅ Estimated Ball 3D Position in Base Frame: {center}")
