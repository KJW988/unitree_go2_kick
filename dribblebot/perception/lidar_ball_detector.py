import numpy as np


class Go2LidarBallDetector:
    """
    Go2 EDU 내장 3D LiDAR Point Cloud 데이터를 기반으로 
    축구공(반지름 R=0.11m)의 3차원 상대 위치 (x, y, z)를 추정하는 Perception 모듈.
    
    알고리즘 흐름:
    1. 지면 제거 (RANSAC Ground Plane Removal)
    2. 관심 영역(ROI) 필터링 (로봇 정면 x: 0.1m~3.0m, y: -1.5m~1.5m, z: -0.4m~0.5m)
    3. 3D Euclidean Clustering (거리 기반 포인트 그룹화)
    4. Sphere RANSAC Fitting (반지름 R=0.11m 구체 피팅)
    """

<<<<<<< HEAD
    def __init__(self, ball_radius: float = 0.11, radius_tolerance: float = 0.02):
=======
    def __init__(self, ball_radius: float = 0.11, radius_tolerance: float = 0.025):
>>>>>>> d527aab (Add Go2 LiDAR ball detection)
        self.ball_radius = ball_radius
        self.radius_tolerance = radius_tolerance

    def filter_roi(self, points: np.ndarray) -> np.ndarray:
        """로봇 기준 전방 ROI 범위 필터링"""
        mask = (
            (points[:, 0] >= 0.1) & (points[:, 0] <= 3.0) &
            (points[:, 1] >= -1.5) & (points[:, 1] <= 1.5) &
            (points[:, 2] >= -0.4) & (points[:, 2] <= 0.5)
        )
        return points[mask]

    def remove_ground_plane(self, points: np.ndarray, distance_threshold: float = 0.03) -> np.ndarray:
        """RANSAC 평면 피팅으로 지면 포인트 제거"""
        if len(points) < 10:
            return points

        try:
            import pyransac3d as ransac
            plane = ransac.Plane()
            _, inliers = plane.fit(points, thresh=distance_threshold, maxIteration=100)
            # 지면(inliers) 이외의 아웃라이어 포인트만 반환
            mask = np.ones(len(points), dtype=bool)
            mask[inliers] = False
            return points[mask]
        except Exception:
            # Fallback: 단순 z축 높이 기준 지면 절단
            return points[points[:, 2] > -0.3]

    def euclidean_clusters(
        self,
        points: np.ndarray,
        tolerance: float = 0.07,
        min_points: int = 6,
        max_points: int = 250,
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

                distance_sq = np.sum(
                    (points - points[index]) ** 2,
                    axis=1,
                )
                neighbors = np.flatnonzero(
                    unvisited & (distance_sq <= tolerance_sq)
                )
                unvisited[neighbors] = False
                queue.extend(neighbors.tolist())

            if min_points <= len(indices) <= max_points:
                clusters.append(points[indices])

        return clusters

    def fit_sphere_ransac(self, points: np.ndarray, iterations: int = 100):
        if len(points) < 6:
            return None

        try:
            import pyransac3d as ransac

            sphere = ransac.Sphere()
            center, radius, inliers = sphere.fit(
                points,
                thresh=0.02,
                maxIteration=iterations,
            )

            minimum_inliers = max(
                5,
                int(np.ceil(len(points) * 0.45)),
            )

            valid = (
                np.isfinite(center).all()
                and np.isfinite(radius)
                and abs(radius - self.ball_radius) <= self.radius_tolerance
                and len(inliers) >= minimum_inliers
                and 0.1 <= center[0] <= 3.0
                and -1.5 <= center[1] <= 1.5
                and -0.30 <= center[2] <= 0.05
            )

            return np.asarray(center) if valid else None
        except Exception:
            return None

    def detect_ball_3d(self, raw_point_cloud: np.ndarray):
        if (
            raw_point_cloud is None
            or raw_point_cloud.ndim != 2
            or raw_point_cloud.shape[1] != 3
            or len(raw_point_cloud) == 0
        ):
            return None

        roi_points = self.filter_roi(raw_point_cloud)
        if len(roi_points) < 6:
            return None

        non_ground_points = self.remove_ground_plane(roi_points)
        if len(non_ground_points) < 6:
            return None

        candidates = []

        for cluster in self.euclidean_clusters(non_ground_points):
            center = self.fit_sphere_ransac(cluster)

            if center is not None:
                candidates.append((len(cluster), center))

        if not candidates:
            return None

        return max(candidates, key=lambda item: item[0])[1]



if __name__ == "__main__":
    # Dummy Point Cloud 테스트
    print("Testing Go2LidarBallDetector Module...")
    detector = Go2LidarBallDetector()
    
    # 0.5m 전방 지점에 공 모양 드미 포인트 생성
    theta = np.linspace(0, 2 * np.pi, 50)
    phi = np.linspace(0, np.pi, 50)
    r = 0.11
    x = 0.5 + r * np.outer(np.cos(theta), np.sin(phi)).flatten()
    y = 0.0 + r * np.outer(np.sin(theta), np.sin(phi)).flatten()
    z = 0.09 + r * np.outer(np.ones_like(theta), np.cos(phi)).flatten()
    dummy_pc = np.vstack((x, y, z)).T
    
    center = detector.detect_ball_3d(dummy_pc)
    print(f"✅ Estimated Ball 3D Position: {center} (Target: [0.5, 0.0, 0.09])")
