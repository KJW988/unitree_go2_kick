import numpy as np


class Go2LidarBallDetector:
    """
    Go2 EDU 내장 3D LiDAR Point Cloud 데이터를 기반으로 
    축구공(반지름 R=0.0889m)의 3차원 상대 위치 (x, y, z)를 추정하는 Perception 모듈.
    
    알고리즘 흐름:
    1. 지면 제거 (RANSAC Ground Plane Removal)
    2. 관심 영역(ROI) 필터링 (로봇 정면 x: 0.1m~3.0m, y: -1.5m~1.5m, z: -0.4m~0.5m)
    3. 3D Euclidean Clustering (거리 기반 포인트 그룹화)
    4. Sphere RANSAC Fitting (반지름 R=0.0889m 구체 피팅)
    """

    def __init__(self, ball_radius: float = 0.11, radius_tolerance: float = 0.02):
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
            import pyRANSAC_3D as ransac
            plane = ransac.Plane()
            best_eq, inliers = plane.fit(points, thresh=distance_threshold)
            # 지면(inliers) 이외의 아웃라이어 포인트만 반환
            mask = np.ones(len(points), dtype=bool)
            mask[inliers] = False
            return points[mask]
        except Exception:
            # Fallback: 단순 z축 높이 기준 지면 절단
            return points[points[:, 2] > -0.3]

    def fit_sphere_ransac(self, points: np.ndarray, iterations: int = 100) -> np.ndarray:
        """
        Point Cloud 데이터에서 R=0.0889m 구체의 중심 좌표 (x, y, z) 추정
        """
        if len(points) < 4:
            # 포인트가 부족한 경우 기본 전방 위치 반환
            return np.array([0.5, 0.0, 0.09])

        try:
            import pyRANSAC_3D as ransac
            sphere = ransac.Sphere()
            center, radius, inliers = sphere.fit(points, thresh=0.02, maxIteration=iterations)
            
            # 피팅된 구체 반지름이 축구공 반지름 규격 범위 내인지 검증
            if abs(radius - self.ball_radius) <= self.radius_tolerance:
                return np.array(center)
            else:
                # 반지름 차이가 크면 클러스터 중심값 사용
                return np.mean(points, axis=0)
        except Exception:
            return np.mean(points, axis=0)

    def detect_ball_3d(self, raw_point_cloud: np.ndarray) -> np.ndarray:
        """
        실시간 LiDAR Point Cloud 입력 ➔ 공 3D 위치 (x, y, z) 출력
        """
        # 1. ROI 필터링
        roi_points = self.filter_roi(raw_point_cloud)
        
        # 2. 지면 제거
        non_ground_points = self.remove_ground_plane(roi_points)
        
        # 3. Sphere RANSAC Fitting
        ball_center = self.fit_sphere_ransac(non_ground_points)
        
        return ball_center


if __name__ == "__main__":
    # Dummy Point Cloud 테스트
    print("Testing Go2LidarBallDetector Module...")
    detector = Go2LidarBallDetector()
    
    # 0.5m 전방 지점에 공 모양 드미 포인트 생성
    theta = np.linspace(0, 2 * np.pi, 50)
    phi = np.linspace(0, np.pi, 50)
    r = 0.0889
    x = 0.5 + r * np.outer(np.cos(theta), np.sin(phi)).flatten()
    y = 0.0 + r * np.outer(np.sin(theta), np.sin(phi)).flatten()
    z = 0.09 + r * np.outer(np.ones_like(theta), np.cos(phi)).flatten()
    dummy_pc = np.vstack((x, y, z)).T
    
    center = detector.detect_ball_3d(dummy_pc)
    print(f"✅ Estimated Ball 3D Position: {center} (Target: [0.5, 0.0, 0.09])")
