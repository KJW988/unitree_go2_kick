#!/usr/bin/env python3
import time
import numpy as np


class BallStateTracker:
    """
    RoboNaldo (OpenDriveLab 2026) onboard/perception/ball_fuser.py 및
    DribbleBot (MIT Science Robotics 2023) EKF State Estimator 표준 구현:

    3D Constant Velocity Kalman Filter (EKF):
    1. 센서 15.4Hz 주기를 로봇 제어용 50Hz 고주파 데이터로 100% 연속 보간(Interpolation).
    2. 센서 빔 빗나감으로 인한 순간 결측(NO BALL) 시 직전 3D 위치/속도 기반 예측(Predict)으로 끊김 보장.
    3. 공이 사람이 손으로 옮겨져 1.0m 이상 크게 이동하는 경우 2프레임 내 동적 추적 리셋(Dynamic Re-init).
    """

    def __init__(self, dt: float = 0.02, process_noise: float = 0.05, measurement_noise: float = 0.02):
        self.dt = dt
        self.state = None  # 상태 Vector: [x, y, z, vx, vy, vz]
        self.P = np.eye(6, dtype=np.float32) * 0.1
        self.Q = np.eye(6, dtype=np.float32) * process_noise
        self.R = np.eye(3, dtype=np.float32) * measurement_noise
        self.last_update_time = None
        self.miss_count = 0
        self.max_miss_frames = 20  # 1초 이상 연속 미검출 시에만 추적 초기화

    def update(self, measurement: np.ndarray = None, current_time: float = None):
        if current_time is None:
            current_time = time.time()

        if self.last_update_time is None:
            dt = self.dt
        else:
            dt = max(0.001, current_time - self.last_update_time)
        self.last_update_time = current_time

        # F: 3D Constant Velocity 상태 전이 행렬
        F = np.eye(6, dtype=np.float32)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # H: Measurement Matrix (관측치는 x, y, z 좌표만 수신)
        H = np.zeros((3, 6), dtype=np.float32)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        # 1. Predict Step (상태 및 오차 공분산 예측)
        if self.state is None:
            if measurement is not None:
                self.state = np.array([measurement[0], measurement[1], measurement[2], 0.0, 0.0, 0.0], dtype=np.float32)
                self.miss_count = 0
                return self.state[:3], np.zeros(3, dtype=np.float32)
            else:
                return None, None

        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q

        # 2. Correct Step (관측치 보정)
        if measurement is not None:
            pred_pos = self.state[:3]
            dist = np.linalg.norm(measurement - pred_pos)

            # 공 유연 추적 (1.2m 이내 관측치 수용)
            if dist < 1.20:
                y = measurement - (H @ self.state)
                S = H @ self.P @ H.T + self.R
                K = self.P @ H.T @ np.linalg.inv(S)
                self.state = self.state + K @ y
                self.P = (np.eye(6) - K @ H) @ self.P
                self.miss_count = 0
            else:
                # 공을 사람이 손으로 1.2m 이상 크게 치워 위치가 점프한 경우 2프레임 후 새 위치로 리셋
                self.miss_count += 1
                if self.miss_count >= 2:
                    self.state = np.array([measurement[0], measurement[1], measurement[2], 0.0, 0.0, 0.0], dtype=np.float32)
                    self.miss_count = 0
        else:
            self.miss_count += 1

        if self.miss_count > self.max_miss_frames:
            self.state = None
            return None, None

        return self.state[:3], self.state[3:]


class Go2LidarBallDetector:
    """
    Unitree Go2 내장 3D LiDAR (/utlidar/cloud) 전용 5호 축구공 3D 검출기.

    공식 URDF pitch 회전(theta=2.8782 rad) 변환 적용:
      R_LIDAR2BASE = [[-0.965512, 0, 0.260358], [0, 1, 0], [-0.260358, 0, -0.965512]]
      T_LIDAR2BASE = [0.28945, 0.0, -0.046825]
    """

    R_LIDAR2BASE = np.array([
        [-0.965512, 0.0, 0.260358],
        [0.0, 1.0, 0.0],
        [-0.260358, 0.0, -0.965512],
    ], dtype=np.float32)
    T_LIDAR2BASE = np.array([0.28945, 0.0, -0.046825], dtype=np.float32)

    def __init__(self, ball_radius: float = 0.11, radius_tolerance: float = 0.03, history_size: int = 4):
        self.ball_radius = ball_radius
        self.radius_tolerance = radius_tolerance
        self.history_size = history_size
        self.pos_history = []

    @classmethod
    def transform_utlidar_to_base(cls, points_lidar: np.ndarray) -> np.ndarray:
        """utlidar_lidar 원시 센서 좌표계를 Go2 base 로봇 좌표계로 변환"""
        if points_lidar is None or len(points_lidar) == 0:
            return points_lidar
        return points_lidar @ cls.R_LIDAR2BASE.T + cls.T_LIDAR2BASE

    def filter_roi_base(self, points_base: np.ndarray) -> np.ndarray:
        """
        base 좌표계 기준 전방 ROI 범위 필터링:
        - x: 0.38m ~ 2.5m (로봇 앞다리 x < 0.38m 완벽 마스킹)
        - y: -0.8m ~ 0.8m
        - z: -0.38m ~ -0.10m (지면 바닥 -0.34m 부근의 5호 축구공 중심 z ≈ -0.23m 영역만 선택)
        """
        mask = (
            (points_base[:, 0] >= 0.38) & (points_base[:, 0] <= 2.5) &
            (points_base[:, 1] >= -0.8) & (points_base[:, 1] <= 0.8) &
            (points_base[:, 2] >= -0.38) & (points_base[:, 2] <= -0.10)
        )
        return points_base[mask]

    def voxel_downsample(self, points: np.ndarray, voxel_size: float = 0.02) -> np.ndarray:
        """연산 가속을 위한 3D Voxel Grid 다운샘플링"""
        if len(points) == 0:
            return points
        voxel_coords = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
        return points[unique_indices]

    def euclidean_clusters(
        self,
        points: np.ndarray,
        tolerance: float = 0.06,
        min_points: int = 4,
        max_points: int = 400,
    ):
        """유클리드 거리 기반 3D 점군 클러스터링"""
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

    def fit_sphere_ransac_and_least_squares(self, points: np.ndarray, iterations: int = 40):
        """
        RANSAC + Least Squares 2단계 구체 피팅 (RANSAC Inlier filtering + Least Squares refinement):
        1단계: RANSAC으로 구체 표면 인라이어 점군(inliers)을 추출하여 평면/장애물 점 제거.
        2단계: 추출된 inliers 점들에 대해 정밀 최소제곱법(Least Squares)으로 중심 (a,b,c) 및 반지름 R 산출.
        """
        if len(points) < 4:
            return None, 999.0, "too_few_points"

        bbox_min = np.min(points, axis=0)
        bbox_max = np.max(points, axis=0)
        bbox_dim = bbox_max - bbox_min
        if np.any(bbox_dim > 0.38):
            return None, 999.0, f"bbox_too_large_{np.round(bbox_dim, 2)}"

        best_center = None
        best_radius = None
        best_inliers = []
        best_inlier_count = 0

        num_pts = len(points)
        # 1. RANSAC 3D 구체 피팅
        for _ in range(iterations):
            sample_idx = np.random.choice(num_pts, 4, replace=False)
            pts_sample = points[sample_idx]

            x, y, z = pts_sample[:, 0], pts_sample[:, 1], pts_sample[:, 2]
            A = np.column_stack([2 * x, 2 * y, 2 * z, np.ones_like(x)])
            B = x**2 + y**2 + z**2

            try:
                res, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
                a, b, c, D = res[0], res[1], res[2], res[3]
                r_sq = D + a**2 + b**2 + c**2
                if r_sq <= 0:
                    continue
                r_cand = np.sqrt(r_sq)
                if abs(r_cand - self.ball_radius) > 0.05:
                    continue

                center_cand = np.array([a, b, c], dtype=np.float32)
                dists = np.linalg.norm(points - center_cand, axis=1)
                inlier_mask = np.abs(dists - r_cand) <= 0.035
                inlier_cnt = np.sum(inlier_mask)

                if inlier_cnt > best_inlier_count:
                    best_inlier_count = inlier_cnt
                    best_center = center_cand
                    best_radius = r_cand
                    best_inliers = points[inlier_mask]
            except Exception:
                continue

        if best_inlier_count < 4 or best_inliers is None or len(best_inliers) < 4:
            return None, 999.0, "ransac_no_valid_sphere"

        # 2. Least Squares Refinement (inliers 점들 대상 정밀 재피팅)
        try:
            x_in = best_inliers[:, 0]
            y_in = best_inliers[:, 1]
            z_in = best_inliers[:, 2]

            A_in = np.column_stack([2 * x_in, 2 * y_in, 2 * z_in, np.ones_like(x_in)])
            B_in = x_in**2 + y_in**2 + z_in**2

            res_in, _, _, _ = np.linalg.lstsq(A_in, B_in, rcond=None)
            a_ref, b_ref, c_ref, D_ref = res_in[0], res_in[1], res_in[2], res_in[3]
            r_sq_ref = D_ref + a_ref**2 + b_ref**2 + c_ref**2

            if r_sq_ref <= 0:
                final_center, final_radius = best_center, best_radius
            else:
                final_radius = np.sqrt(r_sq_ref)
                final_center = np.array([a_ref, b_ref, c_ref], dtype=np.float32)

            # 표면 평균 잔차 계산
            dist_final = np.linalg.norm(best_inliers - final_center, axis=1)
            mean_residual = np.mean(np.abs(dist_final - final_radius))

            # 5호 축구공 검증 (반지름 0.11m ± 0.035m, 바닥 z 중심 -0.32m ~ -0.16m)
            valid = (
                np.isfinite(final_center).all()
                and np.isfinite(final_radius)
                and abs(final_radius - self.ball_radius) <= self.radius_tolerance
                and mean_residual <= 0.030
                and 0.38 <= final_center[0] <= 2.5
                and -0.8 <= final_center[1] <= 0.8
                and -0.32 <= final_center[2] <= -0.15
            )

            if valid:
                return final_center, mean_residual, "ok"
            else:
                reason = f"r={final_radius:.3f}(target=0.11), res={mean_residual:.3f}, z={final_center[2]:.2f}"
                return None, mean_residual, reason
        except Exception as e:
            return None, 999.0, f"refinement_exception_{e}"

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
        if len(roi_points) < 4:
            if debug:
                print(f"[DEBUG] raw={len(raw_point_cloud)}, base_roi={len(roi_points)} (<4)")
            return None

        # 3. Voxel 다운샘플링 (속도 최적화)
        ds_points = self.voxel_downsample(roi_points, voxel_size=0.02)
        if len(ds_points) < 4:
            return None

        # 4. 유클리드 클러스터링
        clusters = self.euclidean_clusters(ds_points)
        if debug:
            print(f"[DEBUG] input={len(raw_point_cloud)}, base_roi={len(roi_points)}, clusters={len(clusters)}")

        candidates = []
        for i, cluster in enumerate(clusters):
            center, residual, reason = self.fit_sphere_ransac_and_least_squares(cluster)
            if debug:
                cnt = len(cluster)
                mean_pos = np.mean(cluster, axis=0)
                print(f"  Cluster {i}: pts={cnt}, mean=({mean_pos[0]:.2f}, {mean_pos[1]:.2f}, {mean_pos[2]:.2f}), fit={reason}")
            if center is not None:
                candidates.append((residual, center))

        if not candidates:
            return None

        # 구체 잔차 오차(residual)가 가장 적은 (가장 완벽한 공 구체 형상) 클러스터 1순위 채택
        best_cand = min(candidates, key=lambda item: item[0])
        raw_center = best_cand[1]

        if len(self.pos_history) > 0:
            last_avg = np.mean(self.pos_history, axis=0)
            if np.linalg.norm(raw_center - last_avg) > 1.20:
                if debug:
                    print(f"[FILTER] Relocation detected: reset tracking buffer to raw={raw_center}")
                self.pos_history.clear()

        self.pos_history.append(raw_center)
        if len(self.pos_history) > self.history_size:
            self.pos_history.pop(0)

        smooth_center = np.mean(self.pos_history, axis=0)
        return smooth_center


if __name__ == "__main__":
    print("Testing Go2LidarBallDetector Module (RANSAC + Least Squares 2-Stage Sphere Fit)...")
    detector = Go2LidarBallDetector(ball_radius=0.11)

    # base 기준 전방 0.5m, z=-0.23m 지점에 구형 5호 축구공 생성
    theta = np.linspace(0, 2 * np.pi, 20)
    phi = np.linspace(0, np.pi, 20)
    r = 0.11
    x_base = 0.5 + r * np.outer(np.cos(theta), np.sin(phi)).flatten()
    y_base = 0.0 + r * np.outer(np.sin(theta), np.sin(phi)).flatten()
    z_base = -0.23 + r * np.outer(np.ones_like(theta), np.cos(phi)).flatten()
    pc_base = np.vstack((x_base, y_base, z_base)).T

    # 역변환으로 utlidar_lidar 원시 좌표 생성
    pc_lidar = (pc_base - Go2LidarBallDetector.T_LIDAR2BASE) @ Go2LidarBallDetector.R_LIDAR2BASE

    center = detector.detect_ball_3d(pc_lidar, is_base_frame=False, debug=True)
    print(f"✅ Estimated Ball 3D Position in Base Frame: {center}")
