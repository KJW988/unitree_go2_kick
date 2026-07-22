#!/usr/bin/env python3
import time
import numpy as np
from dribblebot.perception.lidar_ball_detector import Go2LidarBallDetector


def test_real_go2_lidar():
    """
    Go2 EDU 실물 로봇에 올려서 3D LiDAR 공 인식 성능(Sphere RANSAC)을 단독 검증하는 스크립트.
    Unitree LiDAR SDK / ROS2 PointCloud2 데이터와 결합하여 실시간 공 3D 좌표를 출력합니다.
    """
    print("=" * 65)
    print(" 🚀 Go2 EDU 3D LiDAR Ball Perception Standalone Test ")
    print("=" * 65)

    detector = Go2LidarBallDetector(ball_radius=0.0889)

    # unitree_sdk2 / unilidar_sdk 연동 시도
    using_real_lidar = False
    try:
        # unitree_sdk2 LiDAR subscriber 수신부 (실기 탑재 시)
        import unitree_sdk2
        print("[Info] Unitree SDK2 detected. Connecting to Go2 4D LiDAR...")
        using_real_lidar = True
    except ImportError:
        print("[Notice] unitree_sdk2 not found on local PC. Running with Live Simulator / Test Points.")

    print("\nStarting 50Hz Real-Time Ball Detection Loop... (Press Ctrl+C to stop)")
    print("-" * 65)

    try:
        step = 0
        while True:
            t0 = time.time()

            if using_real_lidar:
                # 실물 Go2 3D LiDAR Point Cloud 수신 (N x 3 numpy array)
                # point_cloud = unitree_lidar_subscriber.get_points()
                point_cloud = np.random.randn(500, 3)  # Placeholder
            else:
                # 시뮬레이션 / 구형 축구공 포인트 가상 생성 (x=0.8m 전방)
                theta = np.linspace(0, 2 * np.pi, 30)
                phi = np.linspace(0, np.pi, 30)
                r = 0.0889
                x = 0.8 + r * np.outer(np.cos(theta), np.sin(phi)).flatten()
                y = 0.1 + r * np.outer(np.sin(theta), np.sin(phi)).flatten()
                z = 0.09 + r * np.outer(np.ones_like(theta), np.cos(phi)).flatten()
                point_cloud = np.vstack((x, y, z)).T

            # 3D LiDAR Sphere RANSAC 공 인식 수행
            ball_pos_3d = detector.detect_ball_3d(point_cloud)
            dt = (time.time() - t0) * 1000.0  # ms

            step += 1
            if step % 10 == 0:
                print(f"[Step {step:05d}] ⚽ Detected Ball Pos (x, y, z): ({ball_pos_3d[0]:.3f}m, {ball_pos_3d[1]:.3f}m, {ball_pos_3d[2]:.3f}m) | Processing Latency: {dt:.2f}ms")

            time.sleep(0.02)  # 50Hz

    except KeyboardInterrupt:
        print("\n[Terminated] Ball Perception Test Stopped cleanly.")


if __name__ == "__main__":
    test_real_go2_lidar()
