#!/usr/bin/env python3
"""합성 base-frame point cloud로 검증 전용 LiDAR detector를 시험한다."""

import sys
import unittest
from pathlib import Path

# 스크립트를 저장소 밖의 Foxy shell에서 직접 실행해도 project package를 찾는다.
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from dribblebot.perception.temporal_lidar_ball_detector import StaticTemporalBallDetector
from dribblebot.perception.validated_lidar_ball_detector import (
    LidarBallDetectorConfig,
    StaleDetectionGate,
    ValidatedLidarBallDetector,
)


def _ground(rng, count=1800):
    xy = rng.uniform((0.20, -1.0), (2.6, 1.0), size=(count, 2))
    z = -0.32 + rng.normal(0.0, 0.002, size=(count, 1))
    return np.column_stack((xy, z))


def _ball(rng, center, count=360):
    directions = rng.normal(size=(count, 3))
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    # 바닥 근처의 가려진 하부 표면은 제외해 실제 지면 위 공을 흉내 낸다.
    directions = directions[directions[:, 2] > -0.72]
    return np.asarray(center) + 0.11 * directions + rng.normal(0.0, 0.0025, size=directions.shape)


class ValidatedLidarBallDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = ValidatedLidarBallDetector(LidarBallDetectorConfig(
            min_confidence=0.30,
            min_cluster_points=5,
        ))

    def test_ground_removal_cluster_and_sphere_fit_recovers_ball_center(self):
        rng = np.random.default_rng(71)
        truth = np.array((1.00, -0.18, -0.21))
        clutter = np.column_stack((
            rng.uniform(1.55, 1.75, size=90),
            rng.uniform(0.35, 0.43, size=90),
            rng.uniform(-0.30, -0.02, size=90),
        ))
        points = np.vstack((_ground(rng), _ball(rng, truth), clutter))
        detection = self.detector.detect(points, frame_id="base_link", stamp_s=10.0)
        self.assertIsNotNone(detection)
        assert detection is not None
        np.testing.assert_allclose(detection.center_base_xyz, truth, atol=0.018)
        self.assertAlmostEqual(detection.radius_m, 0.11, delta=0.012)
        self.assertGreaterEqual(detection.confidence, 0.30)

    def test_empty_ground_and_box_do_not_create_ball_detection(self):
        rng = np.random.default_rng(99)
        box = np.column_stack((
            rng.uniform(0.80, 0.98, size=180),
            rng.uniform(-0.55, -0.37, size=180),
            rng.uniform(-0.30, -0.05, size=180),
        ))
        points = np.vstack((_ground(rng), box))
        self.assertIsNone(self.detector.detect(points, frame_id="base_link"))

    def test_non_base_frame_requires_measured_transform(self):
        with self.assertRaisesRegex(ValueError, "no measured sensor_to_base"):
            self.detector.detect(np.zeros((8, 3)), frame_id="utlidar_lidar")

    def test_static_temporal_window_recovers_sparse_ball_surface(self):
        rng = np.random.default_rng(121)
        truth = np.array((1.00, 0.05, -0.21))
        temporal = StaticTemporalBallDetector(window_frames=12)
        evaluation = None
        for frame in range(12):
            sparse_surface = _ball(rng, truth, count=28)[:16]
            evaluation = temporal.update(
                np.vstack((_ground(rng, count=220), sparse_surface)),
                frame_id="base_link", stamp_s=float(frame),
            )
        self.assertIsNotNone(evaluation)
        assert evaluation is not None
        self.assertIsNotNone(evaluation.detection)
        assert evaluation.detection is not None
        np.testing.assert_allclose(evaluation.detection.center_base_xyz, truth, atol=0.025)

    def test_stale_gate_fails_closed(self):
        rng = np.random.default_rng(8)
        detection = self.detector.detect(
            np.vstack((_ground(rng), _ball(rng, (0.8, 0.0, -0.21)))),
            stamp_s=5.0,
        )
        self.assertIsNotNone(detection)
        gate = StaleDetectionGate(max_age_s=0.20)
        gate.update(detection)
        self.assertIsNotNone(gate.current(5.19))
        self.assertIsNone(gate.current(5.21))


if __name__ == "__main__":
    unittest.main(verbosity=2)
