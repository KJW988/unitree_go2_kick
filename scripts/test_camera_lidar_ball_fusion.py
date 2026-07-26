#!/usr/bin/env python3
"""YOLO bbox + Depth/LiDAR base-frame fusion core의 순수 단위 테스트."""

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from dribblebot.perception.camera_lidar_ball_fusion import (
    CameraModel, FusedBallStaleGate, FusionConfig, ImageBallDetection,
    fuse_image_ball_with_range,
)


class CameraLidarFusionTest(unittest.TestCase):
    def setUp(self):
        self.model = CameraModel(
            fx=100.0, fy=100.0, cx=50.0, cy=50.0,
            rotation_base_from_camera=((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)),
            translation_base_from_camera_m=(0.0, 0.0, 0.30),
        )
        self.detection = ImageBallDetection(42.0, 42.0, 58.0, 58.0, 0.90, 10.0)
        self.points = np.array(((1.00, 0.00, 0.30), (1.02, 0.01, 0.29), (0.98, -0.01, 0.31), (2.0, 0.0, 0.0)))

    def test_depth_lidar_agree_and_yield_base_point(self):
        observation = fuse_image_ball_with_range(self.model, self.detection, self.points, depth_range_m=1.0)
        self.assertIsNotNone(observation)
        assert observation is not None
        np.testing.assert_allclose(observation.ball_base_xyz, (1.0, 0.0, 0.30), atol=0.03)
        self.assertEqual(observation.range_source, "depth+lidar")

    def test_depth_lidar_disagreement_fails_closed(self):
        self.assertIsNone(fuse_image_ball_with_range(self.model, self.detection, self.points, depth_range_m=1.6))

    def test_lidar_only_and_stale_jump_gates(self):
        observation = fuse_image_ball_with_range(
            self.model, self.detection, self.points, config=FusionConfig(min_confidence=0.2)
        )
        self.assertIsNotNone(observation)
        assert observation is not None
        gate = FusedBallStaleGate(FusionConfig(min_confidence=0.2, max_age_s=0.2, max_jump_m=0.1))
        self.assertIsNotNone(gate.update(observation))
        self.assertIsNone(gate.current(10.21))
        jumped = ImageBallDetection(42.0, 42.0, 58.0, 58.0, 0.9, 10.1)
        far_points = self.points + np.array((0.5, 0.0, 0.0))
        self.assertIsNone(gate.update(fuse_image_ball_with_range(
            self.model, jumped, far_points, config=FusionConfig(min_confidence=0.2)
        )))


if __name__ == "__main__":
    unittest.main(verbosity=2)
