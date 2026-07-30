#!/usr/bin/env python3
"""MCF staging의 좌표/방향/정지 gate를 DDS 없이 검증한다."""
from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

import stage_go2_mcf_ball_tag_webrtc as stage
import watch_go2_physical_remote_dds as watcher


def pose(x: float, y: float, yaw: float, elapsed: float = 1.0) -> dict[str, float]:
    return {"x": x, "y": y, "z": 0.31, "yaw_rad": yaw, "elapsed_s": elapsed}


class StageGeometryTest(unittest.TestCase):
    def test_static_baseline_preserves_pi_wrap(self) -> None:
        samples = [pose(0.0, 0.0, value) for value in (3.13, -3.13, 3.12, -3.12, 3.14)]
        baseline, planar_span, yaw_span = stage.static_baseline(samples)
        self.assertIsNotNone(baseline)
        assert baseline is not None
        self.assertLess(abs(stage.angle_distance(baseline["yaw_rad"], math.pi)), 0.02)
        self.assertEqual(planar_span, 0.0)
        self.assertLess(yaw_span, 0.05)

    def test_command_progress_rejects_opposite_direction(self) -> None:
        baseline = pose(0.0, 0.0, 0.0)
        self.assertGreater(stage.commanded_yaw_progress_rad(0.2, baseline, pose(0.0, 0.0, -0.1)), 0.0)
        self.assertLess(stage.commanded_yaw_progress_rad(0.2, baseline, pose(0.0, 0.0, 0.1)), 0.0)
        self.assertGreater(stage.commanded_lateral_progress_m(0.2, baseline, pose(0.0, -0.1, 0.0)), 0.0)
        self.assertLess(stage.commanded_lateral_progress_m(0.2, baseline, pose(0.0, 0.1, 0.0)), 0.0)

    def test_camera_bearing_error_uses_measured_yaw_response_sign(self) -> None:
        lane = stage.FrLaneTemplate(11, 0.70, 0.10, 0.22, 0.0, 0.03, 0.03)
        args = SimpleNamespace(
            lane_axis_bearing_rad=0.0,
            ball_bearing_tolerance_rad=0.03,
            target_bearing_tolerance_rad=0.03,
            joystick_magnitude=0.2,
            camera_stage_entry_slack_m=0.04,
        )
        # 실물 response에서 +rx는 camera bearing을 감소시킨다. 따라서 observed-desired가
        # 양수면 +rx, 음수면 -rx여야 하며 Tag/ball-only가 같은 부호 계약을 사용해야 한다.
        tag_perception = stage.Perception(
            0.90, 0.5, 0.26, 0.10, 1.0, True, 0.01, 0.0, 0.95, 0.90, 1.0,
        )
        reason, _, _, rx = stage.action_for(tag_perception, args, lane, 1.0, True)
        self.assertEqual(reason, "turn_to_tag_ray")
        self.assertGreater(rx, 0.0)
        ball_perception = stage.Perception(
            0.90, 0.5, 0.10, None, None, False, 0.01, 0.0, 0.95, 0.90, 1.0,
        )
        reason, _, _, rx = stage.action_for(ball_perception, args, lane, 1.0, False)
        self.assertEqual(reason, "turn_to_ball_lane")
        self.assertLess(rx, 0.0)

    def test_walk_preflight_rejects_nonzero_yaw_speed(self) -> None:
        state = {
            "mode": 0,
            "progress": 0,
            "body_height": 0.31,
            "velocity": [0.0, 0.0, 0.0],
            "yaw_speed": 0.2,
        }
        ok, reason = stage.state_is_safe_to_walk(state)
        self.assertFalse(ok)
        self.assertEqual(reason, "robot_already_turning")

    def test_motion_settle_rejects_yaw_drift(self) -> None:
        states = [
            {"elapsed_s": value, "velocity": [0.0, 0.0, 0.0], "yaw_speed": 0.0}
            for value in (1.0, 1.2, 1.4, 1.6)
        ]
        poses = [pose(0.0, 0.0, yaw, elapsed) for yaw, elapsed in zip(
            (0.0, 0.1, 0.2, 0.3), (1.0, 1.2, 1.4, 1.6),
        )]
        result = stage.motion_settle_snapshot(
            states, poses, window_s=0.7, max_planar_speed_m_s=0.08,
            max_yaw_speed_rad_s=0.12, max_odom_span_m=0.025,
            max_odom_yaw_span_rad=0.04,
        )
        self.assertFalse(result["ok"])
        self.assertGreater(result["odom_yaw_span_rad"], 0.04)

    def test_final_gap_uses_foot_radius_and_full_se2(self) -> None:
        args = SimpleNamespace(
            use_direct_fr_kinematics=True,
            camera_body_yaw_rad=0.0,
            camera_body_lateral_m=0.0,
            ball_radius_m=0.11,
            final_fr_to_ball_target_surface_gap_m=0.15,
            final_gait_to_kick_clearance_m=0.11,
            camera_to_fr_forward_m=-0.15,
            fr_to_ball_forward_m=0.18,
            final_fr_to_ball_min_surface_gap_m=0.10,
            final_fr_gap_tolerance_m=0.02,
            final_fr_max_lateral_error_m=0.05,
            final_fr_max_foot_speed_m_s=0.05,
        )
        fr = stage.DirectFrKinematics(
            1.0, 10, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
            (0.202, -0.1, -0.3), (0.0, 0.0, 0.0),
            (0.2, -0.1, -0.3), (0.0, 0.0, 0.0), 0.022,
        )
        ball_range = math.hypot(0.66, 0.1)
        perception = stage.Perception(
            ball_range, 0.5, math.atan2(0.1, 0.66), None, None, False,
            0.01, 0.0, ball_range, 0.66, 1.0,
        )
        start = pose(0.0, 0.0, 0.0)
        dock = stage.final_dock_plan(perception, args, fr, 0.335, start)
        self.assertAlmostEqual(dock["observed_ball_body_xy_m"][1], -0.1, places=6)
        self.assertAlmostEqual(dock["desired_final_fr_to_ball_surface_gap_m"], 0.15)
        final = pose(0.50, 0.0, 0.0)
        gap = stage.settled_fr_ball_gap(dock, final, fr, args)
        self.assertAlmostEqual(gap["estimated_fr_to_ball_forward_surface_gap_m"], 0.163, places=3)
        self.assertTrue(gap["gap_ready"])
        self.assertTrue(gap["lateral_ready"])

    def test_urdf_collision_center_is_offset_from_foot_origin(self) -> None:
        position, speed = watcher.fr_foot_kinematics_body(
            (0.0, 0.7, -1.4), (0.0, 0.0, 0.0),
        )
        center, center_speed = watcher.fr_foot_collision_kinematics_body(
            (0.0, 0.7, -1.4), (0.0, 0.0, 0.0), position, speed,
        )
        self.assertAlmostEqual(math.dist(position, center), 0.002, places=7)
        self.assertEqual(center_speed, speed)


if __name__ == "__main__":
    unittest.main()
