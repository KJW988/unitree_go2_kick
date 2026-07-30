#!/usr/bin/env python3
"""D435i 공/AprilTag와 LiDAR odometry로 Go2를 camera-visible staging 위치에만 둔다.

LowCmd, MotionSwitcher ownership, obstacle setting은 이 프로그램의 범위 밖이다. 기본
``CAMERA_STAGING_READY``는 camera-visible 준비 상태일 뿐이다. 다만 명시적
``--use-direct-fr-kinematics``에서는 direct DDS LowState+URDF FK의 현재 FR 위치와 고정
camera body 위치를 함께 사용해, 정지 후 FR→공 표면 gap까지 통과한 경우에만
``FINAL_DOCKING_READY``를 낸다.

실행은 연속 보행이 아닌 ``짧은 virtual joystick pulse -> neutral 3회 -> 재관측``이다.
각 cycle에서 D435i state freshness, ball/Tag geometry, MCF stand state, LiDAR odometry,
start-pose 기준 travel hard limit을 모두 검사한다. 이 firmware의 WebRTC subscriber는
physical remote input을 전달하지 않으므로, 별도 direct-DDS watchdog의 fresh heartbeat와
input proof를 실행 전 요구한다. watcher가 stage의 동일 virtual packet echo만 좁은 time
window에서 제외한 뒤 다른 remote input을 관측하면, 다음 packet 전에 neutralize하고 이번
process를 종료한다. SDK callback에는 DDS writer identity가 없어 같은 값의 동시 physical input은
구분 불가하며 physical remote/E-stop은 계속 operator의 1차 안전 수단이다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
``examples/go2/data_channel/obstacles_avoid/obstacles_avoid.py``의 App-equivalent
``rt/wirelesscontroller`` payload/50 Hz 전송을 채택했다. 원 예제의 키보드 연속 drive와
obstacle-toggle은 채택하지 않고, 이 프로젝트의 D435i/LiDAR gate와 pulse hard limits를
추가했다.

기본은 계속 requested AprilTag 필수다. ``--allow-tagless-ball-kick``을 명시한 경우에만
Tag가 처음부터 없을 때 captured FR lane의 camera→ball bearing과 D435i floor-plane range로
대체한다. 이 fallback은 공을 FR 앞에 두지만 kick target 방향은 검증하지 않는다.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import statistics
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


JOYSTICK_MAGNITUDE = 0.20
COMMAND_RATE_HZ = 50.0
# 이 실물에서는 0.50초마다 neutral로 끊으면 gait initiation만 하고 1 pulse당
# 약 1~2 cm만 이동했다. calibration에서 검증된 2.0초 상한을 쓰되, 아래의
# odometry progress gate가 목표 거리에서 pulse 도중 먼저 neutralize한다.
FORWARD_PULSE_S = 2.00
MAX_FORWARD_PULSE_TRAVEL_M = 0.15
FINAL_DOCK_MAX_M = 0.85
FINAL_DOCK_MAX_DURATION_S = 6.0
# 실물에서 FR→ball 약 0.20m까지 gait를 유지하면 마지막 step이 공을 먼저 건드렸다.
# 보행은 FR→ball 약 0.29m에서 끝내고 강화된 1.3x FR kick이 마지막 약 0.11m를
# 담당하도록 nominal kick 거리 0.18m에 0.11m clearance를 남긴다.
FINAL_GAIT_TO_KICK_CLEARANCE_M = 0.11
FINAL_SETTLE_TIMEOUT_S = 2.0
FINAL_SETTLE_WINDOW_S = 0.40
FINAL_SETTLE_MAX_PLANAR_SPEED_M_S = 0.08
FINAL_SETTLE_MAX_YAW_SPEED_RAD_S = 0.12
FINAL_SETTLE_MAX_ODOM_SPAN_M = 0.025
FINAL_SETTLE_MAX_ODOM_YAW_SPAN_RAD = 0.04
FR_KINEMATICS_MAX_AGE_S = 0.15
FINAL_FR_MAX_FOOT_SPEED_M_S = 0.05
BALL_RADIUS_M = 0.11
FR_FOOT_COLLISION_RADIUS_M = 0.022
FINAL_FR_TO_BALL_MIN_SURFACE_GAP_M = 0.10
FINAL_FR_TO_BALL_TARGET_SURFACE_GAP_M = 0.15
FINAL_FR_GAP_TOLERANCE_M = 0.02
FINAL_FR_MAX_LATERAL_ERROR_M = 0.05
# 0.80 s lateral pulse도 이 실물에서 gait initiation만 반복하고 실제 횟이동은
# 4.5 mm에 그쳤다. magnitude는 그대로 두고 2.0 s 상한을 주되, LiDAR odometry가
# 목표 횟변위를 연속 확인하면 상한 전에 neutralize한다.
TURN_PULSE_S = 0.80
LATERAL_PULSE_S = 2.00
MAX_LATERAL_PULSE_TRAVEL_M = 0.08
# ball-only 실물에서 0.0317 rad 오차는 0.8 s yaw gait의 최소 해상도보다 작아
# 회전이 오히려 공을 FOV 밖으로 밀었다. strict kick gate는 그대로 두고
# high-level yaw action만 0.04 rad 이하에서 억제한다.
BALL_ONLY_YAW_ACTION_TOLERANCE_RAD = 0.04
MIN_ODOM_STOP_ACTIVE_S = 0.60
ODOM_STOP_CONFIRM_SAMPLES = 3
# camera staging까지 수 cm만 남으면 작은 pulse가 gait initiation을 반복한다. 이 구간은
# 정렬을 다시 확인한 뒤 odometry-bounded continuous final dock으로 넘긴다.
CAMERA_STAGE_ENTRY_SLACK_M = 0.04
NEUTRAL_PACKET_COUNT = 3
STATE_SETTLE_S = 1.0
REOBSERVE_SETTLE_S = 0.50
MAX_STATIC_BASELINE_SPAN_M = 0.04
MAX_STATIC_BASELINE_YAW_SPAN_RAD = 0.08
MAX_ODOM_STALE_S = 0.50
MAX_PERCEPTION_AGE_S = 0.35
MAX_BALL_DETECTION_AGE_S = 0.50
OBSERVATION_TIMEOUT_S = 5.0
MIN_BALL_CONFIDENCE = 0.015
MIN_BALL_DIAMETER_CONSISTENCY_RATIO = 0.45
MAX_BALL_DIAMETER_CONSISTENCY_RATIO = 2.20
STAGE_RANGE_MIN_M = 0.65
STAGE_RANGE_MAX_M = 0.85
MAX_TRAVEL_M = 0.35
MAX_TOTAL_TRAVEL_M = 1.20
MAX_CYCLES = 5
DIRECT_REMOTE_MAX_STATUS_AGE_S = 0.35
DIRECT_REMOTE_HOLD_S = 0.60
DIRECT_REMOTE_UDP_PORT = 18181
DEFAULT_VIRTUAL_ECHO_WINDOW = Path("hardware_measurements/go2_stage_virtual_joystick_echo_window.json")
VIRTUAL_ECHO_GRACE_S = 0.15
VIRTUAL_ECHO_REFRESH_S = 0.20
VIRTUAL_ECHO_LEASE_S = 0.50
LATERAL_MIN_IMPROVEMENT_RAD = 0.01
MAX_LATERAL_PROBE_ATTEMPTS = 3
CONFIRMATION = "MCF_CAMERA_STAGE_CLEAR_FLOOR_ESTOP_READY"
# 2026-07-28/29 실물 odometry 응답: +rx는 yaw를 감소시키고, +lx는 body lateral
# 좌표를 감소시켰다. 목표 도달 판정은 이 signed 응답과 같은 방향만 인정한다.
YAW_ODOM_RESPONSE_SIGN_PER_RX = -1.0
LATERAL_ODOM_RESPONSE_SIGN_PER_LX = -1.0
VIRTUAL_ECHO_PROTOCOL_VERSION = 2


def unwrap_data(message: Any) -> dict[str, Any] | None:
    if not isinstance(message, dict):
        return None
    data = message.get("data", message)
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except ValueError:
            return None
    return data if isinstance(data, dict) else None


def angle_distance(first: float, second: float) -> float:
    return math.atan2(math.sin(first - second), math.cos(first - second))


def circular_mean(angles: list[float]) -> float:
    """-pi/pi 경계를 보존하는 원형 평균이다."""
    if not angles:
        raise ValueError("angles가 비었습니다")
    return math.atan2(
        sum(math.sin(value) for value in angles),
        sum(math.cos(value) for value in angles),
    )


def circular_span(angles: list[float], center: float | None = None) -> float:
    """원형 중심으로부터의 최대 양방향 yaw 폭을 반환한다."""
    if not angles:
        return float("inf")
    anchor = circular_mean(angles) if center is None else center
    return 2.0 * max(abs(angle_distance(value, anchor)) for value in angles)


def body_xy_to_world(pose: dict[str, float], point_xy: tuple[float, float]) -> tuple[float, float]:
    cosine, sine = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
    return (
        pose["x"] + cosine * point_xy[0] - sine * point_xy[1],
        pose["y"] + sine * point_xy[0] + cosine * point_xy[1],
    )


def world_xy_to_body(pose: dict[str, float], point_xy: tuple[float, float]) -> tuple[float, float]:
    dx, dy = point_xy[0] - pose["x"], point_xy[1] - pose["y"]
    cosine, sine = math.cos(pose["yaw_rad"]), math.sin(pose["yaw_rad"])
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


@dataclass(frozen=True)
class Perception:
    ball_range_m: float
    ball_confidence: float
    ball_bearing_rad: float
    target_bearing_rad: float | None
    target_distance_m: float | None
    tag_visible: bool
    age_s: float
    ball_detection_age_s: float
    ball_ground_range_m: float
    ball_ground_forward_m: float
    ball_diameter_consistency_ratio: float


@dataclass(frozen=True)
class FrLaneTemplate:
    """실제 FR toe→ball→Tag 선상 staging spot에서 read-only로 얻은 camera 목표값."""

    tag_id: int
    desired_ball_range_m: float
    range_tolerance_m: float
    desired_ball_bearing_rad: float
    desired_target_bearing_rad: float
    ball_bearing_tolerance_rad: float
    target_bearing_tolerance_rad: float


def point3(value: Any) -> tuple[float, float, float] | None:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        result = tuple(float(component) for component in value)
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(component) for component in result) else None


def fetch_perception_with_reason(
    url: str, tag_id: int, timeout_s: float, allow_tagless_ball_kick: bool = False,
) -> tuple[Perception | None, str]:
    """stream geometry와 누락 원인을 읽는다. base/FR 좌표로 변환하지 않는다."""
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (OSError, ValueError, urllib.error.URLError):
        return None, "perception_http_unavailable"
    if not isinstance(payload, dict) or not payload.get("ready"):
        return None, "perception_not_ready"
    ball = payload.get("ball")
    target = payload.get("target_line")
    floor_plane = payload.get("floor_plane_camera")
    if not isinstance(ball, dict):
        return None, "ball_not_detected"
    try:
        target_matches = isinstance(target, dict) and int(target.get("tag_id", -1)) == tag_id
    except (TypeError, ValueError):
        target_matches = False
    if not target_matches and not allow_tagless_ball_kick:
        return None, (
            "tag_or_target_line_missing" if not isinstance(target, dict)
            else "requested_tag_missing"
        )
    ball_ground = point3(ball.get("ground_camera_xyz_m"))
    target_unit = point3(target.get("unit_camera_xyz")) if target_matches else None
    plane_normal = point3(floor_plane.get("normal")) if isinstance(floor_plane, dict) else None
    try:
        confidence = float(ball["confidence"])
        ball_range = float(ball["depth_range_m"])
        target_distance = float(target["distance_m"]) if target_matches else None
        stamp = float(payload["stamp_monotonic_s"])
        detection_age = float(ball["detection_age_s"])
        diameter_consistency = float(ball["diameter_consistency_ratio"])
        plane_offset = float(floor_plane["offset"])
        plane_inliers = int(floor_plane["inlier_count"])
    except (KeyError, TypeError, ValueError):
        return None, "perception_fields_invalid"
    if (
        ball_ground is None
        or plane_normal is None
        or not all(math.isfinite(value) for value in (
            confidence, ball_range, stamp, detection_age, diameter_consistency, plane_offset,
        ))
        or (target_matches and (target_unit is None or target_distance is None or not math.isfinite(target_distance)))
        or ball_range <= 0.0
        or (target_distance is not None and target_distance < 0.5)
        or detection_age < 0.0
        or plane_inliers < 80
        or plane_normal[1] < 0.65
        or not 0.15 <= abs(plane_offset) <= 0.80
    ):
        return None, "perception_geometry_invalid"
    camera_ground = tuple(-plane_offset * component for component in plane_normal)
    ball_ground_range = math.sqrt(sum(
        (ball_ground[index] - camera_ground[index]) ** 2 for index in range(3)
    ))
    # Tag가 없으면 optical +z를 floor에 투영한 방향을 camera-forward로 사용한다.
    # camera-to-base yaw가 실측되지 않았으므로 이 fallback은 목표물 방향을 보장하지 않고,
    # 명시 opt-in에서 FR에 공을 놓는 용도로만 허용한다.
    forward_unit = target_unit if target_unit is not None else (0.0, 0.0, 1.0)
    forward_normal_component = sum(
        forward_unit[index] * plane_normal[index] for index in range(3)
    )
    forward_ground = tuple(
        forward_unit[index] - forward_normal_component * plane_normal[index]
        for index in range(3)
    )
    forward_ground_norm = math.sqrt(sum(component * component for component in forward_ground))
    if forward_ground_norm > 0.0:
        forward_ground = tuple(component / forward_ground_norm for component in forward_ground)
    camera_to_ball_ground = tuple(
        ball_ground[index] - camera_ground[index] for index in range(3)
    )
    ball_ground_forward = sum(
        camera_to_ball_ground[index] * forward_ground[index] for index in range(3)
    )
    if (
        not math.isfinite(ball_ground_range)
        or not math.isfinite(ball_ground_forward)
        or ball_ground_range <= 0.0
        or forward_ground_norm <= 1e-6
        or ball_ground_forward <= 0.0
    ):
        return None, "ball_ground_range_invalid"
    # RealSense optical frame: +x image-right, +z forward. y는 floor plane vertical 성분이다.
    return Perception(
        ball_range_m=ball_range,
        ball_confidence=confidence,
        ball_bearing_rad=math.atan2(ball_ground[0], ball_ground[2]),
        target_bearing_rad=(
            None if target_unit is None else math.atan2(target_unit[0], target_unit[2])
        ),
        target_distance_m=target_distance,
        tag_visible=target_unit is not None,
        age_s=time.monotonic() - stamp,
        ball_detection_age_s=detection_age,
        ball_ground_range_m=ball_ground_range,
        ball_ground_forward_m=ball_ground_forward,
        ball_diameter_consistency_ratio=diameter_consistency,
    ), "perception_sample_valid" if target_unit is not None else "tagless_ball_sample_valid"


def fetch_perception(url: str, tag_id: int, timeout_s: float) -> Perception | None:
    """기존 read-only caller용 호환 wrapper다."""
    return fetch_perception_with_reason(url, tag_id, timeout_s)[0]


async def stable_perception(args: argparse.Namespace) -> tuple[Perception | None, str]:
    """짧은 detector frame drop은 bounded retry하고 stale/unstable state는 계속 거부한다."""
    samples: list[Perception] = []
    deadline = time.monotonic() + args.observation_timeout_s
    latest_reason = "perception_missing"
    while len(samples) < args.observation_count and time.monotonic() < deadline:
        sample, sample_reason = fetch_perception_with_reason(
            args.perception_url, args.tag_id, args.http_timeout_s,
            args.allow_tagless_ball_kick,
        )
        if sample is None:
            latest_reason = sample_reason
            await asyncio.sleep(args.observation_interval_s)
            continue
        if sample.age_s > args.perception_max_age_s:
            latest_reason = "perception_stale"
            await asyncio.sleep(args.observation_interval_s)
            continue
        if sample.ball_confidence < args.min_ball_confidence:
            latest_reason = "ball_confidence_low"
            await asyncio.sleep(args.observation_interval_s)
            continue
        if sample.ball_detection_age_s > args.ball_detection_max_age_s:
            latest_reason = "ball_detection_stale"
            await asyncio.sleep(args.observation_interval_s)
            continue
        if not (
            args.min_ball_diameter_consistency_ratio
            <= sample.ball_diameter_consistency_ratio
            <= args.max_ball_diameter_consistency_ratio
        ):
            latest_reason = "ball_metric_size_inconsistent"
            await asyncio.sleep(args.observation_interval_s)
            continue
        if not 0.30 <= sample.ball_range_m <= 3.0:
            latest_reason = "ball_depth_out_of_range"
            await asyncio.sleep(args.observation_interval_s)
            continue
        samples.append(sample)
        if len(samples) < args.observation_count:
            await asyncio.sleep(args.observation_interval_s)
    if len(samples) < args.observation_count:
        return None, latest_reason
    anchor = samples[-1]
    if max(abs(sample.ball_range_m - anchor.ball_range_m) for sample in samples) > args.max_range_jitter_m:
        return None, "ball_range_unstable"
    if max(abs(angle_distance(sample.ball_bearing_rad, anchor.ball_bearing_rad)) for sample in samples) > args.max_bearing_jitter_rad:
        return None, "ball_bearing_unstable"
    if any(sample.tag_visible != anchor.tag_visible for sample in samples):
        return None, "tag_visibility_unstable"
    if anchor.tag_visible:
        assert anchor.target_bearing_rad is not None
        if max(
            abs(angle_distance(sample.target_bearing_rad, anchor.target_bearing_rad))
            for sample in samples if sample.target_bearing_rad is not None
        ) > args.max_bearing_jitter_rad:
            return None, "target_bearing_unstable"
    aggregate = Perception(
        ball_range_m=float(statistics.median(sample.ball_range_m for sample in samples)),
        ball_confidence=min(sample.ball_confidence for sample in samples),
        ball_bearing_rad=circular_mean([sample.ball_bearing_rad for sample in samples]),
        target_bearing_rad=(
            circular_mean([
                sample.target_bearing_rad for sample in samples
                if sample.target_bearing_rad is not None
            ]) if anchor.tag_visible else None
        ),
        target_distance_m=(
            float(statistics.median(
                sample.target_distance_m for sample in samples
                if sample.target_distance_m is not None
            )) if anchor.tag_visible else None
        ),
        tag_visible=anchor.tag_visible,
        age_s=max(sample.age_s for sample in samples),
        ball_detection_age_s=max(sample.ball_detection_age_s for sample in samples),
        ball_ground_range_m=float(statistics.median(
            sample.ball_ground_range_m for sample in samples
        )),
        ball_ground_forward_m=float(statistics.median(
            sample.ball_ground_forward_m for sample in samples
        )),
        ball_diameter_consistency_ratio=float(statistics.median(
            sample.ball_diameter_consistency_ratio for sample in samples
        )),
    )
    return aggregate, "perception_stable" if anchor.tag_visible else "tagless_ball_perception_stable"


def sport_state_summary(message: Any, elapsed_s: float) -> dict[str, Any]:
    data = unwrap_data(message)
    if data is None:
        return {"elapsed_s": elapsed_s, "message_type": type(message).__name__}
    velocity = data.get("velocity")
    if not isinstance(velocity, list) or len(velocity) != 3:
        velocity = None
    return {
        "elapsed_s": elapsed_s,
        "mode": data.get("mode"),
        "progress": data.get("progress"),
        "body_height": data.get("body_height"),
        "velocity": velocity,
        "yaw_speed": data.get("yaw_speed"),
    }


def pose_summary(message: Any, elapsed_s: float) -> dict[str, float] | None:
    data = unwrap_data(message)
    if data is None:
        return None
    pose = data.get("pose", data)
    if isinstance(pose, dict) and isinstance(pose.get("pose"), dict):
        pose = pose["pose"]
    if not isinstance(pose, dict):
        return None
    position = pose.get("position")
    orientation = pose.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    try:
        x, y, z = (float(position[key]) for key in ("x", "y", "z"))
        qx, qy, qz, qw = (float(orientation[key]) for key in ("x", "y", "z", "w"))
    except (KeyError, TypeError, ValueError):
        return None
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return {"elapsed_s": elapsed_s, "x": x, "y": y, "z": z, "yaw_rad": yaw}


def static_baseline(
    poses: list[dict[str, float]],
) -> tuple[dict[str, float] | None, float, float]:
    if len(poses) < 5:
        return None, float("inf"), float("inf")
    baseline = {
        key: float(statistics.median(pose[key] for pose in poses))
        for key in ("x", "y", "z")
    }
    yaws = [pose["yaw_rad"] for pose in poses]
    baseline["yaw_rad"] = circular_mean(yaws)
    span = max(math.hypot(pose["x"] - baseline["x"], pose["y"] - baseline["y"]) for pose in poses)
    return baseline, span, circular_span(yaws, baseline["yaw_rad"])


def planar_distance(baseline: dict[str, float], pose: dict[str, float]) -> float:
    return math.hypot(pose["x"] - baseline["x"], pose["y"] - baseline["y"])


def forward_progress_m(baseline: dict[str, float], pose: dict[str, float]) -> float:
    """pulse 시작 yaw로 투영한 LiDAR odometry 전진 거리다."""
    dx = pose["x"] - baseline["x"]
    dy = pose["y"] - baseline["y"]
    return dx * math.cos(baseline["yaw_rad"]) + dy * math.sin(baseline["yaw_rad"])


def lateral_progress_m(baseline: dict[str, float], pose: dict[str, float]) -> float:
    """pulse 시작 yaw로 투영한 signed LiDAR odometry 횟이동 거리다."""
    dx = pose["x"] - baseline["x"]
    dy = pose["y"] - baseline["y"]
    return -dx * math.sin(baseline["yaw_rad"]) + dy * math.cos(baseline["yaw_rad"])


def commanded_yaw_progress_rad(
    rx: float, baseline: dict[str, float], pose: dict[str, float],
) -> float:
    expected_sign = math.copysign(1.0, rx) * YAW_ODOM_RESPONSE_SIGN_PER_RX
    return expected_sign * angle_distance(pose["yaw_rad"], baseline["yaw_rad"])


def commanded_lateral_progress_m(
    lx: float, baseline: dict[str, float], pose: dict[str, float],
) -> float:
    expected_sign = math.copysign(1.0, lx) * LATERAL_ODOM_RESPONSE_SIGN_PER_LX
    return expected_sign * lateral_progress_m(baseline, pose)


def state_is_safe_to_walk(state: dict[str, Any] | None) -> tuple[bool, str]:
    if state is None:
        return False, "sportmodestate_missing"
    if state.get("mode") not in (0, None):
        return False, "mcf_mode_not_idle"
    if state.get("progress") not in (0, None):
        return False, "mcf_motion_in_progress"
    height = state.get("body_height")
    if not isinstance(height, (int, float)) or not 0.24 <= height <= 0.40:
        return False, "body_height_out_of_stand_range"
    velocity = state.get("velocity")
    if velocity is not None and max(abs(float(value)) for value in velocity) > 0.12:
        return False, "robot_already_moving"
    yaw_speed = state.get("yaw_speed")
    if not isinstance(yaw_speed, (int, float)) or not math.isfinite(float(yaw_speed)):
        return False, "yaw_speed_missing_or_invalid"
    if abs(float(yaw_speed)) > 0.12:
        return False, "robot_already_turning"
    return True, "mcf_stand_preflight_pass"


@dataclass(frozen=True)
class DirectFrKinematics:
    """direct DDS LowState와 Go2 URDF에서 계산한 현재 FR foot body pose다."""

    receipt_monotonic_s: float
    sample_count: int
    joint_q_rad: tuple[float, float, float]
    joint_dq_rad_s: tuple[float, float, float]
    foot_position_body_m: tuple[float, float, float]
    foot_speed_body_m_s: tuple[float, float, float]
    foot_collision_center_body_m: tuple[float, float, float]
    foot_collision_speed_body_m_s: tuple[float, float, float]
    foot_collision_radius_m: float


@dataclass(frozen=True)
class DirectRemoteWatchdog:
    """별도 SDK environment의 direct DDS watcher가 남긴 상태다."""

    heartbeat_monotonic_s: float
    event_count: int
    last_active_monotonic_s: float | None
    last_event: dict[str, Any] | None
    virtual_echo_protocol_version: int
    fr_foot_kinematics: DirectFrKinematics | None


class DirectRemoteLatch(asyncio.DatagramProtocol):
    """direct DDS watcher가 보내는 localhost UDP physical-input event를 즉시 저장한다."""

    def __init__(self) -> None:
        self.events: list[dict[str, Any]] = []

    def datagram_received(self, data: bytes, _address: Any) -> None:
        try:
            event = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return
        if isinstance(event, dict):
            self.events.append(event)

    def tripped(self) -> bool:
        return bool(self.events)


def load_direct_remote_watchdog(
    path: Path | None, *, max_age_s: float, hold_s: float,
    virtual_echo_window: Path | None = None,
    require_fr_kinematics: bool = False,
    fr_kinematics_max_age_s: float = FR_KINEMATICS_MAX_AGE_S,
) -> tuple[DirectRemoteWatchdog | None, str]:
    """direct DDS watchdog heartbeat와 actual physical-input proof를 fail-closed로 확인한다."""
    if path is None:
        return None, "direct_remote_watchdog_required"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        return None, "direct_remote_watchdog_unreadable:{}".format(error)
    if not isinstance(payload, dict) or payload.get("kind") != "go2_direct_dds_physical_remote_watchdog":
        return None, "direct_remote_watchdog_invalid_kind"
    try:
        heartbeat = float(payload["heartbeat_monotonic_s"])
        event_count = int(payload["physical_input_event_count"])
        last_active_value = payload.get("last_active_monotonic_s")
        last_active = None if last_active_value is None else float(last_active_value)
        echo_protocol = int(payload["virtual_echo_protocol_version"])
    except (KeyError, TypeError, ValueError):
        return None, "direct_remote_watchdog_missing_fields"
    if payload.get("ready") is not True or payload.get("motion_commands_sent") is not False:
        return None, "direct_remote_watchdog_not_read_only_ready"
    if echo_protocol != VIRTUAL_ECHO_PROTOCOL_VERSION:
        return None, "direct_remote_watchdog_echo_protocol_mismatch"
    if virtual_echo_window is not None and payload.get("virtual_echo_window") != str(virtual_echo_window):
        return None, "direct_remote_watchdog_echo_window_mismatch"
    now = time.monotonic()
    if not math.isfinite(heartbeat) or now - heartbeat > max_age_s:
        return None, "direct_remote_watchdog_stale"
    if event_count < 1 or last_active is None or not math.isfinite(last_active):
        return None, "direct_remote_input_not_proven"
    if now - last_active < hold_s:
        return None, "physical_remote_active"
    fr_kinematics: DirectFrKinematics | None = None
    fr_payload = payload.get("fr_foot_kinematics")
    if isinstance(fr_payload, dict) and fr_payload.get("valid") is True:
        try:
            fr_receipt = float(fr_payload["receipt_monotonic_s"])
            fr_sample_count = int(fr_payload["sample_count"])
            fr_joint_q = point3(fr_payload.get("joint_q_rad"))
            fr_joint_dq = point3(fr_payload.get("joint_dq_rad_s"))
            fr_position = point3(fr_payload.get("foot_position_body_m"))
            fr_speed = point3(fr_payload.get("foot_speed_body_m_s"))
            fr_collision_center = point3(fr_payload.get("foot_collision_center_body_m"))
            fr_collision_speed = point3(fr_payload.get("foot_collision_speed_body_m_s"))
            fr_collision_radius = float(fr_payload["foot_collision_radius_m"])
        except (KeyError, TypeError, ValueError):
            fr_receipt, fr_sample_count = float("nan"), 0
            fr_joint_q = fr_joint_dq = fr_position = fr_speed = None
            fr_collision_center = fr_collision_speed = None
            fr_collision_radius = float("nan")
        if (
            math.isfinite(fr_receipt)
            and fr_sample_count > 0
            and now - fr_receipt <= fr_kinematics_max_age_s
            and fr_joint_q is not None
            and fr_joint_dq is not None
            and fr_position is not None
            and fr_speed is not None
            and fr_collision_center is not None
            and fr_collision_speed is not None
            and math.isfinite(fr_collision_radius)
            and 0.015 <= fr_collision_radius <= 0.04
            and 0.05 <= fr_position[0] <= 0.35
            and -0.35 <= fr_position[1] <= 0.05
            and -0.50 <= fr_position[2] <= -0.10
        ):
            fr_kinematics = DirectFrKinematics(
                fr_receipt, fr_sample_count, fr_joint_q, fr_joint_dq,
                fr_position, fr_speed, fr_collision_center, fr_collision_speed,
                fr_collision_radius,
            )
    if require_fr_kinematics and fr_kinematics is None:
        return None, "direct_fr_kinematics_missing_stale_or_implausible"
    last_event = payload.get("last_event")
    return DirectRemoteWatchdog(
        heartbeat, event_count, last_active,
        last_event if isinstance(last_event, dict) else None,
        echo_protocol,
        fr_kinematics,
    ), \
        "direct_remote_watchdog_pass"


def load_fr_lane_template(path: Path | None, tag_id: int) -> tuple[FrLaneTemplate | None, str]:
    """camera-center가 아닌 실제 FR lane template만 stage target으로 허용한다."""
    if path is None:
        return None, "fr_lane_template_required"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        return None, "fr_lane_template_unreadable:{}".format(error)
    result = payload.get("result") if isinstance(payload, dict) else None
    template = result.get("template") if isinstance(result, dict) else None
    if payload.get("verdict") != "FR_LANE_TEMPLATE_CAPTURED" or not isinstance(template, dict):
        return None, "fr_lane_template_invalid_verdict"
    try:
        lane = FrLaneTemplate(
            tag_id=int(template["tag_id"]),
            desired_ball_range_m=float(template["desired_ball_range_m"]),
            range_tolerance_m=float(template["range_tolerance_m"]),
            desired_ball_bearing_rad=float(template["desired_ball_bearing_rad"]),
            desired_target_bearing_rad=float(template["desired_target_bearing_rad"]),
            ball_bearing_tolerance_rad=float(template["ball_bearing_tolerance_rad"]),
            target_bearing_tolerance_rad=float(template["target_bearing_tolerance_rad"]),
        )
    except (KeyError, TypeError, ValueError):
        return None, "fr_lane_template_missing_fields"
    numbers = tuple(asdict(lane).values())[1:]
    if lane.tag_id != tag_id or not all(math.isfinite(float(value)) for value in numbers):
        return None, "fr_lane_template_tag_or_number_invalid"
    if not 0.30 <= lane.desired_ball_range_m <= 2.0 or not 0.0 < lane.range_tolerance_m <= 0.25:
        return None, "fr_lane_template_range_invalid"
    if not 0.0 < lane.ball_bearing_tolerance_rad <= 0.20 or not 0.0 < lane.target_bearing_tolerance_rad <= 0.20:
        return None, "fr_lane_template_bearing_tolerance_invalid"
    return lane, "fr_lane_template_pass"


def action_for(
    perception: Perception, args: argparse.Namespace, lane: FrLaneTemplate, lateral_sign: float,
    use_tag_guidance: bool,
) -> tuple[str, float, float, float]:
    """FR lane 방향 yaw를 먼저 맞춘 뒤 lateral probe로 FR→ball lane을 보정한다.

    ball bearing을 0으로 맞추지 않는다. FR의 lateral offset 때문에 valid kick lane의 ball은
    camera image 중앙에서 벗어날 수 있다. 먼저 Tag ground ray를 template 값에 맞춰 body/FR
    방향을 kick lane과 평행하게 만든다. robot yaw는 ball/Tag bearing을 거의 함께 변화시키므로
    그 다음 남는 ``ball - Tag`` 상대 bearing 오차는 작은 ``lx`` pulse로 시험해야 한다.
    lateral 뒤 Tag ray는 다시 달라질 수 있으므로 다음 cycle에서 yaw를 재확인한다. joystick
    lateral 부호는 관측으로만 정하고 연속 이동으로 가정하지 않는다.
    """
    if not use_tag_guidance:
        errors = ball_only_lane_errors(perception, lane, args.lane_axis_bearing_rad)
        ball_tolerance = min(
            lane.ball_bearing_tolerance_rad, args.ball_bearing_tolerance_rad,
        )
        ball_action_tolerance = max(
            ball_tolerance, BALL_ONLY_YAW_ACTION_TOLERANCE_RAD,
        )
        if abs(errors["ball_error_rad"]) > ball_action_tolerance:
            # 2026-07-29 실물 response: +rx는 camera ball bearing을 감소시켰다.
            # error=(observed-desired)와 같은 rx 부호를 보내야 오차가 0으로 줄어든다.
            return (
                "turn_to_ball_lane", 0.0, 0.0,
                math.copysign(args.joystick_magnitude, errors["ball_error_rad"]),
            )
        if perception.ball_range_m < lane.desired_ball_range_m - lane.range_tolerance_m:
            return "ball_too_close_no_reverse", 0.0, 0.0, 0.0
        if perception.ball_range_m <= (
            lane.desired_ball_range_m + lane.range_tolerance_m + args.camera_stage_entry_slack_m
        ):
            return "camera_staging_ready", 0.0, 0.0, 0.0
        return "forward", 0.0, args.joystick_magnitude, 0.0

    errors = fr_lane_bearing_errors(perception, lane, args.lane_axis_bearing_rad)
    # bounded yaw pulse의 실물 해상도보다 작은 target 오차까지 매번 회전시키면
    # 공이 FOV 밖으로 나갈 수 있다. 이 값은 rough camera staging의 yaw deadband이며,
    # strict FR lane/kick 허용오차 자체를 완화하는 값은 아니다.
    target_action_tolerance = min(
        lane.target_bearing_tolerance_rad, args.target_bearing_tolerance_rad,
    )
    target_error = errors["target_error_rad"]
    if abs(target_error) > target_action_tolerance:
        # 같은 D435i bearing 오차에는 ball-only와 같은 부호를 적용한다. 실물에서
        # +rx가 camera bearing을 감소시켰으므로 error=(observed-desired)와 같은 부호다.
        return "turn_to_tag_ray", 0.0, 0.0, math.copysign(args.joystick_magnitude, target_error)
    # yaw 뒤에도 남는 ball-vs-Tag 상대 bearing은 FR toe가 공 중심 뒤 선상에 없다는 뜻이다.
    if abs(errors["relative_error_rad"]) > min(
        lane.ball_bearing_tolerance_rad, lane.target_bearing_tolerance_rad,
        args.ball_bearing_tolerance_rad,
    ):
        return "lateral_to_fr_lane", math.copysign(args.joystick_magnitude, lateral_sign), 0.0, 0.0
    if perception.ball_range_m < lane.desired_ball_range_m - lane.range_tolerance_m:
        return "ball_too_close_no_reverse", 0.0, 0.0, 0.0
    if perception.ball_range_m <= (
        lane.desired_ball_range_m + lane.range_tolerance_m + args.camera_stage_entry_slack_m
    ):
        return "camera_staging_ready", 0.0, 0.0, 0.0
    return "forward", 0.0, args.joystick_magnitude, 0.0


def fr_lane_bearing_errors(
    perception: Perception, lane: FrLaneTemplate,
    lane_axis_bearing_rad: float | None = None,
) -> dict[str, float]:
    """camera frame에서 FR toe→ball→Tag ray의 상대/절대 bearing 오차를 기록한다.

    ``lane_axis_bearing_rad``가 주어지면 template의 range/상대 bearing은 보존하되,
    ball→Tag 지면축 자체는 그 camera bearing에 평행하게 맞춘다. head mount가 robot
    전진축과 평행한 현재 실물에서는 0 rad가 body/FR과 kick lane의 평행 조건이다.
    """
    if perception.target_bearing_rad is None:
        raise ValueError("Tag-guided FR lane errors require target_bearing_rad")
    observed_relative = angle_distance(perception.ball_bearing_rad, perception.target_bearing_rad)
    desired_relative = angle_distance(lane.desired_ball_bearing_rad, lane.desired_target_bearing_rad)
    desired_target = (
        lane.desired_target_bearing_rad
        if lane_axis_bearing_rad is None else lane_axis_bearing_rad
    )
    desired_ball = angle_distance(desired_target + desired_relative, 0.0)
    return {
        "ball_error_rad": angle_distance(perception.ball_bearing_rad, desired_ball),
        "target_error_rad": angle_distance(perception.target_bearing_rad, desired_target),
        "observed_relative_bearing_rad": observed_relative,
        "desired_relative_bearing_rad": desired_relative,
        "relative_error_rad": angle_distance(observed_relative, desired_relative),
        "desired_lane_axis_bearing_rad": desired_target,
        "desired_ball_bearing_rad": desired_ball,
    }


def ball_only_lane_errors(
    perception: Perception, lane: FrLaneTemplate,
    lane_axis_bearing_rad: float | None = None,
) -> dict[str, float]:
    """Tag 없이 captured FR lane의 camera→ball bearing만 재사용한다.

    Tag가 있던 template의 ``ball - Tag`` 상대 bearing과 사용자가 지정한 lane axis를 합쳐
    FR 앞에 있어야 할 ball bearing을 얻는다. 이는 target 방향 검증이 아니며, camera mount
    yaw가 바뀌면 template를 다시 측정해야 한다.
    """
    desired_relative = angle_distance(
        lane.desired_ball_bearing_rad, lane.desired_target_bearing_rad,
    )
    desired_axis = (
        lane.desired_target_bearing_rad
        if lane_axis_bearing_rad is None else lane_axis_bearing_rad
    )
    desired_ball = angle_distance(desired_axis + desired_relative, 0.0)
    return {
        "ball_error_rad": angle_distance(perception.ball_bearing_rad, desired_ball),
        "desired_ball_bearing_rad": desired_ball,
        "desired_lane_axis_bearing_rad": desired_axis,
    }


def alignment_errors(
    perception: Perception, lane: FrLaneTemplate,
    lane_axis_bearing_rad: float | None, use_tag_guidance: bool,
) -> dict[str, float]:
    return (
        fr_lane_bearing_errors(perception, lane, lane_axis_bearing_rad)
        if use_tag_guidance
        else ball_only_lane_errors(perception, lane, lane_axis_bearing_rad)
    )


def final_dock_plan(
    perception: Perception, args: argparse.Namespace,
    fr_kinematics: DirectFrKinematics | None = None,
    camera_body_forward_m: float | None = None,
    start_pose: dict[str, float] | None = None,
) -> dict[str, Any]:
    """D435i 공 중심과 현재 FR collision sphere의 2D final dock 목표를 계산한다."""
    forward = perception.ball_ground_range_m * math.cos(perception.ball_bearing_rad)
    image_right = perception.ball_ground_range_m * math.sin(perception.ball_bearing_rad)
    camera_yaw = args.camera_body_yaw_rad
    ball_body_xy = (
        float(camera_body_forward_m or 0.0)
        + forward * math.cos(camera_yaw) + image_right * math.sin(camera_yaw),
        args.camera_body_lateral_m
        + forward * math.sin(camera_yaw) - image_right * math.cos(camera_yaw),
    )
    if args.use_direct_fr_kinematics:
        if fr_kinematics is None or camera_body_forward_m is None:
            raise RuntimeError("direct FR kinematics 또는 camera body calibration이 없습니다")
        foot_xy = fr_kinematics.foot_collision_center_body_m[:2]
        foot_radius = fr_kinematics.foot_collision_radius_m
        desired_fr_to_ball_center = foot_radius + args.ball_radius_m + (
            args.final_fr_to_ball_target_surface_gap_m
        )
        desired_ball_body_x = foot_xy[0] + desired_fr_to_ball_center
        geometry_mode = "direct_lowstate_fr_collision_sphere_2d_surface_gap"
    else:
        foot_xy = (
            float(camera_body_forward_m or 0.0) + args.camera_to_fr_forward_m,
            args.camera_body_lateral_m,
        )
        foot_radius = 0.0
        desired_fr_to_ball_center = (
            args.fr_to_ball_forward_m + args.final_gait_to_kick_clearance_m
        )
        desired_ball_body_x = foot_xy[0] + desired_fr_to_ball_center
        geometry_mode = "legacy_fixed_camera_to_fr"
    ball_world_xy = None if start_pose is None else body_xy_to_world(start_pose, ball_body_xy)
    return {
        "geometry_mode": geometry_mode,
        "observed_ball_ground_range_m": perception.ball_ground_range_m,
        "observed_ball_ground_forward_m": perception.ball_ground_forward_m,
        "observed_ball_body_xy_m": list(ball_body_xy),
        "observed_ball_world_xy_m": None if ball_world_xy is None else list(ball_world_xy),
        "desired_final_ball_body_forward_m": desired_ball_body_x,
        "desired_final_fr_to_ball_center_m": desired_fr_to_ball_center,
        "desired_final_fr_to_ball_surface_gap_m": (
            desired_fr_to_ball_center - args.ball_radius_m - foot_radius
        ),
        "ball_radius_m": args.ball_radius_m,
        "fr_foot_collision_radius_m": foot_radius,
        "current_fr_foot_collision_body_xy_m": list(foot_xy),
        "camera_body_forward_m": camera_body_forward_m,
        "camera_body_lateral_m": args.camera_body_lateral_m,
        "camera_body_yaw_rad": args.camera_body_yaw_rad,
        "gait_to_kick_clearance_m": args.final_gait_to_kick_clearance_m,
        "forward_m": max(0.0, ball_body_xy[0] - desired_ball_body_x),
        "lateral_error_m": ball_body_xy[1] - foot_xy[1],
    }


def settled_fr_ball_gap(
    dock: dict[str, Any], settled_pose: dict[str, float],
    fr_kinematics: DirectFrKinematics, args: argparse.Namespace,
) -> dict[str, float | bool]:
    """고정된 공 world 위치를 full SE(2) odometry로 전파해 FR 간격을 검증한다."""
    ball_world = dock.get("observed_ball_world_xy_m")
    if not isinstance(ball_world, list) or len(ball_world) != 2:
        raise RuntimeError("final dock의 ball world 좌표가 없습니다")
    ball_body_xy = world_xy_to_body(
        settled_pose, (float(ball_world[0]), float(ball_world[1])),
    )
    foot_xy = fr_kinematics.foot_collision_center_body_m[:2]
    dx, dy = ball_body_xy[0] - foot_xy[0], ball_body_xy[1] - foot_xy[1]
    center_distance_m = math.hypot(dx, dy)
    radii_m = args.ball_radius_m + fr_kinematics.foot_collision_radius_m
    surface_gap_m = center_distance_m - radii_m
    forward_surface_gap_m = dx - radii_m
    lateral_error_m = dy
    foot_speed_m_s = math.sqrt(sum(
        value * value for value in fr_kinematics.foot_collision_speed_body_m_s
    ))
    gap_ready = (
        args.final_fr_to_ball_min_surface_gap_m
        <= forward_surface_gap_m
        <= args.final_fr_to_ball_target_surface_gap_m + args.final_fr_gap_tolerance_m
    )
    lateral_ready = abs(lateral_error_m) <= args.final_fr_max_lateral_error_m
    return {
        "estimated_ball_body_forward_m": ball_body_xy[0],
        "estimated_ball_body_lateral_m": ball_body_xy[1],
        "settled_fr_foot_collision_body_forward_m": foot_xy[0],
        "settled_fr_foot_collision_body_lateral_m": foot_xy[1],
        "fr_foot_collision_radius_m": fr_kinematics.foot_collision_radius_m,
        "estimated_fr_to_ball_center_m": center_distance_m,
        "estimated_fr_to_ball_surface_gap_m": surface_gap_m,
        "estimated_fr_to_ball_forward_surface_gap_m": forward_surface_gap_m,
        "estimated_fr_to_ball_lateral_error_m": lateral_error_m,
        "accepted_min_surface_gap_m": args.final_fr_to_ball_min_surface_gap_m,
        "accepted_max_surface_gap_m": (
            args.final_fr_to_ball_target_surface_gap_m + args.final_fr_gap_tolerance_m
        ),
        "fr_foot_speed_m_s": foot_speed_m_s,
        "fr_foot_speed_limit_m_s": args.final_fr_max_foot_speed_m_s,
        "gap_ready": gap_ready,
        "lateral_error_limit_m": args.final_fr_max_lateral_error_m,
        "lateral_ready": lateral_ready,
        "fr_speed_ready": foot_speed_m_s <= args.final_fr_max_foot_speed_m_s,
    }


def motion_settle_snapshot(
    sport_states: list[dict[str, Any]], odom_poses: list[dict[str, float]],
    *, window_s: float, max_planar_speed_m_s: float,
    max_yaw_speed_rad_s: float, max_odom_span_m: float,
    max_odom_yaw_span_rad: float,
) -> dict[str, Any]:
    """neutral 뒤 최근 state/odometry window가 실제 정지인지 fail-closed로 판정한다."""
    if not sport_states or not odom_poses:
        return {"ok": False, "reason": "settle_samples_missing"}
    latest_elapsed = min(sport_states[-1]["elapsed_s"], odom_poses[-1]["elapsed_s"])
    cutoff = latest_elapsed - window_s
    recent_states = [state for state in sport_states if state["elapsed_s"] >= cutoff]
    recent_poses = [pose for pose in odom_poses if pose["elapsed_s"] >= cutoff]
    if len(recent_states) < 3 or len(recent_poses) < 3:
        return {
            "ok": False, "reason": "settle_window_incomplete",
            "sport_sample_count": len(recent_states), "odom_sample_count": len(recent_poses),
        }
    velocities = [state.get("velocity") for state in recent_states]
    yaw_speeds = [state.get("yaw_speed") for state in recent_states]
    if any(not isinstance(value, list) or len(value) < 2 for value in velocities):
        return {"ok": False, "reason": "settle_velocity_missing"}
    try:
        max_planar_speed = max(math.hypot(float(value[0]), float(value[1])) for value in velocities)
        max_yaw_speed = max(abs(float(value)) for value in yaw_speeds)
    except (TypeError, ValueError):
        return {"ok": False, "reason": "settle_velocity_invalid"}
    anchor = recent_poses[-1]
    odom_span = max(planar_distance(anchor, pose) for pose in recent_poses)
    odom_yaw_span = circular_span([pose["yaw_rad"] for pose in recent_poses])
    ok = (
        max_planar_speed <= max_planar_speed_m_s
        and max_yaw_speed <= max_yaw_speed_rad_s
        and odom_span <= max_odom_span_m
        and odom_yaw_span <= max_odom_yaw_span_rad
    )
    return {
        "ok": ok,
        "reason": "motion_settled" if ok else "motion_still_active",
        "window_s": window_s,
        "sport_sample_count": len(recent_states),
        "odom_sample_count": len(recent_poses),
        "max_planar_speed_m_s": max_planar_speed,
        "max_yaw_speed_rad_s": max_yaw_speed,
        "odom_span_m": odom_span,
        "odom_yaw_span_rad": odom_yaw_span,
    }


async def wait_for_motion_settle(
    sport_states: list[dict[str, Any]], odom_poses: list[dict[str, float]],
    args: argparse.Namespace,
) -> dict[str, Any]:
    deadline = time.monotonic() + args.final_settle_timeout_s
    snapshot: dict[str, Any] = {"ok": False, "reason": "settle_not_sampled"}
    while time.monotonic() < deadline:
        snapshot = motion_settle_snapshot(
            sport_states, odom_poses,
            window_s=args.final_settle_window_s,
            max_planar_speed_m_s=args.final_settle_max_planar_speed_m_s,
            max_yaw_speed_rad_s=args.final_settle_max_yaw_speed_rad_s,
            max_odom_span_m=args.final_settle_max_odom_span_m,
            max_odom_yaw_span_rad=args.final_settle_max_odom_yaw_span_rad,
        )
        if snapshot["ok"]:
            return snapshot
        await asyncio.sleep(0.05)
    snapshot["timeout_s"] = args.final_settle_timeout_s
    return snapshot


def publish_joystick(pub_sub: Any, topic: str, *, lx: float = 0.0, ly: float = 0.0, rx: float = 0.0) -> None:
    pub_sub.publish_without_callback(topic, {"lx": lx, "ly": ly, "rx": rx, "ry": 0.0, "keys": 0})


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name("." + path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def arm_virtual_echo_window(path: Path, *, lx: float, ly: float, rx: float, duration_s: float) -> None:
    """direct DDS watcher가 자기 WebRTC packet echo를 physical input으로 오인하지 않게 한다.

    Unitree SDK callback에는 DDS writer identity가 없다. 따라서 active pulse 동안 짧은
    exact-value lease만 watcher와 공유하고 주기적으로 갱신한다. 값까지 같은 physical input은
    구별할 수 없으므로 lease는 scheduling 지연을 덮는 범위보다 길게 유지하지 않는다.
    """
    now = time.monotonic()
    atomic_write_json(path, {
        "schema_version": 1,
        "kind": "go2_stage_virtual_joystick_echo_window",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "expires_monotonic_s": now + duration_s + VIRTUAL_ECHO_GRACE_S,
        "joystick": {"lx": lx, "ly": ly, "rx": rx, "ry": 0.0, "keys": 0},
    })


def disarm_virtual_echo_window(path: Path) -> None:
    """pulse가 끝나면 즉시 expiry를 과거로 보내 watcher의 physical-input 감시를 복원한다."""
    atomic_write_json(path, {
        "schema_version": 1,
        "kind": "go2_stage_virtual_joystick_echo_window",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "expires_monotonic_s": time.monotonic() - 1.0,
        "joystick": {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "keys": 0},
    })


async def neutralize(pub_sub: Any, topic: str) -> int:
    for _ in range(NEUTRAL_PACKET_COUNT):
        publish_joystick(pub_sub, topic)
        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
    return NEUTRAL_PACKET_COUNT


async def joystick_pulse(
    pub_sub: Any, topic: str, *, lx: float, ly: float, rx: float, duration_s: float,
    direct_latch: DirectRemoteLatch, args: argparse.Namespace,
    odom_poses: list[dict[str, float]] | None = None,
    start_pose: dict[str, float] | None = None,
    stop_after_forward_m: float | None = None,
    stop_after_yaw_rad: float | None = None,
    stop_after_lateral_m: float | None = None,
) -> tuple[int, int, str]:
    """partial burst도 neutralize한다. direct DDS remote input은 다음 packet 전에 stop한다."""
    packets = 0
    termination = "pulse_complete"
    last_pose_count = len(odom_poses) if odom_poses is not None else 0
    checked_pose_count = last_pose_count
    last_odom_at = time.monotonic()
    pulse_started_at = time.monotonic()
    forward_confirmation_count = 0
    yaw_confirmation_count = 0
    lateral_confirmation_count = 0
    yaw_direction_mismatch_count = 0
    lateral_direction_mismatch_count = 0
    arm_virtual_echo_window(
        args.virtual_echo_window, lx=lx, ly=ly, rx=rx, duration_s=VIRTUAL_ECHO_LEASE_S,
    )
    next_echo_refresh_at = pulse_started_at + VIRTUAL_ECHO_REFRESH_S
    try:
        for _ in range(math.ceil(duration_s * COMMAND_RATE_HZ)):
            now = time.monotonic()
            if now >= next_echo_refresh_at:
                # 50 Hz loop scheduling이 nominal duration보다 늘어나도 active DDS echo가
                # physical remote로 오인되지 않도록 짧은 exact-value lease만 갱신한다.
                arm_virtual_echo_window(
                    args.virtual_echo_window, lx=lx, ly=ly, rx=rx,
                    duration_s=VIRTUAL_ECHO_LEASE_S,
                )
                next_echo_refresh_at = now + VIRTUAL_ECHO_REFRESH_S
            if (
                stop_after_forward_m is not None
                or stop_after_yaw_rad is not None
                or stop_after_lateral_m is not None
            ):
                assert odom_poses is not None and start_pose is not None
                if len(odom_poses) != last_pose_count:
                    last_pose_count = len(odom_poses)
                    last_odom_at = time.monotonic()
                if time.monotonic() - last_odom_at > args.max_odom_stale_s:
                    termination = "odom_stale_during_pulse"
                    break
                # robot_odom의 gait/body transient 한 샘플이 3 cm 목표를 넘었다고 즉시
                # neutralize하면 실제 발걸음 전에 몸만 움찔한다. 최소 active time 뒤,
                # 서로 다른 새 odometry 샘플이 연속으로 목표를 확인할 때만 끝낸다.
                if (
                    odom_poses
                    and len(odom_poses) != checked_pose_count
                    and time.monotonic() - pulse_started_at >= args.min_odom_stop_active_s
                ):
                    checked_pose_count = len(odom_poses)
                    if stop_after_forward_m is not None:
                        forward_reached = (
                            forward_progress_m(start_pose, odom_poses[-1]) >= stop_after_forward_m
                        )
                        forward_confirmation_count = (
                            forward_confirmation_count + 1 if forward_reached else 0
                        )
                        if forward_confirmation_count >= args.odom_stop_confirm_samples:
                            termination = "forward_odom_target_reached_confirmed"
                            break
                    if stop_after_yaw_rad is not None:
                        yaw_progress = commanded_yaw_progress_rad(
                            rx, start_pose, odom_poses[-1],
                        )
                        yaw_confirmation_count = (
                            yaw_confirmation_count + 1
                            if yaw_progress >= stop_after_yaw_rad else 0
                        )
                        if yaw_confirmation_count >= args.odom_stop_confirm_samples:
                            termination = "yaw_odom_target_reached_confirmed"
                            break
                        yaw_direction_mismatch_count = (
                            yaw_direction_mismatch_count + 1
                            if yaw_progress <= -min(0.02, stop_after_yaw_rad * 0.5) else 0
                        )
                        if yaw_direction_mismatch_count >= args.odom_stop_confirm_samples:
                            termination = "yaw_odom_direction_mismatch"
                            break
                    if stop_after_lateral_m is not None:
                        lateral_progress = commanded_lateral_progress_m(
                            lx, start_pose, odom_poses[-1],
                        )
                        lateral_reached = lateral_progress >= stop_after_lateral_m
                        lateral_confirmation_count = (
                            lateral_confirmation_count + 1 if lateral_reached else 0
                        )
                        if lateral_confirmation_count >= args.odom_stop_confirm_samples:
                            termination = "lateral_odom_target_reached_confirmed"
                            break
                        lateral_direction_mismatch_count = (
                            lateral_direction_mismatch_count + 1
                            if lateral_progress <= -min(0.02, stop_after_lateral_m * 0.5) else 0
                        )
                        if lateral_direction_mismatch_count >= args.odom_stop_confirm_samples:
                            termination = "lateral_odom_direction_mismatch"
                            break
            watchdog, watchdog_reason = load_direct_remote_watchdog(
                args.direct_remote_status,
                max_age_s=args.direct_remote_max_age_s,
                hold_s=args.direct_remote_hold_s,
                virtual_echo_window=args.virtual_echo_window,
                require_fr_kinematics=args.use_direct_fr_kinematics,
                fr_kinematics_max_age_s=args.fr_kinematics_max_age_s,
            )
            if direct_latch.tripped() or watchdog_reason == "physical_remote_active":
                termination = "physical_remote_preempted"
                break
            if watchdog is None:
                termination = "direct_remote_watchdog_lost:{}".format(watchdog_reason)
                break
            publish_joystick(pub_sub, topic, lx=lx, ly=ly, rx=rx)
            packets += 1
            await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
    finally:
        neutral_packets = await neutralize(pub_sub, topic)
        # 마지막 active packet의 DDS echo가 neutral 뒤에 도착할 수 있다.
        await asyncio.sleep(VIRTUAL_ECHO_GRACE_S)
        disarm_virtual_echo_window(args.virtual_echo_window)
    return packets, neutral_packets, termination


async def run(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.constants import RTC_TOPIC
    except ImportError as error:
        raise RuntimeError("unitree_webrtc_connect import 실패: {}".format(error)) from error

    if args.execute:
        watchdog, watchdog_reason = load_direct_remote_watchdog(
            args.direct_remote_status,
            max_age_s=args.direct_remote_max_age_s,
            hold_s=args.direct_remote_hold_s,
            virtual_echo_window=args.virtual_echo_window,
            require_fr_kinematics=args.use_direct_fr_kinematics,
            fr_kinematics_max_age_s=args.fr_kinematics_max_age_s,
        )
        if watchdog is None:
            return {
                "connected": False, "execute": True, "motion_commands_sent": False,
                "verdict": "DIRECT_REMOTE_WATCHDOG_REJECTED", "reason": watchdog_reason,
            }

    lane_template, template_reason = load_fr_lane_template(args.fr_lane_template, args.tag_id)
    if args.execute and lane_template is None:
        return {
            "connected": False, "execute": True, "motion_commands_sent": False,
            "verdict": "FR_LANE_TEMPLATE_REJECTED", "reason": template_reason,
        }

    connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.robot_ip)
    sport_states: list[dict[str, Any]] = []
    odom_poses: list[dict[str, float]] = []
    direct_latch = DirectRemoteLatch()
    udp_transport: asyncio.DatagramTransport | None = None
    connected_at = 0.0
    command_packets = 0
    neutral_packets = 0
    try:
        if args.execute:
            loop = asyncio.get_running_loop()
            try:
                udp_transport, _ = await loop.create_datagram_endpoint(
                    lambda: direct_latch,
                    local_addr=("127.0.0.1", args.direct_remote_udp_port),
                )
            except OSError as error:
                return {
                    "connected": False, "execute": True, "motion_commands_sent": False,
                    "verdict": "DIRECT_REMOTE_UDP_BIND_FAILED", "reason": str(error),
                }
        await asyncio.wait_for(connection.connect(), timeout=args.connect_timeout_s)
        connected_at = time.monotonic()

        def on_sport_state(message: Any) -> None:
            sport_states.append(sport_state_summary(message, time.monotonic() - connected_at))

        def on_robot_pose(message: Any) -> None:
            pose = pose_summary(message, time.monotonic() - connected_at)
            if pose is not None:
                odom_poses.append(pose)

        pub_sub = connection.datachannel.pub_sub
        pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_sport_state)
        pub_sub.subscribe(RTC_TOPIC["ROBOTODOM"], on_robot_pose)
        await asyncio.sleep(STATE_SETTLE_S)
        initial_state = sport_states[-1] if sport_states else None
        state_ok, state_reason = state_is_safe_to_walk(initial_state)
        baseline, baseline_span_m, baseline_yaw_span_rad = static_baseline(odom_poses)
        odom_ok = (
            baseline is not None
            and baseline_span_m <= args.max_static_baseline_span_m
            and baseline_yaw_span_rad <= args.max_static_baseline_yaw_span_rad
        )
        perception, perception_reason = await stable_perception(args)
        use_tag_guidance = perception is not None and perception.tag_visible
        watchdog, watchdog_reason = load_direct_remote_watchdog(
            args.direct_remote_status,
            max_age_s=args.direct_remote_max_age_s,
            hold_s=args.direct_remote_hold_s,
            virtual_echo_window=args.virtual_echo_window,
            require_fr_kinematics=args.use_direct_fr_kinematics,
            fr_kinematics_max_age_s=args.fr_kinematics_max_age_s,
        )
        direct_remote_ok = (
            watchdog is not None and (not args.execute or not direct_latch.tripped())
            if args.use_direct_fr_kinematics else
            (not args.execute) or (watchdog is not None and not direct_latch.tripped())
        )
        direct_fr_ok = (
            not args.use_direct_fr_kinematics
            or (watchdog is not None and watchdog.fr_foot_kinematics is not None)
        )
        camera_body_forward_m = args.camera_body_forward_m
        direct_fr_calibration_ok = not args.use_direct_fr_kinematics
        if (
            direct_fr_ok and args.use_direct_fr_kinematics
            and camera_body_forward_m is not None
        ):
            assert watchdog is not None and watchdog.fr_foot_kinematics is not None
            direct_fr_calibration_ok = -0.30 <= (
                watchdog.fr_foot_kinematics.foot_position_body_m[0]
                - float(camera_body_forward_m)
            ) <= 0.10
        result: dict[str, Any] = {
            "connected": True,
            "robot_ip": args.robot_ip,
            "execute": args.execute,
            "motion_commands_sent": False,
            "preflight": {
                "ok": (
                    state_ok and odom_ok and perception is not None
                    and lane_template is not None and direct_remote_ok and direct_fr_ok
                    and direct_fr_calibration_ok
                ),
                "mcf_state_reason": state_reason,
                "sport_state": initial_state,
                "odom_baseline": baseline,
                "odom_static_span_m": baseline_span_m,
                "odom_static_span_threshold_m": args.max_static_baseline_span_m,
                "odom_static_yaw_span_rad": baseline_yaw_span_rad,
                "odom_static_yaw_span_threshold_rad": args.max_static_baseline_yaw_span_rad,
                "perception_reason": perception_reason,
                "perception": None if perception is None else asdict(perception),
                "alignment_mode": "tag_guided" if use_tag_guidance else "ball_only",
                "target_direction_verified": use_tag_guidance,
                "fr_lane_bearing_errors": (
                    None if perception is None or lane_template is None
                    else alignment_errors(
                        perception, lane_template, args.lane_axis_bearing_rad, use_tag_guidance,
                    )
                ),
                "fr_lane_template_reason": template_reason,
                "fr_lane_template": None if lane_template is None else asdict(lane_template),
                "direct_remote_watchdog_reason": watchdog_reason,
                "direct_remote_watchdog": None if watchdog is None else asdict(watchdog),
                "direct_fr_kinematics_ok": direct_fr_ok,
                "direct_fr_camera_calibration_ok": direct_fr_calibration_ok,
                "camera_body_forward_m": camera_body_forward_m,
                "direct_remote_events_before_command": direct_latch.events,
            },
            "command_contract": {
                "transport": "WebRTC rt/wirelesscontroller only",
                "physical_remote_guard": "direct DDS watcher heartbeat + localhost UDP event",
                "virtual_echo_window": str(args.virtual_echo_window),
                "writer_identity_available": False,
                "same_value_physical_input_during_echo_window_unobservable": True,
                "joystick_magnitude": args.joystick_magnitude,
                "rate_hz": COMMAND_RATE_HZ,
                "min_odom_stop_active_s": args.min_odom_stop_active_s,
                "odom_stop_confirm_samples": args.odom_stop_confirm_samples,
                "camera_stage_entry_slack_m": args.camera_stage_entry_slack_m,
                "final_gait_to_kick_clearance_m": args.final_gait_to_kick_clearance_m,
                "final_settle_timeout_s": args.final_settle_timeout_s,
                "final_settle_window_s": args.final_settle_window_s,
                "final_settle_max_odom_yaw_span_rad": (
                    args.final_settle_max_odom_yaw_span_rad
                ),
                "virtual_echo_lease_refresh_s": VIRTUAL_ECHO_REFRESH_S,
                "virtual_echo_lease_s": VIRTUAL_ECHO_LEASE_S,
                "observation_timeout_s": args.observation_timeout_s,
                "lane_axis_bearing_rad": args.lane_axis_bearing_rad,
                "target_bearing_tolerance_rad": args.target_bearing_tolerance_rad,
                "ball_bearing_tolerance_rad": args.ball_bearing_tolerance_rad,
                "ball_diameter_consistency_ratio": [
                    args.min_ball_diameter_consistency_ratio,
                    args.max_ball_diameter_consistency_ratio,
                ],
                "ball_only_yaw_action_tolerance_rad": (
                    BALL_ONLY_YAW_ACTION_TOLERANCE_RAD
                ),
                "tagless_ball_kick_opt_in": args.allow_tagless_ball_kick,
                "forward_pulse_s": args.forward_pulse_s,
                "max_forward_pulse_travel_m": args.max_forward_pulse_travel_m,
                "final_dock_enabled": args.enable_final_dock,
                "camera_to_fr_forward_m": args.camera_to_fr_forward_m,
                "fr_to_ball_forward_m": args.fr_to_ball_forward_m,
                "use_direct_fr_kinematics": args.use_direct_fr_kinematics,
                "camera_body_forward_m": camera_body_forward_m,
                "camera_body_lateral_m": args.camera_body_lateral_m,
                "camera_body_yaw_rad": args.camera_body_yaw_rad,
                "fr_kinematics_max_age_s": args.fr_kinematics_max_age_s,
                "ball_radius_m": args.ball_radius_m,
                "final_fr_to_ball_min_surface_gap_m": (
                    args.final_fr_to_ball_min_surface_gap_m
                ),
                "final_fr_to_ball_target_surface_gap_m": (
                    args.final_fr_to_ball_target_surface_gap_m
                ),
                "final_fr_gap_tolerance_m": args.final_fr_gap_tolerance_m,
                "fr_foot_collision_radius_m": FR_FOOT_COLLISION_RADIUS_M,
                "final_fr_max_lateral_error_m": args.final_fr_max_lateral_error_m,
                "final_fr_max_foot_speed_m_s": args.final_fr_max_foot_speed_m_s,
                "final_dock_max_m": args.final_dock_max_m,
                "final_dock_max_duration_s": args.final_dock_max_duration_s,
                "turn_pulse_s": args.turn_pulse_s,
                "lateral_pulse_s": args.lateral_pulse_s,
                "max_lateral_pulse_travel_m": args.max_lateral_pulse_travel_m,
                "lateral_search_opt_in": args.allow_lateral_search,
                "initial_lateral_sign": args.lateral_sign,
                "lateral_sign_preconfirmed": args.lateral_sign_confirmed,
                "lateral_probe_only": args.lateral_probe_only,
                "lateral_min_improvement_rad": args.lateral_min_improvement_rad,
                "max_lateral_probe_attempts": args.max_lateral_probe_attempts,
                "neutral_packets_after_each_pulse": NEUTRAL_PACKET_COUNT,
                "max_travel_m": args.max_travel_m,
                "max_total_travel_m": args.max_total_travel_m,
                "max_cycles": args.max_cycles,
                "yaw_odom_response_sign_per_rx": YAW_ODOM_RESPONSE_SIGN_PER_RX,
                "lateral_odom_response_sign_per_lx": LATERAL_ODOM_RESPONSE_SIGN_PER_LX,
                "final_fr_kick_or_lowcmd": False,
                "obstacle_avoidance_modified": False,
            },
            "cycles": [],
        }
        if lane_template is None:
            result["verdict"] = "FR_LANE_TEMPLATE_REQUIRED" if args.fr_lane_template is None else "FR_LANE_TEMPLATE_REJECTED"
            return result
        if not result["preflight"]["ok"]:
            result["verdict"] = "PREFLIGHT_REJECTED"
            return result

        assert perception is not None and baseline is not None and lane_template is not None
        if not use_tag_guidance and not args.allow_tagless_ball_kick:
            result["verdict"] = "TAG_GUIDANCE_REQUIRED"
            return result
        result["alignment_mode"] = "tag_guided" if use_tag_guidance else "ball_only"
        result["target_direction_verified"] = use_tag_guidance
        # lateral joystick의 + 부호가 camera ball-bearing을 어느 방향으로 바꾸는지는
        # mount/firmware 환경마다 확정할 수 없다. 첫 짧은 probe만 관측해 sign을 정한다.
        lateral_sign = args.lateral_sign
        lateral_sign_confirmed = args.lateral_sign_confirmed
        lateral_response_sign: float | None = None
        if lateral_sign_confirmed and use_tag_guidance:
            initial_relative_error = fr_lane_bearing_errors(
                perception, lane_template, args.lane_axis_bearing_rad,
            )["relative_error_rad"]
            lateral_response_sign = (
                -math.copysign(1.0, initial_relative_error)
                * math.copysign(1.0, lateral_sign)
            )
        lateral_probe_attempts = 0
        pending_lateral_probe: dict[str, Any] | None = None
        result["lateral_probe_results"] = []
        first_action = action_for(
            perception, args, lane_template, lateral_sign, use_tag_guidance,
        )
        if not args.execute:
            result["dry_run_next_action"] = {
                "reason": first_action[0],
                "joystick": list(first_action[1:]),
                "fr_lane_bearing_errors": alignment_errors(
                    perception, lane_template, args.lane_axis_bearing_rad, use_tag_guidance,
                ),
            }
            if args.enable_final_dock:
                result["dry_run_final_dock_plan"] = final_dock_plan(
                    perception, args,
                    None if watchdog is None else watchdog.fr_foot_kinematics,
                    camera_body_forward_m,
                    odom_poses[-1] if odom_poses else baseline,
                )
            result["verdict"] = "DRY_RUN_READY"
            return result

        for cycle_index in range(args.max_cycles):
            watchdog, watchdog_reason = load_direct_remote_watchdog(
                args.direct_remote_status,
                max_age_s=args.direct_remote_max_age_s,
                hold_s=args.direct_remote_hold_s,
                virtual_echo_window=args.virtual_echo_window,
                require_fr_kinematics=args.use_direct_fr_kinematics,
                fr_kinematics_max_age_s=args.fr_kinematics_max_age_s,
            )
            if direct_latch.tripped() or watchdog_reason == "physical_remote_active":
                result["verdict"] = "PHYSICAL_REMOTE_PREEMPTED"
                break
            if watchdog is None:
                result["verdict"] = "DIRECT_REMOTE_WATCHDOG_LOST"
                result["reason"] = watchdog_reason
                break
            perception, perception_reason = await stable_perception(args)
            latest_pose = odom_poses[-1] if odom_poses else None
            if perception is None:
                result["verdict"] = "PERCEPTION_REJECTED_{}".format(perception_reason.upper())
                break
            if use_tag_guidance and not perception.tag_visible:
                result["verdict"] = "TAG_LOST_DURING_TAG_GUIDED_STAGE"
                break
            if not use_tag_guidance and perception.tag_visible:
                result["verdict"] = "TAG_VISIBILITY_CHANGED_DURING_BALL_ONLY_STAGE"
                break
            if args.use_direct_fr_kinematics:
                # detector wait가 길어졌더라도 final plan에는 방금 받은 LowState FK만 쓴다.
                watchdog, watchdog_reason = load_direct_remote_watchdog(
                    args.direct_remote_status,
                    max_age_s=args.direct_remote_max_age_s,
                    hold_s=args.direct_remote_hold_s,
                    virtual_echo_window=args.virtual_echo_window,
                    require_fr_kinematics=True,
                    fr_kinematics_max_age_s=args.fr_kinematics_max_age_s,
                )
                if watchdog is None:
                    result["verdict"] = "DIRECT_FR_KINEMATICS_LOST"
                    result["reason"] = watchdog_reason
                    break
            if pending_lateral_probe is not None:
                if not perception.tag_visible:
                    result["verdict"] = "TAG_LOST_DURING_LATERAL_PROBE"
                    break
                after_errors = fr_lane_bearing_errors(
                    perception, lane_template, args.lane_axis_bearing_rad,
                )
                after_abs_error = abs(after_errors["relative_error_rad"])
                improvement = pending_lateral_probe["before_abs_relative_error_rad"] - after_abs_error
                probe_result = {
                    **pending_lateral_probe,
                    "after_abs_relative_error_rad": after_abs_error,
                    "after_fr_lane_bearing_errors": after_errors,
                    "improvement_rad": improvement,
                }
                if improvement >= args.lateral_min_improvement_rad:
                    signed_change = angle_distance(
                        after_errors["relative_error_rad"],
                        pending_lateral_probe["before_relative_error_rad"],
                    )
                    lateral_response_sign = math.copysign(
                        1.0, signed_change * pending_lateral_probe["sign"],
                    )
                    lateral_sign = (
                        -math.copysign(1.0, after_errors["relative_error_rad"])
                        * lateral_response_sign
                    )
                    lateral_sign_confirmed = True
                    probe_result["verdict"] = "sign_confirmed"
                    probe_result["lateral_response_sign"] = lateral_response_sign
                elif lateral_probe_attempts >= args.max_lateral_probe_attempts:
                    probe_result["verdict"] = "no_convergence"
                    result["lateral_probe_results"].append(probe_result)
                    result["verdict"] = "LATERAL_PROBE_NO_CONVERGENCE"
                    break
                else:
                    # 반대 sign도 한 번만 관측한다. sign을 추정으로 고정하지 않는다.
                    lateral_sign = -pending_lateral_probe["sign"]
                    probe_result["verdict"] = "flip_sign_and_retry"
                probe_result["recommended_lateral_sign"] = lateral_sign
                result["lateral_probe_results"].append(probe_result)
                pending_lateral_probe = None
                if args.lateral_probe_only:
                    result["recommended_lateral_sign"] = lateral_sign
                    result["lateral_sign_confirmed"] = lateral_sign_confirmed
                    result["verdict"] = "LATERAL_PROBE_MEASURED"
                    break
            if latest_pose is None or time.monotonic() - connected_at - latest_pose["elapsed_s"] > args.max_odom_stale_s:
                result["verdict"] = "ODOM_STALE"
                break
            travel_m = planar_distance(baseline, latest_pose)
            if travel_m >= args.max_travel_m:
                result["verdict"] = "TRAVEL_LIMIT_REACHED"
                break
            remaining_stage_travel_m = args.max_travel_m - travel_m
            action_lateral_sign = lateral_sign
            if lateral_response_sign is not None and use_tag_guidance:
                current_relative_error = fr_lane_bearing_errors(
                    perception, lane_template, args.lane_axis_bearing_rad,
                )["relative_error_rad"]
                action_lateral_sign = (
                    -math.copysign(1.0, current_relative_error)
                    * lateral_response_sign
                )
                lateral_sign = action_lateral_sign
            reason, lx, ly, rx = action_for(
                perception, args, lane_template, action_lateral_sign, use_tag_guidance,
            )
            cycle: dict[str, Any] = {
                "index": cycle_index,
                "perception": asdict(perception),
                "fr_lane_bearing_errors": alignment_errors(
                    perception, lane_template, args.lane_axis_bearing_rad, use_tag_guidance,
                ),
                "start_odom": latest_pose,
                "travel_m": travel_m,
                "reason": reason,
                "joystick": {"lx": lx, "ly": ly, "rx": rx},
            }
            result["cycles"].append(cycle)
            if reason == "camera_staging_ready":
                strict_errors = alignment_errors(
                    perception, lane_template, args.lane_axis_bearing_rad, use_tag_guidance,
                )
                if use_tag_guidance:
                    strict_lane_ready = (
                        abs(strict_errors["ball_error_rad"]) <= min(
                            lane_template.ball_bearing_tolerance_rad,
                            args.ball_bearing_tolerance_rad,
                        )
                        and abs(strict_errors["target_error_rad"]) <= min(
                            lane_template.target_bearing_tolerance_rad,
                            args.target_bearing_tolerance_rad,
                        )
                        and abs(strict_errors["relative_error_rad"]) <= min(
                            lane_template.ball_bearing_tolerance_rad,
                            lane_template.target_bearing_tolerance_rad,
                            args.ball_bearing_tolerance_rad,
                        )
                    )
                else:
                    strict_lane_ready = abs(strict_errors["ball_error_rad"]) <= min(
                        lane_template.ball_bearing_tolerance_rad,
                        args.ball_bearing_tolerance_rad,
                    )
                dock_complete = not args.enable_final_dock
                final_fr_gap: dict[str, float | bool] | None = None
                if args.enable_final_dock:
                    dock = final_dock_plan(
                        perception, args, watchdog.fr_foot_kinematics,
                        camera_body_forward_m,
                        latest_pose,
                    )
                    result["final_dock"] = dock
                    remaining_total_m = max(
                        0.0, args.max_total_travel_m - planar_distance(baseline, latest_pose),
                    )
                    result["final_dock"]["remaining_total_travel_limit_m"] = remaining_total_m
                    if dock["forward_m"] > min(args.final_dock_max_m, remaining_total_m):
                        result["verdict"] = "FINAL_DOCK_DISTANCE_REJECTED"
                        result["kick_ready"] = {
                            "eligible": False,
                            "reason": "computed final dock exceeds explicit hard limit",
                        }
                        break
                    if dock["forward_m"] <= 0.02:
                        dock_complete = True
                        result["final_dock"].update({
                            "pulse_result": "already_at_final_ground_range",
                            "measured_forward_m": 0.0,
                            "start_odom": latest_pose,
                            "final_odom": latest_pose,
                            "complete": True,
                        })
                    else:
                        sent, neutral, pulse_result = await joystick_pulse(
                            pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"],
                            lx=0.0, ly=args.joystick_magnitude, rx=0.0,
                            duration_s=args.final_dock_max_duration_s,
                            direct_latch=direct_latch, args=args,
                            odom_poses=odom_poses, start_pose=latest_pose,
                            stop_after_forward_m=dock["forward_m"],
                        )
                        command_packets += sent
                        neutral_packets += neutral
                        result["motion_commands_sent"] = result["motion_commands_sent"] or sent > 0
                        await asyncio.sleep(args.reobserve_settle_s)
                        final_dock_pose = odom_poses[-1] if odom_poses else None
                        measured_forward = (
                            forward_progress_m(latest_pose, final_dock_pose)
                            if final_dock_pose is not None else float("-inf")
                        )
                        dock_complete = measured_forward >= dock["forward_m"] - 0.02
                        result["final_dock"].update({
                            "pulse_result": pulse_result,
                            "packets_sent": sent,
                            "neutral_packets": neutral,
                            "start_odom": latest_pose,
                            "final_odom": final_dock_pose,
                            "measured_forward_m": measured_forward,
                            "complete": dock_complete,
                        })
                        if pulse_result == "physical_remote_preempted":
                            result["verdict"] = "PHYSICAL_REMOTE_PREEMPTED"
                            break
                        if pulse_result.startswith("direct_remote_watchdog_lost:"):
                            result["verdict"] = "DIRECT_REMOTE_WATCHDOG_LOST"
                            result["reason"] = pulse_result
                            break
                        if pulse_result == "odom_stale_during_pulse":
                            result["verdict"] = "ODOM_STALE"
                            break
                        if pulse_result.endswith("_odom_direction_mismatch"):
                            result["verdict"] = "ODOM_COMMAND_DIRECTION_MISMATCH"
                            result["reason"] = pulse_result
                            break
                    settle = await wait_for_motion_settle(sport_states, odom_poses, args)
                    result["final_dock"]["motion_settle"] = settle
                    if "start_odom" in result["final_dock"] and odom_poses:
                        settled_pose = odom_poses[-1]
                        settled_forward = forward_progress_m(
                            result["final_dock"]["start_odom"], settled_pose,
                        )
                        settled_total_travel_m = planar_distance(baseline, settled_pose)
                        dock_complete = settled_forward >= dock["forward_m"] - 0.02
                        result["final_dock"].update({
                            "final_odom": settled_pose,
                            "measured_forward_m": settled_forward,
                            "measured_total_travel_m": settled_total_travel_m,
                            "complete": dock_complete,
                        })
                        if settled_total_travel_m > args.max_total_travel_m:
                            dock_complete = False
                            result["final_dock"]["complete"] = False
                            result["verdict"] = "TOTAL_TRAVEL_LIMIT_EXCEEDED"
                            break
                    if not settle["ok"]:
                        dock_complete = False
                        result["final_dock"]["complete"] = False
                        result["verdict"] = "FINAL_DOCK_NOT_SETTLED"
                        break
                    if not dock_complete:
                        result["verdict"] = "FINAL_DOCK_INCOMPLETE"
                        break
                    if args.use_direct_fr_kinematics:
                        settled_watchdog, settled_watchdog_reason = load_direct_remote_watchdog(
                            args.direct_remote_status,
                            max_age_s=args.direct_remote_max_age_s,
                            hold_s=args.direct_remote_hold_s,
                            virtual_echo_window=args.virtual_echo_window,
                            require_fr_kinematics=True,
                            fr_kinematics_max_age_s=args.fr_kinematics_max_age_s,
                        )
                        if (
                            settled_watchdog is None
                            or settled_watchdog.fr_foot_kinematics is None
                        ):
                            result["final_dock"]["complete"] = False
                            result["verdict"] = "DIRECT_FR_KINEMATICS_LOST"
                            result["reason"] = settled_watchdog_reason
                            break
                        assert settled_pose is not None
                        final_fr_gap = settled_fr_ball_gap(
                            dock,
                            settled_pose,
                            settled_watchdog.fr_foot_kinematics,
                            args,
                        )
                        result["final_dock"]["settled_fr_ball_gap"] = final_fr_gap
                        if not final_fr_gap["fr_speed_ready"]:
                            result["final_dock"]["complete"] = False
                            result["verdict"] = "FINAL_FR_NOT_SETTLED"
                            break
                        if not final_fr_gap["gap_ready"]:
                            result["final_dock"]["complete"] = False
                            result["verdict"] = "FINAL_FR_GAP_OUT_OF_RANGE"
                            break
                        if not final_fr_gap["lateral_ready"]:
                            result["final_dock"]["complete"] = False
                            result["verdict"] = "FINAL_FR_LATERAL_OUT_OF_RANGE"
                            break
                result["verdict"] = (
                    "FINAL_DOCKING_READY" if args.enable_final_dock and dock_complete
                    else "CAMERA_STAGING_READY"
                )
                result["kick_ready"] = {
                    "eligible": strict_lane_ready and dock_complete,
                    "geometry_source": (
                        "D435i ball+Tag FR lane template + floor-plane range + LiDAR odometry"
                        if use_tag_guidance else
                        "D435i ball-only calibrated FR bearing + floor-plane range + LiDAR odometry"
                    ) + (
                        " + direct DDS LowState/URDF FR FK surface-gap gate"
                        if args.use_direct_fr_kinematics else ""
                    ),
                    "alignment_mode": "tag_guided" if use_tag_guidance else "ball_only",
                    "target_direction_verified": use_tag_guidance,
                    "lowcmd_started": False,
                    "final_dock_complete": dock_complete,
                    "direct_fr_gap_ready": (
                        None if final_fr_gap is None else (
                            final_fr_gap["gap_ready"] and final_fr_gap["lateral_ready"]
                        )
                    ),
                    "settled_fr_ball_gap": final_fr_gap,
                    "reason": (
                        (
                            "strict FR lane geometry and bounded final docking passed; "
                            "MCF_to_LowCmd remains a separate harness action"
                            if use_tag_guidance else
                            "explicit tagless ball-only FR bearing and bounded final docking passed; "
                            "kick target direction is unverified"
                        )
                        if strict_lane_ready and dock_complete else
                        "rough geometry or incomplete docking; explicit rough-kick opt-in remains required"
                    ),
                }
                break
            if reason == "ball_too_close_no_reverse":
                result["verdict"] = reason.upper()
                break
            if reason == "lateral_to_fr_lane" and not args.allow_lateral_search:
                result["verdict"] = "LATERAL_SEARCH_NOT_ARMED"
                break
            if reason == "lateral_to_fr_lane" and not lateral_sign_confirmed:
                if lateral_probe_attempts >= args.max_lateral_probe_attempts:
                    result["verdict"] = "LATERAL_PROBE_NO_CONVERGENCE"
                    break
                lateral_probe_attempts += 1
                pending_lateral_probe = {
                    "attempt": float(lateral_probe_attempts),
                    "sign": math.copysign(1.0, lx),
                    "before_abs_relative_error_rad": abs(
                        fr_lane_bearing_errors(
                            perception, lane_template, args.lane_axis_bearing_rad,
                        )["relative_error_rad"],
                    ),
                    "before_relative_error_rad": fr_lane_bearing_errors(
                        perception, lane_template, args.lane_axis_bearing_rad,
                    )["relative_error_rad"],
                    "before_fr_lane_bearing_errors": fr_lane_bearing_errors(
                        perception, lane_template, args.lane_axis_bearing_rad,
                    ),
                }
            duration_s = (
                args.forward_pulse_s if reason == "forward"
                else args.turn_pulse_s if reason in ("turn_to_tag_ray", "turn_to_ball_lane")
                else args.lateral_pulse_s
            )
            stop_after_forward_m = None
            stop_after_yaw_rad = None
            stop_after_lateral_m = None
            if reason == "forward":
                remaining_to_stage_m = max(
                    0.0,
                    perception.ball_range_m
                    - (
                        lane_template.desired_ball_range_m
                        + lane_template.range_tolerance_m
                        + args.camera_stage_entry_slack_m
                    ),
                )
                stop_after_forward_m = min(
                    args.max_forward_pulse_travel_m,
                    remaining_stage_travel_m,
                    max(0.03, remaining_to_stage_m),
                )
                if stop_after_forward_m <= 0.02:
                    result["verdict"] = "TRAVEL_LIMIT_REACHED"
                    break
                cycle["forward_odom_target_m"] = stop_after_forward_m
            elif reason in ("turn_to_tag_ray", "turn_to_ball_lane"):
                stop_after_yaw_rad = min(
                    abs(cycle["fr_lane_bearing_errors"][
                        "target_error_rad" if use_tag_guidance else "ball_error_rad"
                    ]),
                    0.20,
                )
                cycle["yaw_odom_target_rad"] = stop_after_yaw_rad
            elif reason == "lateral_to_fr_lane":
                # small-angle ground approximation: lateral ~= range * bearing error.
                # 8 cm hard cap으로 게걸음 overshoot를 막고 다음 cycle에서 재관측한다.
                stop_after_lateral_m = min(
                    args.max_lateral_pulse_travel_m,
                    remaining_stage_travel_m,
                    max(
                        0.04,
                        perception.ball_ground_range_m
                        * abs(cycle["fr_lane_bearing_errors"]["relative_error_rad"]),
                    ),
                )
                if stop_after_lateral_m <= 0.02:
                    result["verdict"] = "TRAVEL_LIMIT_REACHED"
                    break
                cycle["lateral_odom_target_m"] = stop_after_lateral_m
            sent, neutral, pulse_result = await joystick_pulse(
                pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"], lx=lx, ly=ly, rx=rx,
                duration_s=duration_s, direct_latch=direct_latch, args=args,
                odom_poses=odom_poses, start_pose=latest_pose,
                stop_after_forward_m=stop_after_forward_m,
                stop_after_yaw_rad=stop_after_yaw_rad,
                stop_after_lateral_m=stop_after_lateral_m,
            )
            command_packets += sent
            neutral_packets += neutral
            cycle.update({"duration_s": duration_s, "packets_sent": sent, "neutral_packets": neutral, "pulse_result": pulse_result})
            result["motion_commands_sent"] = result["motion_commands_sent"] or sent > 0
            if pulse_result == "physical_remote_preempted":
                result["verdict"] = "PHYSICAL_REMOTE_PREEMPTED"
                break
            if pulse_result.startswith("direct_remote_watchdog_lost:"):
                result["verdict"] = "DIRECT_REMOTE_WATCHDOG_LOST"
                result["reason"] = pulse_result
                break
            if pulse_result == "odom_stale_during_pulse":
                result["verdict"] = "ODOM_STALE"
                break
            if pulse_result.endswith("_odom_direction_mismatch"):
                result["verdict"] = "ODOM_COMMAND_DIRECTION_MISMATCH"
                result["reason"] = pulse_result
                break
            await asyncio.sleep(args.reobserve_settle_s)
        else:
            result["verdict"] = "CYCLE_LIMIT_REACHED"
        result["command_packets_sent"] = command_packets
        result["neutral_packets_sent"] = neutral_packets
        result["direct_remote_events"] = direct_latch.events
        result["final_odom_pose"] = odom_poses[-1] if odom_poses else None
        if odom_poses:
            result["final_travel_m"] = planar_distance(baseline, odom_poses[-1])
        return result
    finally:
        if command_packets > 0:
            try:
                await neutralize(connection.datachannel.pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"])
            except Exception:
                pass
        await connection.disconnect()
        if udp_transport is not None:
            udp_transport.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True, help="Go2 MCU WebRTC IP (현재 실물: 192.168.123.161)")
    parser.add_argument("--perception-url", default="http://127.0.0.1:8080/state.json")
    parser.add_argument("--tag-id", type=int, required=True)
    parser.add_argument(
        "--fr-lane-template", type=Path, default=None,
        help="capture_d435i_fr_lane_template.py가 만든 camera-visible FR lane template JSON",
    )
    parser.add_argument("--execute", action="store_true", help="없으면 read-only dry-run plan만 만든다")
    parser.add_argument("--operator-confirm", default="")
    parser.add_argument(
        "--direct-remote-status", type=Path, default=None,
        help="watch_go2_physical_remote_dds.py가 갱신하는 fresh direct-DDS watchdog JSON",
    )
    parser.add_argument("--direct-remote-udp-port", type=int, default=DIRECT_REMOTE_UDP_PORT)
    parser.add_argument("--direct-remote-max-age-s", type=float, default=DIRECT_REMOTE_MAX_STATUS_AGE_S)
    parser.add_argument("--direct-remote-hold-s", type=float, default=DIRECT_REMOTE_HOLD_S)
    parser.add_argument(
        "--virtual-echo-window", type=Path, default=DEFAULT_VIRTUAL_ECHO_WINDOW,
        help="watch_go2_physical_remote_dds.py와 공유하는 bounded WebRTC self-echo window JSON",
    )
    parser.add_argument("--connect-timeout-s", type=float, default=12.0)
    parser.add_argument("--http-timeout-s", type=float, default=0.25)
    parser.add_argument("--perception-max-age-s", type=float, default=MAX_PERCEPTION_AGE_S)
    parser.add_argument("--ball-detection-max-age-s", type=float, default=MAX_BALL_DETECTION_AGE_S)
    parser.add_argument("--min-ball-confidence", type=float, default=MIN_BALL_CONFIDENCE)
    parser.add_argument(
        "--min-ball-diameter-consistency-ratio", type=float,
        default=MIN_BALL_DIAMETER_CONSISTENCY_RATIO,
    )
    parser.add_argument(
        "--max-ball-diameter-consistency-ratio", type=float,
        default=MAX_BALL_DIAMETER_CONSISTENCY_RATIO,
    )
    parser.add_argument(
        "--allow-tagless-ball-kick", action="store_true",
        help=(
            "AprilTag/target line이 없을 때 captured FR lane의 ball bearing과 D435i depth만으로 "
            "접근한다. target 방향은 검증하지 못하는 명시 opt-in이다"
        ),
    )
    parser.add_argument("--observation-count", type=int, default=3)
    parser.add_argument("--observation-interval-s", type=float, default=0.10)
    parser.add_argument("--observation-timeout-s", type=float, default=OBSERVATION_TIMEOUT_S)
    parser.add_argument("--max-range-jitter-m", type=float, default=0.08)
    parser.add_argument("--max-bearing-jitter-rad", type=float, default=0.10)
    parser.add_argument("--stage-range-min-m", type=float, default=STAGE_RANGE_MIN_M)
    parser.add_argument("--stage-range-max-m", type=float, default=STAGE_RANGE_MAX_M)
    parser.add_argument(
        "--lane-axis-bearing-rad", type=float, default=None,
        help="ball→Tag 지면축이 평행해야 할 camera bearing. 미지정 시 captured template 절대 bearing",
    )
    parser.add_argument("--target-bearing-tolerance-rad", type=float, default=0.10)
    parser.add_argument(
        "--ball-bearing-tolerance-rad", type=float, default=0.12,
        help="FR ball bearing/ball-Tag relative bearing의 runtime 상한; template보다 완화하지 않는다",
    )
    parser.add_argument("--joystick-magnitude", type=float, default=JOYSTICK_MAGNITUDE)
    parser.add_argument(
        "--min-odom-stop-active-s", type=float, default=MIN_ODOM_STOP_ACTIVE_S,
        help="gait/body transient를 이동으로 오판하지 않도록 odometry stop을 금지할 최소 active 시간",
    )
    parser.add_argument(
        "--odom-stop-confirm-samples", type=int, default=ODOM_STOP_CONFIRM_SAMPLES,
        help="forward/yaw 목표 도달을 확정할 연속 새 odometry sample 수",
    )
    parser.add_argument(
        "--camera-stage-entry-slack-m", type=float, default=CAMERA_STAGE_ENTRY_SLACK_M,
        help="수 cm pulse 반복 대신 bounded final dock으로 넘길 camera staging 추가 여유",
    )
    parser.add_argument("--forward-pulse-s", type=float, default=FORWARD_PULSE_S)
    parser.add_argument(
        "--max-forward-pulse-travel-m", type=float, default=MAX_FORWARD_PULSE_TRAVEL_M,
        help="한 continuous forward pulse가 odometry로 이동할 수 있는 최대 거리",
    )
    parser.add_argument(
        "--enable-final-dock", action="store_true",
        help="camera staging 뒤 floor-plane 거리와 LiDAR odometry로 camera-blind final forward를 명시 arm한다",
    )
    parser.add_argument(
        "--camera-to-fr-forward-m", type=float, default=0.0,
        help=(
            "legacy fixed mode의 camera ground projection→FR center signed 거리. "
            "direct FR mode에서는 calibration 참고값일 뿐 거리 gate에는 쓰지 않는다"
        ),
    )
    parser.add_argument(
        "--use-direct-fr-kinematics", action="store_true",
        help=(
            "direct DDS LowState+URDF FK의 현재 FR 위치로 final gap을 계산하고, "
            "정지 뒤에도 fresh FK로 10–17cm 공 표면 간격을 재검증한다"
        ),
    )
    parser.add_argument(
        "--camera-body-forward-m", type=float, default=None,
        help=(
            "robot body origin→D435i lens/ground-ray origin forward 고정값. "
            "direct FR mode에서 필수이며 현재 FR FK로부터 매번 다시 만들면 안 된다"
        ),
    )
    parser.add_argument(
        "--camera-body-lateral-m", type=float, default=0.0,
        help="robot body origin→D435i optical center의 body +y(left) offset",
    )
    parser.add_argument(
        "--camera-body-yaw-rad", type=float, default=0.0,
        help="body +x에서 D435i optical +z로 향하는 signed yaw calibration",
    )
    parser.add_argument(
        "--fr-kinematics-max-age-s", type=float, default=FR_KINEMATICS_MAX_AGE_S,
    )
    parser.add_argument("--ball-radius-m", type=float, default=BALL_RADIUS_M)
    parser.add_argument(
        "--final-fr-to-ball-min-surface-gap-m", type=float,
        default=FINAL_FR_TO_BALL_MIN_SURFACE_GAP_M,
    )
    parser.add_argument(
        "--final-fr-to-ball-target-surface-gap-m", type=float,
        default=FINAL_FR_TO_BALL_TARGET_SURFACE_GAP_M,
    )
    parser.add_argument(
        "--final-fr-gap-tolerance-m", type=float, default=FINAL_FR_GAP_TOLERANCE_M,
    )
    parser.add_argument(
        "--final-fr-max-lateral-error-m", type=float,
        default=FINAL_FR_MAX_LATERAL_ERROR_M,
        help="정지 FR collision center와 공 중심의 body lateral 오차 상한",
    )
    parser.add_argument(
        "--final-fr-max-foot-speed-m-s", type=float, default=FINAL_FR_MAX_FOOT_SPEED_M_S,
    )
    parser.add_argument(
        "--fr-to-ball-forward-m", type=float, default=0.0,
        help="검증된 final kick pose에서 FR center에서 ball center까지 forward 거리",
    )
    parser.add_argument("--final-dock-max-m", type=float, default=FINAL_DOCK_MAX_M)
    parser.add_argument("--final-dock-max-duration-s", type=float, default=FINAL_DOCK_MAX_DURATION_S)
    parser.add_argument(
        "--final-gait-to-kick-clearance-m", type=float,
        default=FINAL_GAIT_TO_KICK_CLEARANCE_M,
        help="마지막 gait FR swing이 공에 닿지 않도록 kick 목표보다 먼저 멈출 추가 거리",
    )
    parser.add_argument("--final-settle-timeout-s", type=float, default=FINAL_SETTLE_TIMEOUT_S)
    parser.add_argument("--final-settle-window-s", type=float, default=FINAL_SETTLE_WINDOW_S)
    parser.add_argument(
        "--final-settle-max-planar-speed-m-s", type=float,
        default=FINAL_SETTLE_MAX_PLANAR_SPEED_M_S,
    )
    parser.add_argument(
        "--final-settle-max-yaw-speed-rad-s", type=float,
        default=FINAL_SETTLE_MAX_YAW_SPEED_RAD_S,
    )
    parser.add_argument(
        "--final-settle-max-odom-span-m", type=float, default=FINAL_SETTLE_MAX_ODOM_SPAN_M,
    )
    parser.add_argument(
        "--final-settle-max-odom-yaw-span-rad", type=float,
        default=FINAL_SETTLE_MAX_ODOM_YAW_SPAN_RAD,
    )
    parser.add_argument("--turn-pulse-s", type=float, default=TURN_PULSE_S)
    parser.add_argument(
        "--allow-lateral-search", action="store_true",
        help="FR toe→ball→Tag 상대 bearing 보정용 bounded lateral probe를 명시적으로 arm한다",
    )
    parser.add_argument(
        "--lateral-sign", type=float, choices=(-1.0, 1.0), default=1.0,
        help="첫 lateral probe의 lx 부호. probe 결과가 sign_confirmed일 때만 다음 실행에 재사용한다",
    )
    parser.add_argument(
        "--lateral-sign-confirmed", action="store_true",
        help="직전 LATERAL_PROBE_MEASURED 결과의 recommended_lateral_sign을 명시적으로 확인했음을 뜻한다",
    )
    parser.add_argument(
        "--lateral-probe-only", action="store_true",
        help="lateral pulse 1회와 재관측 1회만 수행한 뒤 추가 보행 없이 neutral로 끝낸다",
    )
    parser.add_argument("--lateral-pulse-s", type=float, default=LATERAL_PULSE_S)
    parser.add_argument(
        "--max-lateral-pulse-travel-m", type=float,
        default=MAX_LATERAL_PULSE_TRAVEL_M,
        help="continuous lateral pulse를 즉시 neutralize할 LiDAR odometry 횟변위 상한",
    )
    parser.add_argument("--lateral-min-improvement-rad", type=float, default=LATERAL_MIN_IMPROVEMENT_RAD)
    parser.add_argument("--max-lateral-probe-attempts", type=int, default=MAX_LATERAL_PROBE_ATTEMPTS)
    parser.add_argument("--reobserve-settle-s", type=float, default=REOBSERVE_SETTLE_S)
    parser.add_argument("--max-travel-m", type=float, default=MAX_TRAVEL_M)
    parser.add_argument(
        "--max-total-travel-m", type=float, default=MAX_TOTAL_TRAVEL_M,
        help="staging과 final dock을 합친 시작 pose 기준 planar displacement hard limit",
    )
    parser.add_argument("--max-cycles", type=int, default=MAX_CYCLES)
    parser.add_argument("--max-static-baseline-span-m", type=float, default=MAX_STATIC_BASELINE_SPAN_M)
    parser.add_argument(
        "--max-static-baseline-yaw-span-rad", type=float,
        default=MAX_STATIC_BASELINE_YAW_SPAN_RAD,
    )
    parser.add_argument("--max-odom-stale-s", type=float, default=MAX_ODOM_STALE_S)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("--execute에는 --operator-confirm {}가 정확히 필요합니다".format(CONFIRMATION))
    if args.execute and args.direct_remote_status is None:
        parser.error("--execute에는 fresh --direct-remote-status JSON이 필요합니다")
    if args.use_direct_fr_kinematics and args.direct_remote_status is None:
        parser.error("--use-direct-fr-kinematics에는 --direct-remote-status JSON이 필요합니다")
    if args.use_direct_fr_kinematics and not args.enable_final_dock:
        parser.error("--use-direct-fr-kinematics에는 --enable-final-dock이 필요합니다")
    if args.execute and args.fr_lane_template is None:
        parser.error("--execute에는 --fr-lane-template JSON이 필요합니다")
    if (args.lateral_sign_confirmed or args.lateral_probe_only) and not args.allow_lateral_search:
        parser.error("--lateral-sign-confirmed/--lateral-probe-only에는 --allow-lateral-search가 필요합니다")
    if args.lateral_sign_confirmed and args.lateral_probe_only:
        parser.error("--lateral-sign-confirmed와 --lateral-probe-only는 함께 사용할 수 없습니다")
    if args.allow_tagless_ball_kick and args.lateral_probe_only:
        parser.error("tagless ball-only mode에서는 Tag-relative lateral probe를 실행할 수 없습니다")
    if not 0.0 < args.joystick_magnitude <= 0.20:
        parser.error("joystick magnitude는 0보다 크고 0.20 이하여야 합니다")
    if not (
        0.20 <= args.min_ball_diameter_consistency_ratio < 1.0
        and 1.0 < args.max_ball_diameter_consistency_ratio <= 3.0
    ):
        parser.error("ball diameter consistency ratio의 min/max 범위가 유효하지 않습니다")
    if not 0.03 <= args.max_forward_pulse_travel_m <= 0.15:
        parser.error("max-forward-pulse-travel-m은 [0.03, 0.15] 범위여야 합니다")
    if not 0.50 <= args.forward_pulse_s <= 2.0:
        parser.error("forward-pulse-s는 [0.50, 2.0] 범위여야 합니다")
    if not 0.50 <= args.lateral_pulse_s <= 2.0:
        parser.error("lateral-pulse-s는 [0.50, 2.0] 범위여야 합니다")
    if not 0.03 <= args.max_lateral_pulse_travel_m <= 0.12:
        parser.error("max-lateral-pulse-travel-m은 [0.03, 0.12] 범위여야 합니다")
    if not 0.20 <= args.min_odom_stop_active_s <= 1.0:
        parser.error("--min-odom-stop-active-s는 [0.20, 1.0] 범위여야 합니다")
    if not 2 <= args.odom_stop_confirm_samples <= 10:
        parser.error("--odom-stop-confirm-samples는 [2, 10] 범위여야 합니다")
    if not 0.0 <= args.camera_stage_entry_slack_m <= 0.06:
        parser.error("--camera-stage-entry-slack-m은 [0.0, 0.06] 범위여야 합니다")
    if args.enable_final_dock and args.use_direct_fr_kinematics:
        if args.camera_body_forward_m is None or not 0.10 <= args.camera_body_forward_m <= 0.55:
            parser.error("direct FR final dock에는 --camera-body-forward-m [0.10, 0.55]가 필요합니다")
    elif args.enable_final_dock and not (
        -0.40 <= args.camera_to_fr_forward_m <= 0.40
        and 0.0 < args.fr_to_ball_forward_m <= 0.40
    ):
        parser.error("legacy final dock camera-to-FR은 signed, FR-to-ball은 양수 실측값이어야 합니다")
    if not -0.30 <= args.camera_body_lateral_m <= 0.30:
        parser.error("--camera-body-lateral-m은 [-0.30, 0.30] 범위여야 합니다")
    if not math.isfinite(args.camera_body_yaw_rad) or abs(args.camera_body_yaw_rad) > 0.35:
        parser.error("--camera-body-yaw-rad는 finite [-0.35, 0.35] 범위여야 합니다")
    if not 0.02 <= args.fr_kinematics_max_age_s <= 0.35:
        parser.error("--fr-kinematics-max-age-s는 [0.02, 0.35] 범위여야 합니다")
    if not 0.08 <= args.ball_radius_m <= 0.14:
        parser.error("--ball-radius-m은 [0.08, 0.14] 범위여야 합니다")
    if not (
        0.05 <= args.final_fr_to_ball_min_surface_gap_m <= 0.20
        and args.final_fr_to_ball_min_surface_gap_m
        <= args.final_fr_to_ball_target_surface_gap_m <= 0.25
    ):
        parser.error("FR→공 표면 최소/목표 gap 범위가 유효하지 않습니다")
    if not 0.0 <= args.final_fr_gap_tolerance_m <= 0.05:
        parser.error("--final-fr-gap-tolerance-m은 [0.0, 0.05] 범위여야 합니다")
    if not 0.01 <= args.final_fr_max_lateral_error_m <= 0.12:
        parser.error("--final-fr-max-lateral-error-m은 [0.01, 0.12] 범위여야 합니다")
    if not 0.01 <= args.final_fr_max_foot_speed_m_s <= 0.15:
        parser.error("--final-fr-max-foot-speed-m-s는 [0.01, 0.15] 범위여야 합니다")
    if not 0.05 <= args.final_dock_max_m <= FINAL_DOCK_MAX_M:
        parser.error("final-dock-max-m은 [0.05, 0.85] 범위여야 합니다")
    if not 1.0 <= args.final_dock_max_duration_s <= FINAL_DOCK_MAX_DURATION_S:
        parser.error("final-dock-max-duration-s는 [1.0, 6.0] 범위여야 합니다")
    if not 0.05 <= args.final_gait_to_kick_clearance_m <= 0.15:
        parser.error("--final-gait-to-kick-clearance-m은 [0.05, 0.15] 범위여야 합니다")
    if not 1.0 <= args.final_settle_timeout_s <= 3.0:
        parser.error("--final-settle-timeout-s는 [1.0, 3.0] 범위여야 합니다")
    if not 0.25 <= args.final_settle_window_s <= 1.0:
        parser.error("--final-settle-window-s는 [0.25, 1.0] 범위여야 합니다")
    if not (
        0.02 <= args.final_settle_max_planar_speed_m_s <= 0.12
        and 0.05 <= args.final_settle_max_yaw_speed_rad_s <= 0.20
        and 0.005 <= args.final_settle_max_odom_span_m <= 0.04
        and 0.01 <= args.final_settle_max_odom_yaw_span_rad <= 0.12
    ):
        parser.error("final settle speed/yaw/odom threshold 범위가 유효하지 않습니다")
    if not 0.20 <= args.max_total_travel_m <= MAX_TOTAL_TRAVEL_M:
        parser.error("--max-total-travel-m은 [0.20, 1.20] 범위여야 합니다")
    if args.max_total_travel_m < args.max_travel_m:
        parser.error("--max-total-travel-m은 --max-travel-m 이상이어야 합니다")
    if not 0.01 <= args.max_static_baseline_yaw_span_rad <= 0.20:
        parser.error("--max-static-baseline-yaw-span-rad는 [0.01, 0.20] 범위여야 합니다")
    if not 0.30 <= args.stage_range_min_m < args.stage_range_max_m <= 2.0:
        parser.error("stage range는 D435i valid range 안의 min < max여야 합니다")
    if (
        args.lane_axis_bearing_rad is not None
        and (not math.isfinite(args.lane_axis_bearing_rad) or abs(args.lane_axis_bearing_rad) > 0.35)
    ):
        parser.error("--lane-axis-bearing-rad는 finite [-0.35, 0.35] 범위여야 합니다")
    if not 0.01 <= args.target_bearing_tolerance_rad <= 0.20:
        parser.error("--target-bearing-tolerance-rad는 [0.01, 0.20] 범위여야 합니다")
    if not 0.01 <= args.ball_bearing_tolerance_rad <= 0.20:
        parser.error("--ball-bearing-tolerance-rad는 [0.01, 0.20] 범위여야 합니다")
    if args.observation_count < 3 or args.max_cycles < 1 or args.max_lateral_probe_attempts < 1:
        parser.error("observation-count는 3 이상이고 max-cycles/max-lateral-probe-attempts는 1 이상이어야 합니다")
    if min(
        args.forward_pulse_s, args.turn_pulse_s, args.lateral_pulse_s, args.lateral_min_improvement_rad,
        args.observation_interval_s, args.observation_timeout_s,
        args.reobserve_settle_s, args.max_travel_m,
        args.max_odom_stale_s, args.direct_remote_max_age_s, args.direct_remote_hold_s,
        args.ball_detection_max_age_s,
        args.max_forward_pulse_travel_m,
        args.max_lateral_pulse_travel_m,
    ) <= 0.0 or not 1 <= args.direct_remote_udp_port <= 65535:
        parser.error("pulse/settle/travel/stale/direct-remote 값은 양수 범위여야 합니다")

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "go2_mcf_d435i_ball_tag_camera_staging",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "robot_ip": args.robot_ip,
        "tag_id": args.tag_id,
        "motion_commands_sent": False,
    }
    started = time.monotonic()
    try:
        payload["result"] = asyncio.run(run(args))
        payload["motion_commands_sent"] = bool(payload["result"].get("motion_commands_sent", False))
        payload["verdict"] = payload["result"].get("verdict", "UNKNOWN")
    except Exception as error:
        payload["error"] = "{}: {}".format(type(error).__name__, error)
        payload["verdict"] = "FAIL"
    payload["elapsed_s"] = time.monotonic() - started
    output = args.output or Path("hardware_measurements") / (
        "go2_mcf_d435i_ball_tag_stage_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("MCF_D435I_BALL_TAG_STAGE_{} output={}".format(payload["verdict"], output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] in {
        "DRY_RUN_READY", "CAMERA_STAGING_READY", "FINAL_DOCKING_READY", "LATERAL_PROBE_MEASURED",
    } else 2


if __name__ == "__main__":
    raise SystemExit(main())
