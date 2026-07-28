#!/usr/bin/env python3
"""D435i 공/AprilTag와 LiDAR odometry로 Go2를 camera-visible staging 위치에만 둔다.

최종 FR kick lane, LowCmd, MotionSwitcher ownership, obstacle setting은 이 프로그램의
범위 밖이다. D435i camera→base→FR rigid transform이 아직 실측되지 않았으므로, 여기서의
``CAMERA_STAGING_READY``는 공과 Tag가 camera에서 보이는 안전한 다음 단계의 준비 상태일
뿐 FR foot 정렬이나 킥 허가가 아니다.

실행은 연속 보행이 아닌 ``짧은 virtual joystick pulse -> neutral 3회 -> 재관측``이다.
각 cycle에서 D435i state freshness, ball/Tag geometry, MCF stand state, LiDAR odometry,
start-pose 기준 travel hard limit을 모두 검사한다. 이 firmware의 WebRTC subscriber는
physical remote input을 전달하지 않으므로, 별도 direct-DDS watchdog의 fresh heartbeat와
input proof를 실행 전 요구한다. watchdog의 localhost UDP event 또는 상태 파일에서 실제
remote input이 보이면 다음 packet 전에 neutralize하고 이번 process를 종료한다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
``examples/go2/data_channel/obstacles_avoid/obstacles_avoid.py``의 App-equivalent
``rt/wirelesscontroller`` payload/50 Hz 전송을 채택했다. 원 예제의 키보드 연속 drive와
obstacle-toggle은 채택하지 않고, 이 프로젝트의 D435i/LiDAR gate와 pulse hard limits를
추가했다.
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
FORWARD_PULSE_S = 0.50
TURN_PULSE_S = 0.20
NEUTRAL_PACKET_COUNT = 3
STATE_SETTLE_S = 1.0
REOBSERVE_SETTLE_S = 0.50
MAX_STATIC_BASELINE_SPAN_M = 0.04
MAX_ODOM_STALE_S = 0.50
MAX_PERCEPTION_AGE_S = 0.35
MIN_BALL_CONFIDENCE = 0.015
STAGE_RANGE_MIN_M = 0.65
STAGE_RANGE_MAX_M = 0.85
MAX_TRAVEL_M = 0.35
MAX_CYCLES = 5
DIRECT_REMOTE_MAX_STATUS_AGE_S = 0.35
DIRECT_REMOTE_HOLD_S = 0.60
DIRECT_REMOTE_UDP_PORT = 18181
CONFIRMATION = "MCF_CAMERA_STAGE_CLEAR_FLOOR_ESTOP_READY"


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


@dataclass(frozen=True)
class Perception:
    ball_range_m: float
    ball_confidence: float
    ball_bearing_rad: float
    target_bearing_rad: float
    target_distance_m: float
    age_s: float


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


def fetch_perception(url: str, tag_id: int, timeout_s: float) -> Perception | None:
    """stream의 camera-frame ground geometry를 읽는다. base/FR 좌표로 변환하지 않는다."""
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (OSError, ValueError, urllib.error.URLError):
        return None
    if not isinstance(payload, dict) or not payload.get("ready"):
        return None
    ball = payload.get("ball")
    target = payload.get("target_line")
    if not isinstance(ball, dict) or not isinstance(target, dict):
        return None
    if int(target.get("tag_id", -1)) != tag_id:
        return None
    ball_ground = point3(ball.get("ground_camera_xyz_m"))
    target_unit = point3(target.get("unit_camera_xyz"))
    try:
        confidence = float(ball["confidence"])
        ball_range = float(ball["depth_range_m"])
        target_distance = float(target["distance_m"])
        stamp = float(payload["stamp_monotonic_s"])
    except (KeyError, TypeError, ValueError):
        return None
    if (
        ball_ground is None
        or target_unit is None
        or not all(math.isfinite(value) for value in (confidence, ball_range, target_distance, stamp))
        or ball_range <= 0.0
        or target_distance < 0.5
    ):
        return None
    # RealSense optical frame: +x image-right, +z forward. y는 floor plane vertical 성분이다.
    return Perception(
        ball_range_m=ball_range,
        ball_confidence=confidence,
        ball_bearing_rad=math.atan2(ball_ground[0], ball_ground[2]),
        target_bearing_rad=math.atan2(target_unit[0], target_unit[2]),
        target_distance_m=target_distance,
        age_s=time.monotonic() - stamp,
    )


async def stable_perception(args: argparse.Namespace) -> tuple[Perception | None, str]:
    samples: list[Perception] = []
    for index in range(args.observation_count):
        sample = fetch_perception(args.perception_url, args.tag_id, args.http_timeout_s)
        if sample is None:
            return None, "perception_missing"
        if sample.age_s > args.perception_max_age_s:
            return None, "perception_stale"
        if sample.ball_confidence < args.min_ball_confidence:
            return None, "ball_confidence_low"
        if not 0.30 <= sample.ball_range_m <= 3.0:
            return None, "ball_depth_out_of_range"
        samples.append(sample)
        if index + 1 < args.observation_count:
            await asyncio.sleep(args.observation_interval_s)
    anchor = samples[-1]
    if max(abs(sample.ball_range_m - anchor.ball_range_m) for sample in samples) > args.max_range_jitter_m:
        return None, "ball_range_unstable"
    if max(abs(angle_distance(sample.ball_bearing_rad, anchor.ball_bearing_rad)) for sample in samples) > args.max_bearing_jitter_rad:
        return None, "ball_bearing_unstable"
    if max(abs(angle_distance(sample.target_bearing_rad, anchor.target_bearing_rad)) for sample in samples) > args.max_bearing_jitter_rad:
        return None, "target_bearing_unstable"
    return anchor, "perception_stable"


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


def static_baseline(poses: list[dict[str, float]]) -> tuple[dict[str, float] | None, float]:
    if len(poses) < 5:
        return None, float("inf")
    baseline = {key: float(statistics.median(pose[key] for pose in poses)) for key in ("x", "y", "z", "yaw_rad")}
    span = max(math.hypot(pose["x"] - baseline["x"], pose["y"] - baseline["y"]) for pose in poses)
    return baseline, span


def planar_distance(baseline: dict[str, float], pose: dict[str, float]) -> float:
    return math.hypot(pose["x"] - baseline["x"], pose["y"] - baseline["y"])


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
    return True, "mcf_stand_preflight_pass"


@dataclass(frozen=True)
class DirectRemoteWatchdog:
    """별도 SDK environment의 direct DDS watcher가 남긴 상태다."""

    heartbeat_monotonic_s: float
    event_count: int
    last_active_monotonic_s: float | None
    last_event: dict[str, Any] | None


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
    path: Path | None, *, max_age_s: float, hold_s: float
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
    except (KeyError, TypeError, ValueError):
        return None, "direct_remote_watchdog_missing_fields"
    if payload.get("ready") is not True or payload.get("motion_commands_sent") is not False:
        return None, "direct_remote_watchdog_not_read_only_ready"
    now = time.monotonic()
    if not math.isfinite(heartbeat) or now - heartbeat > max_age_s:
        return None, "direct_remote_watchdog_stale"
    if event_count < 1 or last_active is None or not math.isfinite(last_active):
        return None, "direct_remote_input_not_proven"
    if now - last_active < hold_s:
        return None, "physical_remote_active"
    last_event = payload.get("last_event")
    return DirectRemoteWatchdog(heartbeat, event_count, last_active, last_event if isinstance(last_event, dict) else None), \
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


def action_for(perception: Perception, args: argparse.Namespace, lane: FrLaneTemplate) -> tuple[str, float, float, float]:
    """FR template에 대해 yaw/forward만 제한적으로 보정한다.

    ball bearing을 0으로 맞추지 않는다. FR의 lateral offset 때문에 valid kick lane의 ball은
    camera image 중앙에서 벗어날 수 있다. 두 bearing error의 부호가 반대면 yaw만으로는
    해소할 수 없으므로 lateral/base calibration 전에는 이동을 금지한다.
    """
    ball_error = angle_distance(perception.ball_bearing_rad, lane.desired_ball_bearing_rad)
    target_error = angle_distance(perception.target_bearing_rad, lane.desired_target_bearing_rad)
    ball_aligned = abs(ball_error) <= lane.ball_bearing_tolerance_rad
    target_aligned = abs(target_error) <= lane.target_bearing_tolerance_rad
    if not ball_aligned or not target_aligned:
        if not ball_aligned and not target_aligned and ball_error * target_error < 0.0:
            return "fr_lane_lateral_alignment_required", 0.0, 0.0, 0.0
        correction = (
            ball_error if not target_aligned else target_error if not ball_aligned
            else 0.5 * (ball_error + target_error)
        )
        # Upstream example convention: +rx left turn, -rx right turn.
        return "turn_to_fr_lane", 0.0, 0.0, -math.copysign(args.joystick_magnitude, correction)
    if perception.ball_range_m < lane.desired_ball_range_m - lane.range_tolerance_m:
        return "ball_too_close_no_reverse", 0.0, 0.0, 0.0
    if perception.ball_range_m <= lane.desired_ball_range_m + lane.range_tolerance_m:
        return "camera_staging_ready", 0.0, 0.0, 0.0
    return "forward", 0.0, args.joystick_magnitude, 0.0


def publish_joystick(pub_sub: Any, topic: str, *, lx: float = 0.0, ly: float = 0.0, rx: float = 0.0) -> None:
    pub_sub.publish_without_callback(topic, {"lx": lx, "ly": ly, "rx": rx, "ry": 0.0, "keys": 0})


async def neutralize(pub_sub: Any, topic: str) -> int:
    for _ in range(NEUTRAL_PACKET_COUNT):
        publish_joystick(pub_sub, topic)
        await asyncio.sleep(1.0 / COMMAND_RATE_HZ)
    return NEUTRAL_PACKET_COUNT


async def joystick_pulse(
    pub_sub: Any, topic: str, *, lx: float, ly: float, rx: float, duration_s: float,
    direct_latch: DirectRemoteLatch, args: argparse.Namespace,
) -> tuple[int, int, str]:
    """partial burst도 neutralize한다. direct DDS remote input은 다음 packet 전에 stop한다."""
    packets = 0
    termination = "pulse_complete"
    try:
        for _ in range(math.ceil(duration_s * COMMAND_RATE_HZ)):
            watchdog, watchdog_reason = load_direct_remote_watchdog(
                args.direct_remote_status,
                max_age_s=args.direct_remote_max_age_s,
                hold_s=args.direct_remote_hold_s,
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
        baseline, baseline_span_m = static_baseline(odom_poses)
        odom_ok = baseline is not None and baseline_span_m <= args.max_static_baseline_span_m
        perception, perception_reason = await stable_perception(args)
        watchdog, watchdog_reason = load_direct_remote_watchdog(
            args.direct_remote_status,
            max_age_s=args.direct_remote_max_age_s,
            hold_s=args.direct_remote_hold_s,
        )
        direct_remote_ok = (not args.execute) or (watchdog is not None and not direct_latch.tripped())
        result: dict[str, Any] = {
            "connected": True,
            "robot_ip": args.robot_ip,
            "execute": args.execute,
            "motion_commands_sent": False,
            "preflight": {
                "ok": state_ok and odom_ok and perception is not None and lane_template is not None and direct_remote_ok,
                "mcf_state_reason": state_reason,
                "sport_state": initial_state,
                "odom_baseline": baseline,
                "odom_static_span_m": baseline_span_m,
                "odom_static_span_threshold_m": args.max_static_baseline_span_m,
                "perception_reason": perception_reason,
                "perception": None if perception is None else asdict(perception),
                "fr_lane_template_reason": template_reason,
                "fr_lane_template": None if lane_template is None else asdict(lane_template),
                "direct_remote_watchdog_reason": watchdog_reason,
                "direct_remote_watchdog": None if watchdog is None else asdict(watchdog),
                "direct_remote_events_before_command": direct_latch.events,
            },
            "command_contract": {
                "transport": "WebRTC rt/wirelesscontroller only",
                "physical_remote_guard": "direct DDS watcher heartbeat + localhost UDP event",
                "joystick_magnitude": args.joystick_magnitude,
                "rate_hz": COMMAND_RATE_HZ,
                "forward_pulse_s": args.forward_pulse_s,
                "turn_pulse_s": args.turn_pulse_s,
                "neutral_packets_after_each_pulse": NEUTRAL_PACKET_COUNT,
                "max_travel_m": args.max_travel_m,
                "max_cycles": args.max_cycles,
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
        first_action = action_for(perception, args, lane_template)
        if not args.execute:
            result["dry_run_next_action"] = {"reason": first_action[0], "joystick": list(first_action[1:])}
            result["verdict"] = "DRY_RUN_READY"
            return result

        for cycle_index in range(args.max_cycles):
            watchdog, watchdog_reason = load_direct_remote_watchdog(
                args.direct_remote_status,
                max_age_s=args.direct_remote_max_age_s,
                hold_s=args.direct_remote_hold_s,
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
            if latest_pose is None or time.monotonic() - connected_at - latest_pose["elapsed_s"] > args.max_odom_stale_s:
                result["verdict"] = "ODOM_STALE"
                break
            travel_m = planar_distance(baseline, latest_pose)
            if travel_m >= args.max_travel_m:
                result["verdict"] = "TRAVEL_LIMIT_REACHED"
                break
            reason, lx, ly, rx = action_for(perception, args, lane_template)
            cycle: dict[str, Any] = {
                "index": cycle_index,
                "perception": asdict(perception),
                "start_odom": latest_pose,
                "travel_m": travel_m,
                "reason": reason,
                "joystick": {"lx": lx, "ly": ly, "rx": rx},
            }
            result["cycles"].append(cycle)
            if reason == "camera_staging_ready":
                result["verdict"] = "CAMERA_STAGING_READY"
                break
            if reason in ("ball_too_close_no_reverse", "fr_lane_lateral_alignment_required"):
                result["verdict"] = reason.upper()
                break
            duration_s = args.forward_pulse_s if reason == "forward" else args.turn_pulse_s
            sent, neutral, pulse_result = await joystick_pulse(
                pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"], lx=lx, ly=ly, rx=rx,
                duration_s=duration_s, direct_latch=direct_latch, args=args,
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
    parser.add_argument("--connect-timeout-s", type=float, default=12.0)
    parser.add_argument("--http-timeout-s", type=float, default=0.25)
    parser.add_argument("--perception-max-age-s", type=float, default=MAX_PERCEPTION_AGE_S)
    parser.add_argument("--min-ball-confidence", type=float, default=MIN_BALL_CONFIDENCE)
    parser.add_argument("--observation-count", type=int, default=3)
    parser.add_argument("--observation-interval-s", type=float, default=0.10)
    parser.add_argument("--max-range-jitter-m", type=float, default=0.08)
    parser.add_argument("--max-bearing-jitter-rad", type=float, default=0.10)
    parser.add_argument("--stage-range-min-m", type=float, default=STAGE_RANGE_MIN_M)
    parser.add_argument("--stage-range-max-m", type=float, default=STAGE_RANGE_MAX_M)
    parser.add_argument("--target-bearing-tolerance-rad", type=float, default=0.10)
    parser.add_argument("--ball-bearing-tolerance-rad", type=float, default=0.12)
    parser.add_argument("--joystick-magnitude", type=float, default=JOYSTICK_MAGNITUDE)
    parser.add_argument("--forward-pulse-s", type=float, default=FORWARD_PULSE_S)
    parser.add_argument("--turn-pulse-s", type=float, default=TURN_PULSE_S)
    parser.add_argument("--reobserve-settle-s", type=float, default=REOBSERVE_SETTLE_S)
    parser.add_argument("--max-travel-m", type=float, default=MAX_TRAVEL_M)
    parser.add_argument("--max-cycles", type=int, default=MAX_CYCLES)
    parser.add_argument("--max-static-baseline-span-m", type=float, default=MAX_STATIC_BASELINE_SPAN_M)
    parser.add_argument("--max-odom-stale-s", type=float, default=MAX_ODOM_STALE_S)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("--execute에는 --operator-confirm {}가 정확히 필요합니다".format(CONFIRMATION))
    if args.execute and args.direct_remote_status is None:
        parser.error("--execute에는 fresh --direct-remote-status JSON이 필요합니다")
    if args.execute and args.fr_lane_template is None:
        parser.error("--execute에는 --fr-lane-template JSON이 필요합니다")
    if not 0.0 < args.joystick_magnitude <= 0.20:
        parser.error("joystick magnitude는 0보다 크고 0.20 이하여야 합니다")
    if not 0.30 <= args.stage_range_min_m < args.stage_range_max_m <= 2.0:
        parser.error("stage range는 D435i valid range 안의 min < max여야 합니다")
    if args.observation_count < 3 or args.max_cycles < 1:
        parser.error("observation-count는 3 이상이고 max-cycles는 1 이상이어야 합니다")
    if min(
        args.forward_pulse_s, args.turn_pulse_s, args.reobserve_settle_s, args.max_travel_m,
        args.max_odom_stale_s, args.direct_remote_max_age_s, args.direct_remote_hold_s,
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
    return 0 if payload["verdict"] in {"DRY_RUN_READY", "CAMERA_STAGING_READY"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
