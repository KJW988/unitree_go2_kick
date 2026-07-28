#!/usr/bin/env python3
"""Go2 WebRTC joystick의 bounded forward calibration을 LiDAR pose로 측정한다.

기본은 WebRTC sport state와 ``rt/utlidar/robot_pose``만 구독하는 read-only preflight다.
``--execute``를 주더라도 operator confirmation, 정상 stand state, 그리고 충분한 static
LiDAR pose baseline이 모두 있어야만 virtual joystick을 보낸다.

실행 한계는 의도적으로 고정되어 있다: forward ``ly=0.20``을 최대 2.0초, 50 Hz로 전송한다.
시작 LiDAR yaw 기준 forward progress가 0.20 m에 도달하면 즉시 neutral joystick으로 바꾸며,
어떤 정상/예외/interrupt 경로도 neutral 3회를 보낸다. LowCmd, MotionSwitcher, legacy
SportClient, `Move(1008)`, obstacle setting은 사용하지 않는다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
``examples/go2/data_channel/obstacles_avoid/obstacles_avoid.py``의 WebRTC
``rt/wirelesscontroller`` payload/50 Hz publishing을 채택했다. 여기에 이 프로젝트의
LiDAR pose progress gate와 0.20 m/2.0초 hard limit을 추가했다.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


JOYSTICK_LY = 0.20
COMMAND_RATE_HZ = 50.0
MAX_COMMAND_DURATION_S = 2.0
TARGET_FORWARD_M = 0.20
NEUTRAL_PACKET_COUNT = 3
STATE_SETTLE_S = 1.0
MAX_STATIC_BASELINE_SPAN_M = 0.04
MAX_ODOM_STALE_S = 0.50
CONFIRMATION = "MCF_ODOM_CALIBRATION_EMPTY_FLOOR_ESTOP_READY"


def unwrap_data(message: Any) -> dict[str, Any] | None:
    """WebRTC decoder wrapper와 direct dict 모두에서 data object를 얻는다."""
    if not isinstance(message, dict):
        return None
    data = message.get("data", message)
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except ValueError:
            return None
    return data if isinstance(data, dict) else None


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


def pose_summary(message: Any, elapsed_s: float) -> dict[str, Any] | None:
    """`rt/utlidar/robot_pose`의 known Pose/PoseStamped JSON shapes를 수용한다."""
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
        x = float(position["x"])
        y = float(position["y"])
        z = float(position.get("z", 0.0))
        qx = float(orientation["x"])
        qy = float(orientation["y"])
        qz = float(orientation["z"])
        qw = float(orientation["w"])
    except (KeyError, TypeError, ValueError):
        return None
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return {"elapsed_s": elapsed_s, "x": x, "y": y, "z": z, "yaw_rad": yaw}


def static_baseline(poses: list[dict[str, Any]]) -> tuple[dict[str, float] | None, float]:
    if len(poses) < 5:
        return None, float("inf")
    baseline = {
        key: float(statistics.median(float(pose[key]) for pose in poses))
        for key in ("x", "y", "z", "yaw_rad")
    }
    span = max(math.hypot(pose["x"] - baseline["x"], pose["y"] - baseline["y"]) for pose in poses)
    return baseline, span


def forward_progress_m(baseline: dict[str, float], pose: dict[str, Any]) -> float:
    dx = float(pose["x"]) - baseline["x"]
    dy = float(pose["y"]) - baseline["y"]
    return dx * math.cos(baseline["yaw_rad"]) + dy * math.sin(baseline["yaw_rad"])


def state_is_safe_to_walk(state: dict[str, Any] | None) -> tuple[bool, str]:
    if state is None:
        return False, "sportmodestate를 받지 못했습니다"
    if state.get("mode") not in (0, None):
        return False, "MCF mode가 idle이 아닙니다: {!r}".format(state.get("mode"))
    if state.get("progress") not in (0, None):
        return False, "다른 motion progress가 진행 중입니다: {!r}".format(state.get("progress"))
    height = state.get("body_height")
    if not isinstance(height, (int, float)) or not 0.24 <= height <= 0.40:
        return False, "body_height가 stand 범위를 벗어났습니다: {!r}".format(height)
    return True, "preflight_pass"


def publish_joystick(pub_sub: Any, topic: str, *, ly: float = 0.0) -> None:
    pub_sub.publish_without_callback(topic, {"lx": 0.0, "ly": ly, "rx": 0.0, "ry": 0.0, "keys": 0})


async def run(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.constants import RTC_TOPIC
    except ImportError as error:
        raise RuntimeError("unitree_webrtc_connect import 실패: {}".format(error)) from error

    connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.robot_ip)
    sport_states: list[dict[str, Any]] = []
    odom_poses: list[dict[str, Any]] = []
    connected_at = 0.0
    joystick_packets_sent = 0
    neutral_packets_sent = 0
    try:
        await asyncio.wait_for(connection.connect(), timeout=args.connect_timeout_s)
        connected_at = time.monotonic()

        def on_sport_state(message: Any) -> None:
            sport_states.append(sport_state_summary(message, time.monotonic() - connected_at))

        def on_robot_pose(message: Any) -> None:
            pose = pose_summary(message, time.monotonic() - connected_at)
            if pose is not None:
                odom_poses.append(pose)

        connection.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_sport_state)
        connection.datachannel.pub_sub.subscribe(RTC_TOPIC["ROBOTODOM"], on_robot_pose)
        await asyncio.sleep(STATE_SETTLE_S)
        initial_state = sport_states[-1] if sport_states else None
        state_ok, state_reason = state_is_safe_to_walk(initial_state)
        baseline, baseline_span_m = static_baseline(odom_poses)
        odom_ok = baseline is not None and baseline_span_m <= MAX_STATIC_BASELINE_SPAN_M
        reason = state_reason if not state_ok else (
            "odom_preflight_pass" if odom_ok else "LiDAR pose baseline unavailable/unstable"
        )
        result: dict[str, Any] = {
            "connected": True,
            "robot_ip": args.robot_ip,
            "preflight": {
                "ok": state_ok and odom_ok,
                "reason": reason,
                "sport_state": initial_state,
                "sport_state_count": len(sport_states),
                "odom_pose_count": len(odom_poses),
                "odom_baseline": baseline,
                "odom_static_span_m": baseline_span_m,
                "odom_static_span_threshold_m": MAX_STATIC_BASELINE_SPAN_M,
            },
            "command_contract": {
                "topic": "rt/wirelesscontroller via WebRTC bridge",
                "ly": JOYSTICK_LY,
                "target_forward_m": TARGET_FORWARD_M,
                "max_duration_s": MAX_COMMAND_DURATION_S,
                "rate_hz": COMMAND_RATE_HZ,
                "neutral_packets_after": NEUTRAL_PACKET_COUNT,
                "obstacle_avoidance_modified": False,
            },
            "execute": args.execute,
        }
        if not (state_ok and odom_ok):
            result["verdict"] = "PREFLIGHT_REJECTED"
            return result
        if not args.execute:
            result["motion_commands_sent"] = False
            result["verdict"] = "DRY_RUN_OK"
            return result

        command_started = time.monotonic()
        last_pose_count = len(odom_poses)
        last_odom_at = command_started
        interval_s = 1.0 / COMMAND_RATE_HZ
        termination = "MAX_DURATION_NEUTRALIZED"
        while time.monotonic() - command_started < MAX_COMMAND_DURATION_S:
            latest_pose = odom_poses[-1] if odom_poses else None
            if len(odom_poses) != last_pose_count:
                last_pose_count = len(odom_poses)
                last_odom_at = time.monotonic()
            if latest_pose is None or time.monotonic() - last_odom_at > MAX_ODOM_STALE_S:
                termination = "ODOM_STALE_NEUTRALIZED"
                break
            progress_m = forward_progress_m(baseline, latest_pose)
            if progress_m >= TARGET_FORWARD_M:
                termination = "TARGET_REACHED_NEUTRALIZED"
                break
            publish_joystick(connection.datachannel.pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"], ly=JOYSTICK_LY)
            joystick_packets_sent += 1
            await asyncio.sleep(interval_s)

        for _ in range(NEUTRAL_PACKET_COUNT):
            publish_joystick(connection.datachannel.pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"])
            neutral_packets_sent += 1
            await asyncio.sleep(interval_s)
        await asyncio.sleep(0.5)
        final_pose = odom_poses[-1] if odom_poses else None
        forward_m = forward_progress_m(baseline, final_pose) if final_pose else None
        planar_m = (
            math.hypot(final_pose["x"] - baseline["x"], final_pose["y"] - baseline["y"])
            if final_pose else None
        )
        result.update(
            {
                "motion_commands_sent": True,
                "joystick_packets_sent": joystick_packets_sent,
                "neutral_packets_sent": neutral_packets_sent,
                "termination": termination,
                "final_odom_pose": final_pose,
                "measured_forward_m": forward_m,
                "measured_planar_m": planar_m,
                "verdict": termination,
            }
        )
        return result
    finally:
        if joystick_packets_sent > 0:
            for _ in range(NEUTRAL_PACKET_COUNT - neutral_packets_sent):
                try:
                    publish_joystick(connection.datachannel.pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"])
                except Exception:
                    break
        await connection.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True, help="Go2 MCU WebRTC IP (현재 실물: 192.168.123.161)")
    parser.add_argument("--execute", action="store_true", help="없으면 read-only preflight만 실행")
    parser.add_argument("--operator-confirm", default="", help="실행 시 정확한 safety confirmation 문구")
    parser.add_argument("--connect-timeout-s", type=float, default=12.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.connect_timeout_s <= 0.0:
        parser.error("connect timeout은 양수여야 합니다")
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("실행에는 --operator-confirm {} 가 정확히 필요합니다".format(CONFIRMATION))

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "go2_mcf_webrtc_joystick_odom_calibration",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "robot_ip": args.robot_ip,
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
        "go2_mcf_webrtc_joystick_odom_calibration_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("MCF_JOYSTICK_ODOM_CALIBRATION_{} output={}".format(payload["verdict"], output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] in {"DRY_RUN_OK", "TARGET_REACHED_NEUTRALIZED", "MAX_DURATION_NEUTRALIZED"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
