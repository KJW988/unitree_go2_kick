#!/usr/bin/env python3
"""Go2 MCF의 App-equivalent WebRTC joystick 전진 burst를 proof 한다.

기본은 read-only preflight다. ``--execute``와 정확한 operator confirmation이 있을 때만
WebRTC bridge를 통해 ``rt/wirelesscontroller``에 작은 전진 joystick burst를 보낸다.
이것은 direct DDS publisher가 아니며, physical remote와 같은 App/MCF 입력 경로를
WebRTC bridge로 사용하는 방식이다. LowCmd, MotionSwitcher, legacy SportClient,
``Move(1008)``, obstacle-avoid disable은 사용하지 않는다.

실행 범위는 고정: ``ly=0.20``을 50 Hz로 0.40초만 보내고, 성공/실패/interrupt 모두
마지막에 neutral joystick을 세 번 보낸다. physical remote/E-stop은 항상 operator가
즉시 사용할 수 있어야 한다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
그 repository의 ``examples/go2/data_channel/obstacles_avoid/obstacles_avoid.py``에서
WebRTC ``RTC_TOPIC['WIRELESS_CONTROLLER']``의 joystick payload와 50 Hz burst/neutral
stop 규칙을 채택했다. 이 저장소에서는 원 예제의 0.9 joystick 대신 더 작은 0.20,
0.40초로 제한하고 obstacle avoidance를 변경하지 않는다.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


JOYSTICK_LY = 0.20
JOYSTICK_RATE_HZ = 50.0
JOYSTICK_DURATION_S = 0.40
NEUTRAL_PACKET_COUNT = 3
STATE_SETTLE_S = 1.0
CONFIRMATION = "MCF_JOYSTICK_EMPTY_FLOOR_ESTOP_READY"


def state_summary(message: Any, elapsed_s: float) -> dict[str, Any]:
    """가상 joystick 전송 전후의 MCF stand state만 최소 보존한다."""
    data = message.get("data", {}) if isinstance(message, dict) else {}
    if not isinstance(data, dict):
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
    velocity = state.get("velocity")
    if velocity is not None and max(abs(float(value)) for value in velocity) > 0.12:
        return False, "robot이 이미 움직이고 있습니다: {!r}".format(velocity)
    return True, "preflight_pass"


def publish_joystick(pub_sub: Any, topic: str, *, ly: float = 0.0) -> None:
    """공식 WebRTC example과 동일한 controller payload; x/z/key는 항상 neutral이다."""
    pub_sub.publish_without_callback(
        topic,
        {"lx": 0.0, "ly": ly, "rx": 0.0, "ry": 0.0, "keys": 0},
    )


async def run(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.constants import RTC_TOPIC
    except ImportError as error:
        raise RuntimeError("unitree_webrtc_connect import 실패: {}".format(error)) from error

    connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.robot_ip)
    states: list[dict[str, Any]] = []
    connected_at = 0.0
    joystick_packets_sent = 0
    neutral_packets_sent = 0
    try:
        await asyncio.wait_for(connection.connect(), timeout=args.connect_timeout_s)
        connected_at = time.monotonic()

        def on_state(message: Any) -> None:
            states.append(state_summary(message, time.monotonic() - connected_at))

        connection.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_state)
        await asyncio.sleep(STATE_SETTLE_S)
        preflight_state = states[-1] if states else None
        preflight_ok, preflight_reason = state_is_safe_to_walk(preflight_state)
        result: dict[str, Any] = {
            "connected": True,
            "robot_ip": args.robot_ip,
            "preflight": {
                "ok": preflight_ok,
                "reason": preflight_reason,
                "state": preflight_state,
                "state_count": len(states),
            },
            "command_contract": {
                "topic": "rt/wirelesscontroller via WebRTC bridge",
                "ly": JOYSTICK_LY,
                "duration_s": JOYSTICK_DURATION_S,
                "rate_hz": JOYSTICK_RATE_HZ,
                "neutral_packets_after": NEUTRAL_PACKET_COUNT,
                "obstacle_avoidance_modified": False,
            },
            "execute": args.execute,
        }
        if not preflight_ok:
            result["verdict"] = "PREFLIGHT_REJECTED"
            return result
        if not args.execute:
            result["motion_commands_sent"] = False
            result["verdict"] = "DRY_RUN_OK"
            return result

        interval_s = 1.0 / JOYSTICK_RATE_HZ
        packet_count = int(JOYSTICK_DURATION_S * JOYSTICK_RATE_HZ)
        for _ in range(packet_count):
            publish_joystick(connection.datachannel.pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"], ly=JOYSTICK_LY)
            joystick_packets_sent += 1
            await asyncio.sleep(interval_s)
        for _ in range(NEUTRAL_PACKET_COUNT):
            publish_joystick(connection.datachannel.pub_sub, RTC_TOPIC["WIRELESS_CONTROLLER"])
            neutral_packets_sent += 1
            await asyncio.sleep(interval_s)
        await asyncio.sleep(0.5)
        result.update(
            {
                "motion_commands_sent": True,
                "joystick_packets_sent": joystick_packets_sent,
                "neutral_packets_sent": neutral_packets_sent,
                "post_motion_state": states[-1] if states else None,
                "verdict": "JOYSTICK_BURST_SENT_NEUTRALIZED",
            }
        )
        return result
    finally:
        # Ctrl-C, request error, connection error도 neutral command를 우선 전송한다.
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
        "kind": "go2_mcf_webrtc_bounded_joystick_forward_proof",
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
        "go2_mcf_webrtc_joystick_proof_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("MCF_JOYSTICK_PROOF_{} output={}".format(payload["verdict"], output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] in {"DRY_RUN_OK", "JOYSTICK_BURST_SENT_NEUTRALIZED"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
