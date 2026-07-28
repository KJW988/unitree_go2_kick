#!/usr/bin/env python3
"""Go2 MCF WebRTC의 최소 전진 보행을 안전하게 proof 한다.

기본 동작은 read-only preflight이며 운동 명령을 전송하지 않는다. 실제 보행은 빈 바닥,
즉시 사용할 physical remote/E-stop, 그리고 operator의 육안 감시가 준비된 경우에만
``--execute --operator-confirm MCF_EMPTY_FLOOR_ESTOP_READY``를 함께 줘야 한다.

실행 시에도 명령 범위는 고정이다: 전진 ``0.05 m/s``를 10 Hz로 정확히 1.0초 전송한
뒤 반드시 MCF ``StopMove``를 request/reply로 호출한다. LowCmd, MotionSwitcher,
legacy SportClient와 obstacle-avoid service는 사용하지 않는다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
그 repository의 ``examples/go2/data_channel/sportmode_mcf/sportmode_mcf.py``에서
MCF ``Move``(1008)의 no-reply wire shape와 ``StopMove``(1003) request 형식만 채택했다.
이 저장소에서는 단발 proof의 안전 한계를 위해 API 범위와 속도/시간을 더 좁혔다.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import random
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


FORWARD_VX_M_S = 0.05
COMMAND_RATE_HZ = 10.0
COMMAND_DURATION_S = 1.0
STATE_SETTLE_S = 1.0
POST_STOP_OBSERVE_S = 0.75
CONFIRMATION = "MCF_EMPTY_FLOOR_ESTOP_READY"


def response_summary(response: Any) -> dict[str, Any]:
    """Request/reply 응답에서 상태 code와 비밀 아닌 data만 남긴다."""
    if not isinstance(response, dict):
        return {"response_type": type(response).__name__, "raw": repr(response)}
    data = response.get("data")
    if not isinstance(data, dict):
        return {"response_type": "dict", "data": data}
    header = data.get("header") if isinstance(data.get("header"), dict) else {}
    status = header.get("status") if isinstance(header.get("status"), dict) else {}
    return {
        "status_code": status.get("code"),
        "status_message": status.get("message", ""),
        "data": data.get("data", ""),
    }


def state_summary(message: Any, elapsed_s: float) -> dict[str, Any]:
    """MCF state에서 proof 판정에 필요한 field만 기록한다."""
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
        "gait_type": data.get("gait_type"),
        "body_height": data.get("body_height"),
        "velocity": velocity,
        "yaw_speed": data.get("yaw_speed"),
    }


def state_is_safe_to_walk(state: dict[str, Any] | None) -> tuple[bool, str]:
    """한 state sample만으로도 명백한 비-stand/동작 중 상태를 막는다."""
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


def move_no_reply(connection: Any, topic: str, api_id: int) -> None:
    """공식 MCF 예제와 같은 no-reply Move payload를 단 한 가지 값으로 전송한다."""
    generated_id = int(time.time() * 1000) % 2147483648 + random.randint(0, 1000)
    request_payload = {
        "header": {
            "identity": {"id": generated_id, "api_id": api_id},
            "policy": {"priority": 0, "noreply": True},
        },
        "parameter": json.dumps({"x": FORWARD_VX_M_S, "y": 0.0, "z": 0.0}),
        "binary": [],
    }
    connection.datachannel.pub_sub.publish_without_callback(topic, request_payload)


async def request_stop(connection: Any, topic: str, api_id: int, timeout_s: float) -> dict[str, Any]:
    """StopMove acknowledgement를 기다려 failure를 로그에 남긴다."""
    response = await asyncio.wait_for(
        connection.datachannel.pub_sub.publish_request_new(topic, {"api_id": api_id}),
        timeout=timeout_s,
    )
    return response_summary(response)


async def run(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF
    except ImportError as error:
        raise RuntimeError("unitree_webrtc_connect import 실패: {}".format(error)) from error

    connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.robot_ip)
    states: list[dict[str, Any]] = []
    connected_at = 0.0
    move_packets_sent = 0
    stop_result: dict[str, Any] | None = None
    move_started_at: float | None = None
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
                "vx_m_s": FORWARD_VX_M_S,
                "duration_s": COMMAND_DURATION_S,
                "rate_hz": COMMAND_RATE_HZ,
                "stop_api": "MCF StopMove (1003)",
            },
            "execute": args.execute,
        }
        if not preflight_ok:
            return result
        if not args.execute:
            result["motion_commands_sent"] = False
            result["verdict"] = "DRY_RUN_OK"
            return result

        # StopMove is also sent in finally. This first call clears only a stale prior gait command.
        result["pre_move_stop"] = await request_stop(
            connection, RTC_TOPIC["SPORT_MOD"], SPORT_CMD_MCF["StopMove"], args.request_timeout_s
        )
        if result["pre_move_stop"].get("status_code") != 0:
            result["motion_commands_sent"] = False
            result["verdict"] = "PRE_MOVE_STOP_REJECTED"
            return result

        move_started_at = time.monotonic()
        interval_s = 1.0 / COMMAND_RATE_HZ
        packet_count = int(COMMAND_DURATION_S * COMMAND_RATE_HZ)
        for _ in range(packet_count):
            move_no_reply(connection, RTC_TOPIC["SPORT_MOD"], SPORT_CMD_MCF["Move"])
            move_packets_sent += 1
            await asyncio.sleep(interval_s)
        stop_result = await request_stop(
            connection, RTC_TOPIC["SPORT_MOD"], SPORT_CMD_MCF["StopMove"], args.request_timeout_s
        )
        await asyncio.sleep(POST_STOP_OBSERVE_S)
        post_states = [state for state in states if state["elapsed_s"] >= (move_started_at - connected_at)]
        peak_vx = max(
            (abs(float(state["velocity"][0])) for state in post_states if state.get("velocity") is not None),
            default=0.0,
        )
        result.update(
            {
                "motion_commands_sent": True,
                "move_packets_sent": move_packets_sent,
                "stop_result": stop_result,
                "post_motion_state_count": len(post_states),
                "observed_peak_abs_vx_m_s": peak_vx,
                "verdict": "COMMAND_AND_STOP_ACKED" if stop_result.get("status_code") == 0 else "STOP_UNCONFIRMED",
            }
        )
        return result
    finally:
        # Any error/cancel after sending Move gets a best-effort StopMove before disconnecting.
        if move_packets_sent > 0 and stop_result is None:
            try:
                stop_result = await request_stop(
                    connection, RTC_TOPIC["SPORT_MOD"], SPORT_CMD_MCF["StopMove"], args.request_timeout_s
                )
            except Exception:
                pass
        await connection.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True, help="Go2 MCU WebRTC IP (현재 실물: 192.168.123.161)")
    parser.add_argument("--execute", action="store_true", help="없으면 read-only preflight만 실행")
    parser.add_argument("--operator-confirm", default="", help="실행 시 정확한 safety confirmation 문구")
    parser.add_argument("--connect-timeout-s", type=float, default=12.0)
    parser.add_argument("--request-timeout-s", type=float, default=3.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if min(args.connect_timeout_s, args.request_timeout_s) <= 0.0:
        parser.error("timeout은 양수여야 합니다")
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("실행에는 --operator-confirm {} 가 정확히 필요합니다".format(CONFIRMATION))

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "go2_mcf_webrtc_minimal_forward_walk_proof",
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
        "go2_mcf_webrtc_walk_proof_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("MCF_WALK_PROOF_{} output={}".format(payload["verdict"], output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] in {"DRY_RUN_OK", "COMMAND_AND_STOP_ACKED"} else 2


if __name__ == "__main__":
    raise SystemExit(main())
