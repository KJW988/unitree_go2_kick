#!/usr/bin/env python3
"""Go2 MCF WebRTC data-channel을 **읽기 전용**으로 검증한다.

이 probe는 Unitree Go App과 같은 WebRTC signaling/data-channel transport로 연결한 뒤
MCF ``GetState``(api id 1034)만 request/response로 호출하고 low-frequency
``rt/lf/sportmodestate``를 잠깐 구독한다. ``Move``, ``SwitchJoystick``,
``MotionSwitcher``, ``LowCmd``와 모든 wireless-controller publisher를 만들지 않는다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
그 프로젝트의 Go2 firmware >=1.1.7 MCF ``SPORT_CMD_MCF``와
``RTC_TOPIC['SPORT_MOD']`` request 형식을 읽기 전용으로만 채택했다. 이 저장소는
명령 메뉴/joystick publishing을 의도적으로 포함하지 않는다.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_STATE_KEYS = (
    "state",
    "bodyHeight",
    "speedLevel",
    "gait",
    "continuousGait",
    "economicGait",
)


def _response_summary(response: Any) -> dict[str, Any]:
    """WebRTC library의 request result에서 필요한 non-secret 정보만 남긴다."""
    if not isinstance(response, dict):
        return {"response_type": type(response).__name__, "raw": repr(response)}
    data = response.get("data")
    if not isinstance(data, dict):
        return {"response_type": "dict", "data": data}
    header = data.get("header") if isinstance(data.get("header"), dict) else {}
    status = header.get("status") if isinstance(header.get("status"), dict) else {}
    raw_data = data.get("data", "")
    parsed: Any = raw_data
    if isinstance(raw_data, str):
        try:
            parsed = json.loads(raw_data)
        except ValueError:
            pass
    return {
        "status_code": status.get("code"),
        "status_message": status.get("message", ""),
        "data": parsed,
    }


def _sport_state_summary(message: Any) -> dict[str, Any]:
    """수신 state의 보행 관련 필드만 기록한다; raw message 전체는 보존하지 않는다."""
    data = message.get("data", {}) if isinstance(message, dict) else {}
    if not isinstance(data, dict):
        return {"message_type": type(message).__name__}
    return {
        key: data.get(key)
        for key in (
            "mode",
            "progress",
            "gait_type",
            "body_height",
            "velocity",
            "yaw_speed",
            "range_obstacle",
        )
    }


async def run_probe(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD_MCF
    except ImportError as error:
        raise RuntimeError(
            "unitree_webrtc_connect가 현재 environment에 없습니다. "
            "이 probe 전용 project-local environment를 먼저 준비해야 합니다: {}".format(error)
        ) from error

    connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.robot_ip)
    received_states: list[dict[str, Any]] = []
    try:
        await asyncio.wait_for(connection.connect(), timeout=args.connect_timeout_s)

        def on_sport_state(message: Any) -> None:
            received_states.append(_sport_state_summary(message))

        connection.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], on_sport_state)
        request = {
            "api_id": SPORT_CMD_MCF["GetState"],
            "parameter": list(DEFAULT_STATE_KEYS),
        }
        response = await asyncio.wait_for(
            connection.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], request),
            timeout=args.request_timeout_s,
        )
        await asyncio.sleep(args.observe_s)
        return {
            "connected": True,
            "robot_ip": args.robot_ip,
            "mcf_get_state": _response_summary(response),
            "lf_sportmodestate_count": len(received_states),
            "lf_sportmodestate_last": received_states[-1] if received_states else None,
        }
    finally:
        await connection.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True, help="Unitree App이 연결하는 Go2 LAN IP")
    parser.add_argument("--connect-timeout-s", type=float, default=12.0)
    parser.add_argument("--request-timeout-s", type=float, default=5.0)
    parser.add_argument("--observe-s", type=float, default=3.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if min(args.connect_timeout_s, args.request_timeout_s, args.observe_s) <= 0.0:
        parser.error("모든 timeout/observe 값은 양수여야 합니다")

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "read_only_go2_mcf_webrtc_probe",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "robot_ip": args.robot_ip,
        "motion_commands_sent": False,
    }
    started = time.monotonic()
    try:
        payload["result"] = asyncio.run(run_probe(args))
        payload["verdict"] = "PASS"
    except Exception as error:
        payload["error"] = "{}: {}".format(type(error).__name__, error)
        payload["verdict"] = "FAIL"
    payload["elapsed_s"] = time.monotonic() - started

    output = args.output or Path("hardware_measurements") / (
        "go2_mcf_webrtc_probe_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("MCF_WEBRTC_PROBE_{} output={}".format("OK" if payload["verdict"] == "PASS" else "FAILED", output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
