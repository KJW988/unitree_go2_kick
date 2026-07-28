#!/usr/bin/env python3
"""Go2 MCF WebRTC 경로에서 physical remote 입력을 read-only로 확인한다.

이 probe는 WebRTC bridge의 ``rt/wirelesscontroller``만 구독하며 publisher, LowCmd,
MotionSwitcher, Sport API, obstacle 설정을 전혀 사용하지 않는다. operator가 probe의
READY 뒤에 physical remote stick 또는 버튼을 한 번 입력하면, 후속 camera-staging
walker가 사용할 수 있는 preempt evidence JSON을 남긴다.

출처: legion1581/unitree_webrtc_connect v2.1.2,
https://github.com/legion1581/unitree_webrtc_connect
그 repository의 ``RTC_TOPIC['WIRELESS_CONTROLLER']`` topic 정의와 WebRTC subscription
방식을 채택했다. 이 프로젝트에서는 motion publisher를 의도적으로 추가하지 않았다.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_DURATION_S = 12.0
DEADZONE = 0.15


def unwrap_data(message: Any) -> dict[str, Any] | None:
    """WebRTC decoder wrapper와 direct dict 모두에서 controller payload를 얻는다."""
    if not isinstance(message, dict):
        return None
    data = message.get("data", message)
    if isinstance(data, str):
        try:
            data = json.loads(data)
        except ValueError:
            return None
    return data if isinstance(data, dict) else None


def active_controller(data: dict[str, Any], deadzone: float) -> tuple[bool, dict[str, float | int]] | None:
    """neutral packet은 버리고 실제 stick/button input만 보존한다."""
    try:
        sample: dict[str, float | int] = {
            key: float(data.get(key, 0.0)) for key in ("lx", "ly", "rx", "ry")
        }
        sample["keys"] = int(data.get("keys", 0))
    except (TypeError, ValueError):
        return None
    active = max(abs(float(sample[key])) for key in ("lx", "ly", "rx", "ry")) > deadzone
    return active or int(sample["keys"]) != 0, sample


async def run(args: argparse.Namespace) -> dict[str, Any]:
    try:
        from unitree_webrtc_connect import UnitreeWebRTCConnection, WebRTCConnectionMethod
        from unitree_webrtc_connect.constants import RTC_TOPIC
    except ImportError as error:
        raise RuntimeError("unitree_webrtc_connect import 실패: {}".format(error)) from error

    connection = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=args.robot_ip)
    observed: list[dict[str, Any]] = []
    connected_at = 0.0
    try:
        await asyncio.wait_for(connection.connect(), timeout=args.connect_timeout_s)
        connected_at = time.monotonic()

        def on_wireless(message: Any) -> None:
            data = unwrap_data(message)
            if data is None:
                return
            parsed = active_controller(data, args.deadzone)
            if parsed is None:
                return
            active, sample = parsed
            if active:
                observed.append({"elapsed_s": time.monotonic() - connected_at, **sample})

        connection.datachannel.pub_sub.subscribe(RTC_TOPIC["WIRELESS_CONTROLLER"], on_wireless)
        print(
            "REMOTE_PREEMPT_PROBE_READY duration_s={} deadzone={}; "
            "이제 physical remote stick/button을 한 번 입력하세요.".format(args.duration_s, args.deadzone),
            flush=True,
        )
        await asyncio.sleep(args.duration_s)
        return {
            "connected": True,
            "robot_ip": args.robot_ip,
            "motion_commands_sent": False,
            "command_contract": "read_only WebRTC rt/wirelesscontroller subscription",
            "duration_s": args.duration_s,
            "deadzone": args.deadzone,
            "physical_input_events": observed,
            "physical_input_observed": bool(observed),
            "verdict": "PHYSICAL_INPUT_OBSERVED" if observed else "NO_PHYSICAL_INPUT_OBSERVED",
        }
    finally:
        await connection.disconnect()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True, help="Go2 MCU WebRTC IP (현재 실물: 192.168.123.161)")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--deadzone", type=float, default=DEADZONE)
    parser.add_argument("--connect-timeout-s", type=float, default=12.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.duration_s <= 0.0 or args.connect_timeout_s <= 0.0:
        parser.error("duration/connect timeout은 양수여야 합니다")
    if not 0.0 < args.deadzone < 1.0:
        parser.error("deadzone은 0과 1 사이여야 합니다")

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "read_only_go2_mcf_webrtc_remote_preempt_probe",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "robot_ip": args.robot_ip,
        "motion_commands_sent": False,
    }
    started = time.monotonic()
    try:
        payload["result"] = asyncio.run(run(args))
        payload["verdict"] = payload["result"].get("verdict", "UNKNOWN")
    except Exception as error:
        payload["error"] = "{}: {}".format(type(error).__name__, error)
        payload["verdict"] = "FAIL"
    payload["elapsed_s"] = time.monotonic() - started

    output = args.output or Path("hardware_measurements") / (
        "go2_mcf_webrtc_remote_preempt_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("MCF_WEBRTC_REMOTE_PREEMPT_{} output={}".format(payload["verdict"], output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] == "PHYSICAL_INPUT_OBSERVED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
