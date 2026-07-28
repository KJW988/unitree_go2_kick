#!/usr/bin/env python3
"""명시 arm된 rough camera staging 뒤 기존 FR LowCmd harness를 한 번만 실행한다.

이 runner는 WebRTC virtual joystick으로 공/Tag를 재관측하며 bounded staging을 끝낸 뒤,
기존 ``live_baseline_fr_preset.py``를 별도 SDK Python으로 실행한다. LowCmd의 direct
ReleaseMode + full-gain baseline stream 경로는 harness에서 이미 별도 검증한 구현을 그대로
호출하며, 이 파일은 LowCmd packet, gain, trajectory를 새로 만들지 않는다.

MCF로의 자동 handback은 수행하지 않는다. kick child가 ``--kick-hold-after-s``를 마치고
종료하면 LowCmd stream도 끝나므로, 이후 자세/ownership 복구는 operator가 책임진다.
``--allow-rough-kick``은 strict FR template 오차가 남은 camera staging에서도 kick child를
호출하는 명시적 opt-in이며, target hit을 보장하지 않는다.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
STAGE_SCRIPT = Path(__file__).with_name("stage_go2_mcf_ball_tag_webrtc.py")
HARNESS_SCRIPT = Path(__file__).with_name("live_baseline_fr_preset.py")
CONFIRMATION = "ROUGH_STAGE_THEN_FR_KICK_ESTOP_READY"


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("stage output JSON root가 object가 아닙니다")
    return payload


def stage_command(args: argparse.Namespace, output: Path) -> list[str]:
    command = [
        sys.executable, str(STAGE_SCRIPT),
        "--robot-ip", args.robot_ip,
        "--tag-id", str(args.tag_id),
        "--fr-lane-template", str(args.fr_lane_template),
        "--perception-url", args.perception_url,
        "--direct-remote-status", str(args.direct_remote_status),
        "--allow-lateral-search",
        "--max-cycles", str(args.max_stage_cycles),
        "--max-travel-m", str(args.max_stage_travel_m),
        "--output", str(output),
    ]
    if args.execute:
        command.extend(["--execute", "--operator-confirm", "MCF_CAMERA_STAGE_CLEAR_FLOOR_ESTOP_READY"])
    return command


def harness_command(args: argparse.Namespace) -> list[str]:
    return [
        str(args.lowcmd_python), str(HARNESS_SCRIPT),
        "--interface", args.interface,
        "--trajectory", str(args.trajectory),
        "--kp", str(args.kp),
        "--kd", str(args.kd),
        "--execute",
        "--release-motion-owner",
        "--release-without-stand-down",
        "--handoff-blend-s", str(args.handoff_blend_s),
        "--prehold-s", str(args.prehold_s),
        "--preset-time-scale", str(args.preset_time_scale),
        "--fr-swing-scale", str(args.fr_swing_scale),
        "--hold-after-s", str(args.kick_hold_after_s),
        "--operator-confirm", "HARNESS_ESTOP_READY",
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--tag-id", type=int, required=True)
    parser.add_argument("--fr-lane-template", type=Path, required=True)
    parser.add_argument("--direct-remote-status", type=Path, required=True)
    parser.add_argument("--perception-url", default="http://127.0.0.1:8080/state.json")
    parser.add_argument("--max-stage-cycles", type=int, default=5)
    parser.add_argument("--max-stage-travel-m", type=float, default=0.35)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--lowcmd-python", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--kp", type=float, required=True)
    parser.add_argument("--kd", type=float, required=True)
    parser.add_argument("--handoff-blend-s", type=float, default=1.2)
    parser.add_argument("--prehold-s", type=float, default=1.0)
    parser.add_argument("--preset-time-scale", type=float, default=1.0)
    parser.add_argument("--fr-swing-scale", type=float, default=1.0)
    parser.add_argument(
        "--kick-hold-after-s", type=float, default=None,
        help="kick 뒤 full-gain baseline LowCmd hold 시간. execute에서는 0보다 커야 한다",
    )
    parser.add_argument(
        "--allow-rough-kick", action="store_true",
        help="strict FR template 오차가 남아도 rough staging 뒤 kick child 호출을 명시 허용한다",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--operator-confirm", default="")
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("--execute에는 --operator-confirm {}가 정확히 필요합니다".format(CONFIRMATION))
    if args.max_stage_cycles < 1 or not 0.0 < args.max_stage_travel_m <= 0.35:
        parser.error("max-stage-cycles는 1 이상, max-stage-travel-m은 (0, 0.35]여야 합니다")
    if args.execute and (args.kick_hold_after_s is None or args.kick_hold_after_s <= 0.0):
        parser.error("--execute에는 양수 --kick-hold-after-s가 필요합니다")
    if not args.lowcmd_python.is_file() or not args.trajectory.is_file():
        parser.error("--lowcmd-python과 --trajectory는 존재하는 파일이어야 합니다")

    started = time.monotonic()
    output = args.output or Path("hardware_measurements") / (
        "go2_rough_stage_then_fr_kick_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    stage_output = output.with_name(output.stem + "_stage.json")
    result: dict[str, Any] = {
        "schema_version": 1,
        "kind": "go2_rough_stage_then_existing_fr_lowcmd_harness",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "motion_commands_sent": False,
        "automatic_mcf_handback": False,
        "execute": args.execute,
        "stage_output": str(stage_output),
    }
    try:
        command = stage_command(args, stage_output)
        print("ROUGH_STAGE_START execute={} max_cycles={} max_travel_m={}".format(
            args.execute, args.max_stage_cycles, args.max_stage_travel_m,
        ), flush=True)
        stage_return = subprocess.run(command, cwd=ROOT).returncode
        result["stage_returncode"] = stage_return
        stage = read_json(stage_output)
        result["stage"] = stage
        result["motion_commands_sent"] = bool(stage.get("motion_commands_sent", False))
        if not args.execute:
            result["verdict"] = "DRY_RUN_STAGE_COMPLETE_NO_LOWCMD"
            return 0
        if stage.get("verdict") != "CAMERA_STAGING_READY":
            result["verdict"] = "STAGE_NOT_READY_NO_LOWCMD"
            return 2
        stage_result = stage.get("result")
        kick_ready = stage_result.get("kick_ready") if isinstance(stage_result, dict) else None
        strict_ready = isinstance(kick_ready, dict) and kick_ready.get("eligible") is True
        result["strict_fr_lane_ready"] = strict_ready
        if not strict_ready and not args.allow_rough_kick:
            result["verdict"] = "ROUGH_STAGE_READY_STRICT_KICK_NOT_ARMED"
            return 2

        kick = harness_command(args)
        result["lowcmd_command"] = kick
        result["lowcmd_started"] = True
        result["motion_commands_sent"] = True
        print("LOWCMD_FR_KICK_START strict_fr_lane_ready={} allow_rough_kick={} hold_after_s={}".format(
            strict_ready, args.allow_rough_kick, args.kick_hold_after_s,
        ), flush=True)
        result["lowcmd_returncode"] = subprocess.run(kick, cwd=ROOT).returncode
        result["verdict"] = "LOWCMD_HARNESS_EXITED_NO_AUTO_HANDBACK"
        return 0 if result["lowcmd_returncode"] == 0 else 2
    except Exception as error:
        result["error"] = "{}: {}".format(type(error).__name__, error)
        result["verdict"] = "FAIL"
        return 2
    finally:
        result["elapsed_s"] = time.monotonic() - started
        write_json(output, result)
        print("ROUGH_STAGE_THEN_FR_KICK_{} output={}".format(result.get("verdict", "UNKNOWN"), output), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
