#!/usr/bin/env python3
"""Harness 전용: live standing baseline 위에서 frozen FR preset을 재생한다.

매 실행마다 rt/lowstate의 안정 standing pose를 median으로 측정하고, offline FR teacher의
SDK-order joint delta만 더한다. 즉 simulation default pose를 실물에 강요하지 않는다.
이 파일은 공/Tag/보행을 포함하지 않으며, harness·E-stop 환경의 무공 tuning 전용이다.

외부 출처: Unitree SDK2 Python Go2 low-level stand example
https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/low_level/go2_stand_example.py
Channel, LowCmd, CRC 사용 방식을 채택했다. ownership handoff는 명시적
`--release-motion-owner`에서만 MotionSwitcher/SportClient를 사용한다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from scripts.dry_run_go2_fr_kick_deploy import load_trajectory, validate_artifact  # noqa: E402

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
CONTROL_HZ = 200.0
BASELINE_CAPTURE_S = 4.0
BASELINE_MAX_SPAN_RAD = 0.006
CONFIRMATION = "HARNESS_ESTOP_READY"


class StateBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: Optional[Any] = None
        self._stamp = 0.0

    def callback(self, message: Any) -> None:
        with self._lock:
            self._state, self._stamp = message, time.monotonic()

    def latest(self) -> Tuple[Optional[Any], float]:
        with self._lock:
            return self._state, self._stamp


def _require_sdk() -> Tuple[Any, Any, Any, Any, Any, Any, Any]:
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
    except ImportError as error:
        raise RuntimeError("unitree_sdk2py import 실패: {}".format(error)) from error
    return ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber, unitree_go_msg_dds__LowCmd_, LowCmd_, LowState_, CRC


def _read_state(message: Any) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = np.asarray([message.motor_state[index].q for index in range(12)], dtype=np.float64)
    dq = np.asarray([message.motor_state[index].dq for index in range(12)], dtype=np.float64)
    rpy = np.asarray(message.imu_state.rpy, dtype=np.float64)
    if not np.isfinite(q).all() or not np.isfinite(dq).all() or rpy.shape != (3,) or not np.isfinite(rpy).all():
        raise RuntimeError("invalid rt/lowstate")
    return q, dq, rpy


def _capture_baseline(buffer: StateBuffer, duration_s: float) -> Tuple[np.ndarray, np.ndarray]:
    samples = []
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        message, stamp = buffer.latest()
        if message is not None and time.monotonic() - stamp < 0.25:
            q, _, _ = _read_state(message)
            samples.append(q)
        time.sleep(0.02)
    if len(samples) < 40:
        raise RuntimeError("fresh rt/lowstate가 충분하지 않습니다")
    values = np.asarray(samples)
    baseline, span = np.median(values, axis=0), values.max(axis=0) - values.min(axis=0)
    if float(np.max(span)) > BASELINE_MAX_SPAN_RAD:
        raise RuntimeError("standing baseline이 안정적이지 않습니다: max_span_rad={:.6f}".format(float(np.max(span))))
    return baseline, span


def _make_command(command_type: Any) -> Any:
    command = command_type()
    command.head[0], command.head[1] = 0xFE, 0xEF
    command.level_flag, command.gpio = 0xFF, 0
    for index in range(20):
        motor = command.motor_cmd[index]
        motor.mode, motor.q, motor.dq = 0x01, POS_STOP_F, VEL_STOP_F
        motor.kp, motor.kd, motor.tau = 0.0, 0.0, 0.0
    return command


class CommandStreamer:
    """Ownership handoff 중에도 baseline hold LowCmd를 끊지 않고 보낸다."""

    def __init__(self, publisher: Any, command_type: Any, crc_type: Any, target: np.ndarray, kp: float, kd: float) -> None:
        self._publisher = publisher
        self._command = _make_command(command_type)
        self._crc = crc_type()
        self._target = target.copy()
        self._kp, self._kd = kp, kd
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="go2-lowcmd-stream", daemon=True)

    def set_target(self, target: np.ndarray) -> None:
        with self._lock:
            self._target = target.copy()

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        period, next_tick = 1.0 / CONTROL_HZ, time.monotonic()
        while not self._stop.is_set():
            with self._lock:
                target = self._target.copy()
            for index in range(12):
                motor = self._command.motor_cmd[index]
                motor.q, motor.dq = float(target[index]), 0.0
                motor.kp, motor.kd, motor.tau = self._kp, self._kd, 0.0
            self._command.crc = self._crc.Crc(self._command)
            self._publisher.Write(self._command)
            next_tick += period
            self._stop.wait(max(0.0, next_tick - time.monotonic()))


def _interpolate(elapsed_s: float, time_s: np.ndarray, positions: np.ndarray) -> np.ndarray:
    index = int(np.searchsorted(time_s, elapsed_s, side="right"))
    if index <= 0:
        return positions[0]
    if index >= len(time_s):
        return positions[-1]
    alpha = (elapsed_s - time_s[index - 1]) / (time_s[index] - time_s[index - 1])
    return (1.0 - alpha) * positions[index - 1] + alpha * positions[index]


def _minimum_jerk(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def _write_log(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")


def _release_motion_owner() -> None:
    """이미 실행 중인 baseline stream을 유지한 채 공식 ownership을 넘긴다."""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.go2.sport.sport_client import SportClient

    sport = SportClient()
    sport.SetTimeout(2.0)
    sport.Init()
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(2.0)
    switcher.Init()
    status, result = switcher.CheckMode()
    owner = result.get("name", "")
    print("MOTION_OWNER_BEFORE_RELEASE={!r} status={}".format(owner, status), flush=True)
    if not owner:
        return
    # Unitree example also uses StandDown before ReleaseMode. Harness에서만 허용한다.
    sport.StandDown()
    switcher.ReleaseMode()
    for _ in range(10):
        status, result = switcher.CheckMode()
        if not result.get("name", ""):
            print("LOW_LEVEL_OWNERSHIP_READY", flush=True)
            return
        time.sleep(0.02)
    raise RuntimeError("motion owner release failed: {}".format(result.get("name", "")))


def _handback_motion_owner(mode: str) -> None:
    """LowCmd 종료 뒤 Unitree MotionSwitcher의 controller mode를 다시 선택한다."""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

    switcher = MotionSwitcherClient()
    switcher.SetTimeout(2.0)
    switcher.Init()
    status, _ = switcher.SelectMode(mode)
    if status != 0:
        raise RuntimeError("controller handback SelectMode({!r}) failed status={}".format(mode, status))
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        status, result = switcher.CheckMode()
        if status == 0 and result.get("name", "") == mode:
            print("CONTROLLER_HANDBACK_READY mode={}".format(mode), flush=True)
            return
        time.sleep(0.05)
    raise RuntimeError("controller handback did not acquire mode {!r}".format(mode))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--kp", type=float, required=True, help="all 12 joints low-level position gain")
    parser.add_argument("--kd", type=float, required=True, help="all 12 joints low-level damping gain")
    parser.add_argument("--execute", action="store_true", help="없으면 capture/target preview만 수행")
    parser.add_argument("--operator-confirm", help="execute에는 {} 필요".format(CONFIRMATION))
    parser.add_argument("--release-motion-owner", action="store_true", help="capture 뒤 공식 Sport/MCF ownership을 해제하고 즉시 hold")
    parser.add_argument("--prehold-s", type=float, default=1.0, help="release 뒤 baseline hold 시간")
    parser.add_argument("--hold-only", action="store_true", help="FR preset 대신 captured standing baseline만 유지")
    parser.add_argument("--hold-only-s", type=float, default=3.0, help="--hold-only 유지 시간")
    parser.add_argument("--hold-after-s", type=float, default=0.0, help="preset/hold 뒤 baseline을 추가 유지할 시간; 0이면 즉시 종료")
    parser.add_argument("--preset-time-scale", type=float, default=1.0, help="1보다 작으면 preset을 더 빠르게 재생")
    parser.add_argument("--handoff-blend-s", type=float, default=0.4, help="release 직후 actual q에서 baseline으로 연결하는 minimum-jerk 시간")
    parser.add_argument("--handback-mode", default="", help="검증 후에만 지정하는 controller MotionSwitcher mode")
    parser.add_argument("--log-dir", type=Path, default=Path("hardware_measurements"))
    args = parser.parse_args()
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("--execute requires --operator-confirm {}".format(CONFIRMATION))
    if not all(math.isfinite(value) and value > 0.0 for value in (args.kp, args.kd, args.prehold_s, args.hold_only_s)):
        parser.error("--kp, --kd, --prehold-s, and --hold-only-s must be positive finite")
    if not math.isfinite(args.hold_after_s) or args.hold_after_s < 0.0:
        parser.error("--hold-after-s must be finite and non-negative")
    if not math.isfinite(args.preset_time_scale) or not 0.2 <= args.preset_time_scale <= 2.0:
        parser.error("--preset-time-scale must be between 0.2 and 2.0")
    if not math.isfinite(args.handoff_blend_s) or args.handoff_blend_s <= 0.0:
        parser.error("--handoff-blend-s must be positive and finite")
    if args.release_motion_owner and not args.execute:
        parser.error("--release-motion-owner requires --execute")

    teacher = load_trajectory(args.trajectory)
    errors = validate_artifact(teacher)
    if errors:
        parser.error("invalid FR preset artifact: {}".format("; ".join(errors)))
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber, command_default, command_type, state_type, crc_type = _require_sdk()
    ChannelFactoryInitialize(0, args.interface)
    buffer = StateBuffer()
    subscriber = ChannelSubscriber(TOPIC_LOWSTATE, state_type)
    subscriber.Init(buffer.callback, 10)
    print("CAPTURING_LIVE_STANDING_BASELINE duration_s={}".format(BASELINE_CAPTURE_S), flush=True)
    baseline, span = _capture_baseline(buffer, BASELINE_CAPTURE_S)
    # q_sdk is raw device order, so adding the delta preserves the frozen FR preset exactly.
    preset_delta = teacher["q_sdk_motor_order_rad"] - teacher["q_sdk_motor_order_rad"][0]
    target = baseline[None, :] + preset_delta
    summary = {
        "schema_version": 1,
        "kind": "go2_harness_live_baseline_fr_preset",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "interface": args.interface,
        "baseline_sdk_q_rad": baseline.tolist(),
        "baseline_span_rad": span.tolist(),
        "trajectory": str(args.trajectory),
        "preset_duration_s": float(teacher["time_s"][-1]),
        "kp": args.kp,
        "kd": args.kd,
        "release_motion_owner": args.release_motion_owner,
        "prehold_s": args.prehold_s,
        "hold_only": args.hold_only,
        "hold_only_s": args.hold_only_s,
        "hold_after_s": args.hold_after_s,
        "preset_time_scale": args.preset_time_scale,
        "handoff_blend_s": args.handoff_blend_s,
        "handback_mode": args.handback_mode,
        "execute": args.execute,
        "records": [],
    }
    print("BASELINE_OK max_span_rad={:.6f} preset_max_delta_rad={:.6f}".format(float(np.max(span)), float(np.max(np.abs(preset_delta)))), flush=True)
    if not args.execute:
        path = args.log_dir / "live_baseline_fr_preset_preview.json"
        _write_log(path, summary)
        print("PREVIEW_ONLY_OK no LowCmd publisher was created log={}".format(path))
        return 0

    print("LOWCMD starts in 3 seconds; harness/E-stop/no-ball/low-level ownership must already be ready.", flush=True)
    for seconds in (3, 2, 1):
        print("{}...".format(seconds), flush=True)
        time.sleep(1.0)
    publisher = ChannelPublisher(TOPIC_LOWCMD, command_type)
    publisher.Init()
    streamer = CommandStreamer(publisher, command_default, crc_type, baseline, args.kp, args.kd)
    # Release 전부터 stream을 시작한다. Sport가 owner일 때는 무시되지만 release 순간에는
    # 이미 같은 standing target이 200 Hz로 도착 중이므로 torque 공백을 최소화한다.
    streamer.start()
    release_q = baseline.copy()
    if args.release_motion_owner:
        _release_motion_owner()
        message, stamp = buffer.latest()
        if message is None or time.monotonic() - stamp >= 0.25:
            streamer.stop()
            raise RuntimeError("release 직후 fresh rt/lowstate를 받지 못했습니다")
        release_q, _, _ = _read_state(message)
    start = time.monotonic()
    motion_s = args.hold_only_s if args.hold_only else args.preset_time_scale * float(teacher["time_s"][-1])
    end_s, next_tick = args.handoff_blend_s + args.prehold_s + motion_s, time.monotonic()
    print("LOWCMD_ACTIVE frozen_FR_preset={} hold_only={} handoff_blend_s={} prehold_s={}".format(not args.hold_only, args.hold_only, args.handoff_blend_s, args.prehold_s), flush=True)
    try:
        while True:
            now, elapsed = time.monotonic(), time.monotonic() - start
            if elapsed < args.handoff_blend_s:
                desired = release_q + _minimum_jerk(elapsed / args.handoff_blend_s) * (baseline - release_q)
            else:
                teacher_elapsed = max(0.0, elapsed - args.handoff_blend_s - args.prehold_s) / args.preset_time_scale
                desired = baseline if args.hold_only else _interpolate(teacher_elapsed, teacher["time_s"], target)
            streamer.set_target(desired)
            state, stamp = buffer.latest()
            if state is not None and now - stamp < 0.25:
                q, dq, rpy = _read_state(state)
                summary["records"].append({"t_s": elapsed, "target_sdk_q_rad": desired.tolist(), "q_sdk_q_rad": q.tolist(), "dq_sdk_rad_s": dq.tolist(), "rpy_rad": rpy.tolist()})
            if elapsed >= end_s:
                break
            next_tick += 1.0 / CONTROL_HZ
            time.sleep(max(0.0, next_tick - time.monotonic()))
        if args.hold_after_s > 0.0:
            print("POST_HOLD_ACTIVE duration_s={}".format(args.hold_after_s), flush=True)
            deadline = time.monotonic() + args.hold_after_s
            while time.monotonic() < deadline:
                now = time.monotonic()
                streamer.set_target(baseline)
                state, stamp = buffer.latest()
                if state is not None and now - stamp < 0.25:
                    q, dq, rpy = _read_state(state)
                    summary["records"].append({"t_s": now - start, "target_sdk_q_rad": baseline.tolist(), "q_sdk_q_rad": q.tolist(), "dq_sdk_rad_s": dq.tolist(), "rpy_rad": rpy.tolist()})
                time.sleep(1.0 / CONTROL_HZ)
    finally:
        streamer.stop()
    path = args.log_dir / ("live_baseline_fr_preset_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
    _write_log(path, summary)
    print("PRESET_COMPLETE log={}".format(path), flush=True)
    if args.handback_mode:
        _handback_motion_owner(args.handback_mode)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("INTERRUPTED: execute 중이었다면 즉시 E-stop 상태를 확인하세요.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
