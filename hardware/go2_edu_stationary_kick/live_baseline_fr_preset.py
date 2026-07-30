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
BASELINE_MAX_SPAN_RAD = 0.006
BASELINE_STABLE_WINDOW_S = 1.0
BASELINE_STABLE_TIMEOUT_S = 12.0
CONFIRMATION = "HARNESS_ESTOP_READY"
# export_vendor_go2_fr_kick_teacher.py의 LOAD_END. 이 뒤부터만 swing amplitude를 조정한다.
FR_SWING_START_S = 2.80
LOWSTATE_MAX_AGE_S = 0.15
DIRECT_REMOTE_MAX_AGE_S = 0.35
DIRECT_REMOTE_PROTOCOL_VERSION = 2
# resources/robots/go1/go2/urdf/go2.urdf의 SDK-order 반복 joint limits다.
SDK_POSITION_LIMITS_RAD = np.asarray(
    [(-1.0472, 1.0472), (-1.5708, 3.4907), (-2.7227, -0.83776)] * 4,
    dtype=np.float64,
)
SDK_VELOCITY_LIMITS_RAD_S = np.asarray([30.1, 30.1, 20.07] * 4, dtype=np.float64)


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


def _capture_stable_baseline(
    buffer: StateBuffer, *, stable_window_s: float, timeout_s: float,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """보행 잔동작을 버리고 최근 연속 stable window만 baseline으로 채택한다.

    과거 구현은 시작 직후부터 고정 4초 전체를 묶어 span을 계산했기 때문에, gait가 이미
    멈췄어도 첫 transient 한 번이 있으면 LowCmd를 거부했다. 이제 bounded timeout 안에서
    최근 window만 검사하되 기존 0.006rad 관절 span gate는 완화하지 않는다.
    """
    samples: list[tuple[float, np.ndarray]] = []
    started = time.monotonic()
    deadline = started + timeout_s
    best_span = float("inf")
    last_sample_stamp = float("-inf")
    while time.monotonic() < deadline:
        message, stamp = buffer.latest()
        now = time.monotonic()
        if message is not None and now - stamp < 0.25 and stamp > last_sample_stamp:
            q, _, _ = _read_state(message)
            last_sample_stamp = stamp
            samples.append((stamp, q))
            cutoff = now - stable_window_s
            samples = [sample for sample in samples if sample[0] >= cutoff]
            if len(samples) >= max(20, int(stable_window_s / 0.04)):
                covered_s = samples[-1][0] - samples[0][0]
                if covered_s >= stable_window_s * 0.90:
                    values = np.asarray([sample[1] for sample in samples])
                    baseline = np.median(values, axis=0)
                    span = values.max(axis=0) - values.min(axis=0)
                    best_span = min(best_span, float(np.max(span)))
                    if float(np.max(span)) <= BASELINE_MAX_SPAN_RAD:
                        return baseline, span, now - started
        time.sleep(0.02)
    if not samples:
        raise RuntimeError("fresh rt/lowstate가 충분하지 않습니다")
    raise RuntimeError(
        "standing baseline 안정 window timeout: timeout_s={:.1f} best_max_span_rad={:.6f}".format(
            timeout_s, best_span,
        )
    )


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
    """Ownership handoff 중에도 full-gain standing target LowCmd를 끊지 않고 보낸다."""

    def __init__(self, publisher: Any, command_type: Any, crc_type: Any, target: np.ndarray, kp: float, kd: float) -> None:
        self._publisher = publisher
        self._command = _make_command(command_type)
        self._crc = crc_type()
        self._target = target.copy()
        # Release 순간에도 바로 지지 토크가 있어야 한다. 0-gain ramp는 StandDown 뒤
        # robot을 주저앉게 할 수 있으므로 사용하지 않는다.
        self._kp, self._kd = kp, kd
        self._max_kp, self._max_kd = kp, kd
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._error: Optional[BaseException] = None
        self._thread = threading.Thread(target=self._run, name="go2-lowcmd-stream", daemon=True)

    def set_command(self, target: np.ndarray, kp_scale: float) -> None:
        """target과 bounded PD gain을 같은 tick snapshot으로 바꾼다."""
        kp_scale = min(1.0, max(0.0, kp_scale))
        with self._lock:
            self._target = target.copy()
            self._kp, self._kd = self._max_kp * kp_scale, self._max_kd * kp_scale

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def check(self) -> None:
        if self._error is not None:
            raise RuntimeError("LowCmd publisher thread 실패: {}".format(self._error))

    def _run(self) -> None:
        try:
            period, next_tick = 1.0 / CONTROL_HZ, time.monotonic()
            while not self._stop.is_set():
                with self._lock:
                    target = self._target.copy()
                    kp, kd = self._kp, self._kd
                for index in range(12):
                    motor = self._command.motor_cmd[index]
                    motor.q, motor.dq = float(target[index]), 0.0
                    motor.kp, motor.kd, motor.tau = kp, kd, 0.0
                self._command.crc = self._crc.Crc(self._command)
                self._publisher.Write(self._command)
                next_tick += period
                self._stop.wait(max(0.0, next_tick - time.monotonic()))
        except BaseException as error:
            self._error = error
            self._stop.set()


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


def _validate_scaled_target(target: np.ndarray, time_s: np.ndarray, time_scale: float) -> dict[str, float]:
    """FR swing scale 적용 후의 실제 SDK target을 URDF position/velocity limit로 재검증한다."""
    if target.ndim != 2 or target.shape[1] != 12 or len(time_s) != len(target):
        raise RuntimeError("scaled target shape가 유효하지 않습니다")
    bad_position = np.argwhere(
        (target < SDK_POSITION_LIMITS_RAD[:, 0])
        | (target > SDK_POSITION_LIMITS_RAD[:, 1])
    )
    if bad_position.size:
        sample, joint = (int(value) for value in bad_position[0])
        raise RuntimeError(
            "scaled target joint limit 초과: sample={} sdk_joint={} q={:.5f} limits={}".format(
                sample, joint, target[sample, joint], SDK_POSITION_LIMITS_RAD[joint].tolist(),
            )
        )
    if len(target) > 1:
        dt = np.diff(time_s) * time_scale
        speed = np.abs(np.diff(target, axis=0) / dt[:, None])
        bad_speed = np.argwhere(speed > SDK_VELOCITY_LIMITS_RAD_S[None, :])
        if bad_speed.size:
            sample, joint = (int(value) for value in bad_speed[0])
            raise RuntimeError(
                "scaled target URDF velocity limit 초과: sample={} sdk_joint={} speed={:.5f} limit={:.5f}".format(
                    sample, joint, speed[sample, joint], SDK_VELOCITY_LIMITS_RAD_S[joint],
                )
            )
        max_speed = float(np.max(speed))
    else:
        max_speed = 0.0
    return {
        "min_target_rad": float(np.min(target)),
        "max_target_rad": float(np.max(target)),
        "max_discrete_target_speed_rad_s": max_speed,
    }


def _read_remote_watchdog(path: Path, baseline_event_count: int | None = None) -> int:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        heartbeat = float(payload["heartbeat_monotonic_s"])
        event_count = int(payload["physical_input_event_count"])
        protocol = int(payload["virtual_echo_protocol_version"])
        last_active = float(payload["last_active_monotonic_s"])
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise RuntimeError("direct remote watchdog를 읽을 수 없습니다: {}".format(error)) from error
    if payload.get("ready") is not True or payload.get("motion_commands_sent") is not False:
        raise RuntimeError("direct remote watchdog가 read-only ready가 아닙니다")
    if protocol != DIRECT_REMOTE_PROTOCOL_VERSION:
        raise RuntimeError("direct remote watchdog protocol mismatch; watcher를 재시작하세요")
    if time.monotonic() - heartbeat > DIRECT_REMOTE_MAX_AGE_S:
        raise RuntimeError("direct remote watchdog heartbeat가 stale입니다")
    if event_count < 1 or time.monotonic() - last_active < 0.60:
        raise RuntimeError("physical remote input proof가 없거나 아직 active입니다")
    if baseline_event_count is not None and event_count != baseline_event_count:
        raise RuntimeError("physical remote input이 감지되었습니다")
    return event_count


def _runtime_state_gate(
    buffer: StateBuffer, desired: np.ndarray, *, max_tracking_error_rad: float,
    max_joint_speed_rad_s: float, max_tilt_rad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    state, stamp = buffer.latest()
    age_s = time.monotonic() - stamp
    if state is None or age_s > LOWSTATE_MAX_AGE_S:
        raise RuntimeError("rt/lowstate stale during LowCmd: age_s={:.3f}".format(age_s))
    q, dq, rpy = _read_state(state)
    tracking_error = float(np.max(np.abs(q - desired)))
    joint_speed = float(np.max(np.abs(dq)))
    tilt = float(np.max(np.abs(rpy[:2])))
    if tracking_error > max_tracking_error_rad:
        raise RuntimeError("LowCmd tracking error 초과: {:.4f} rad".format(tracking_error))
    if joint_speed > max_joint_speed_rad_s:
        raise RuntimeError("joint speed 초과: {:.4f} rad/s".format(joint_speed))
    if tilt > max_tilt_rad:
        raise RuntimeError("body tilt 초과: {:.4f} rad".format(tilt))
    return q, dq, rpy, tracking_error


def _release_motion_owner(*, without_stand_down: bool) -> None:
    """이미 실행 중인 baseline stream을 유지한 채 공식 ownership을 넘긴다."""
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(2.0)
    switcher.Init()
    status, result = switcher.CheckMode()
    owner = result.get("name", "")
    print("MOTION_OWNER_BEFORE_RELEASE={!r} status={}".format(owner, status), flush=True)
    if not owner:
        return
    if without_stand_down:
        # 출처: Unitree SDK2 C++ Go2 stand example
        # https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp
        # 해당 공식 예제는 StandDown 없이 ReleaseMode만 호출한다. 이 opt-in 경로는
        # full-gain baseline LowCmd stream이 이미 실행 중인 harness hold-only 검증용이다.
        print("MOTION_RELEASE_PATH=direct_release_only", flush=True)
    else:
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        sport = SportClient()
        sport.SetTimeout(2.0)
        sport.Init()
        sport.StandDown()
        print("MOTION_RELEASE_PATH=stand_down_then_release", flush=True)
    # unitree_sdk2py MotionSwitcherClient.ReleaseMode() returns ``(code, None)``,
    # unlike the C++ SDK's scalar int32 return. 성공 code=0만 판정한다.
    release_result = switcher.ReleaseMode()
    release_status = release_result[0] if isinstance(release_result, tuple) else release_result
    if release_status != 0:
        raise RuntimeError("MotionSwitcher.ReleaseMode failed status={}".format(release_status))
    for _ in range(100):
        status, result = switcher.CheckMode()
        if not result.get("name", ""):
            print("LOW_LEVEL_OWNERSHIP_READY", flush=True)
            return
        time.sleep(0.02)
    raise RuntimeError("motion owner release failed: {}".format(result.get("name", "")))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--kp", type=float, required=True, help="all 12 joints low-level position gain")
    parser.add_argument("--kd", type=float, required=True, help="all 12 joints low-level damping gain")
    parser.add_argument("--execute", action="store_true", help="없으면 capture/target preview만 수행")
    parser.add_argument("--operator-confirm", help="execute에는 {} 필요".format(CONFIRMATION))
    parser.add_argument("--release-motion-owner", action="store_true", help="capture 뒤 공식 Sport/MCF ownership을 해제하고 즉시 hold")
    parser.add_argument("--release-without-stand-down", action="store_true", help="official C++ direct ReleaseMode 경로; harness hold-only 검증에서만 사용")
    parser.add_argument("--prehold-s", type=float, default=1.0, help="release 뒤 baseline hold 시간")
    parser.add_argument("--hold-only", action="store_true", help="FR preset 대신 captured standing baseline만 유지")
    parser.add_argument("--hold-only-s", type=float, default=3.0, help="--hold-only 유지 시간")
    parser.add_argument("--hold-after-s", type=float, default=0.0, help="preset/hold 뒤 baseline을 추가 유지할 시간; 0이면 즉시 종료")
    parser.add_argument("--preset-time-scale", type=float, default=1.0, help="1보다 작으면 preset을 더 빠르게 재생")
    parser.add_argument("--fr-swing-scale", type=float, default=1.0, help="kick phase FR thigh/calf delta scale; 1.15는 15%% extension")
    parser.add_argument("--handoff-blend-s", type=float, default=0.4, help="release 직후 actual q에서 baseline으로 연결하는 minimum-jerk 시간")
    parser.add_argument(
        "--baseline-stable-window-s", type=float, default=BASELINE_STABLE_WINDOW_S,
        help="LowCmd 전환 전에 연속으로 관절 span gate를 통과해야 하는 최근 window",
    )
    parser.add_argument(
        "--baseline-stable-timeout-s", type=float, default=BASELINE_STABLE_TIMEOUT_S,
        help="보행 종료 뒤 stable baseline을 기다리는 fail-closed 최대 시간",
    )
    parser.add_argument(
        "--start-countdown-s", type=float, default=3.0,
        help="stable baseline 뒤 LowCmd publisher/ownership release 전 추가 countdown",
    )
    parser.add_argument(
        "--direct-remote-status", type=Path, default=None,
        help="LowCmd 중 물리 리모컨 preempt를 감시할 direct-DDS watcher JSON",
    )
    parser.add_argument("--runtime-max-tracking-error-rad", type=float, default=0.65)
    parser.add_argument("--runtime-max-joint-speed-rad-s", type=float, default=25.0)
    parser.add_argument("--runtime-max-tilt-rad", type=float, default=0.35)
    parser.add_argument("--abort-hold-s", type=float, default=5.0)
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
    if not math.isfinite(args.fr_swing_scale) or not 0.8 <= args.fr_swing_scale <= 1.3:
        parser.error("--fr-swing-scale must be between 0.8 and 1.3")
    if not 0.5 <= args.baseline_stable_window_s <= 3.0:
        parser.error("--baseline-stable-window-s must be between 0.5 and 3.0")
    if not 2.0 <= args.baseline_stable_timeout_s <= 30.0:
        parser.error("--baseline-stable-timeout-s must be between 2.0 and 30.0")
    if not math.isfinite(args.start_countdown_s) or not 0.0 <= args.start_countdown_s <= 5.0:
        parser.error("--start-countdown-s must be between 0.0 and 5.0")
    if args.release_motion_owner and not args.execute:
        parser.error("--release-motion-owner requires --execute")
    if args.release_without_stand_down and not args.release_motion_owner:
        parser.error("--release-without-stand-down requires --release-motion-owner")
    if args.execute and args.direct_remote_status is None:
        parser.error("--execute에는 fresh --direct-remote-status가 필요합니다")
    if not (
        0.10 <= args.runtime_max_tracking_error_rad <= 0.80
        and 1.0 <= args.runtime_max_joint_speed_rad_s <= 30.0
        and 0.10 <= args.runtime_max_tilt_rad <= 0.50
        and 1.0 <= args.abort_hold_s <= 15.0
    ):
        parser.error("runtime tracking/speed/tilt/abort-hold 범위가 유효하지 않습니다")

    teacher = load_trajectory(args.trajectory)
    errors = validate_artifact(teacher)
    if errors:
        parser.error("invalid FR preset artifact: {}".format("; ".join(errors)))
    ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber, command_default, command_type, state_type, crc_type = _require_sdk()
    ChannelFactoryInitialize(0, args.interface)
    buffer = StateBuffer()
    subscriber = ChannelSubscriber(TOPIC_LOWSTATE, state_type)
    subscriber.Init(buffer.callback, 10)
    print(
        "WAITING_FOR_STABLE_BASELINE window_s={} timeout_s={}".format(
            args.baseline_stable_window_s, args.baseline_stable_timeout_s,
        ),
        flush=True,
    )
    baseline, span, baseline_wait_s = _capture_stable_baseline(
        buffer,
        stable_window_s=args.baseline_stable_window_s,
        timeout_s=args.baseline_stable_timeout_s,
    )
    # q_sdk is raw device order, so adding the delta preserves the frozen FR preset exactly.
    preset_delta = teacher["q_sdk_motor_order_rad"] - teacher["q_sdk_motor_order_rad"][0]
    # SDK raw order에서 FR은 0/1/2이고, sagittal reach는 thigh/calf(1/2)가 담당한다.
    # support preload와 baseline은 보존하고 swing phase만 bounded physical tuning한다.
    swing_mask = teacher["time_s"] >= FR_SWING_START_S
    preset_delta[swing_mask, 1:3] *= args.fr_swing_scale
    target = baseline[None, :] + preset_delta
    scaled_target_validation = _validate_scaled_target(
        target, teacher["time_s"], args.preset_time_scale,
    )
    summary = {
        "schema_version": 1,
        "kind": "go2_harness_live_baseline_fr_preset",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "interface": args.interface,
        "baseline_sdk_q_rad": baseline.tolist(),
        "baseline_span_rad": span.tolist(),
        "baseline_wait_s": baseline_wait_s,
        "baseline_stable_window_s": args.baseline_stable_window_s,
        "baseline_stable_timeout_s": args.baseline_stable_timeout_s,
        "trajectory": str(args.trajectory),
        "preset_duration_s": float(teacher["time_s"][-1]),
        "kp": args.kp,
        "kd": args.kd,
        "release_motion_owner": args.release_motion_owner,
        "release_without_stand_down": args.release_without_stand_down,
        "prehold_s": args.prehold_s,
        "hold_only": args.hold_only,
        "hold_only_s": args.hold_only_s,
        "hold_after_s": args.hold_after_s,
        "preset_time_scale": args.preset_time_scale,
        "fr_swing_scale": args.fr_swing_scale,
        "handoff_blend_s": args.handoff_blend_s,
        "scaled_target_validation": scaled_target_validation,
        "direct_remote_status": str(args.direct_remote_status),
        "runtime_limits": {
            "max_tracking_error_rad": args.runtime_max_tracking_error_rad,
            "max_joint_speed_rad_s": args.runtime_max_joint_speed_rad_s,
            "max_tilt_rad": args.runtime_max_tilt_rad,
            "lowstate_max_age_s": LOWSTATE_MAX_AGE_S,
            "abort_hold_s": args.abort_hold_s,
        },
        "execute": args.execute,
        "records": [],
    }
    print(
        "BASELINE_OK wait_s={:.3f} max_span_rad={:.6f} preset_max_delta_rad={:.6f}".format(
            baseline_wait_s, float(np.max(span)), float(np.max(np.abs(preset_delta))),
        ),
        flush=True,
    )
    if not args.execute:
        path = args.log_dir / "live_baseline_fr_preset_preview.json"
        _write_log(path, summary)
        print("PREVIEW_ONLY_OK no LowCmd publisher was created log={}".format(path))
        return 0

    assert args.direct_remote_status is not None
    remote_event_count = _read_remote_watchdog(args.direct_remote_status)
    if args.start_countdown_s > 0.0:
        print(
            "LOWCMD starts in {:.1f} seconds; harness/E-stop/low-level ownership must already be ready.".format(
                args.start_countdown_s,
            ),
            flush=True,
        )
        time.sleep(args.start_countdown_s)
    _read_remote_watchdog(args.direct_remote_status, remote_event_count)
    path = args.log_dir / (
        "live_baseline_fr_preset_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    publisher = ChannelPublisher(TOPIC_LOWCMD, command_type)
    publisher.Init()
    streamer = CommandStreamer(publisher, command_default, crc_type, baseline, args.kp, args.kd)
    # Release 전부터 stream을 시작한다. Sport가 owner일 때는 무시되지만 release 순간에는
    # 이미 같은 standing target이 200 Hz로 도착 중이므로 torque 공백을 최소화한다.
    streamer.start()
    release_q = baseline.copy()
    try:
        if args.release_motion_owner:
            _release_motion_owner(without_stand_down=args.release_without_stand_down)
            message, stamp = buffer.latest()
            if message is None or time.monotonic() - stamp >= LOWSTATE_MAX_AGE_S:
                raise RuntimeError("release 직후 fresh rt/lowstate를 받지 못했습니다")
            release_q, _, _ = _read_state(message)
            summary["release_sdk_q_rad"] = release_q.tolist()
    except Exception as error:
        summary["verdict"] = "OWNERSHIP_RELEASE_ABORT"
        summary["runtime_abort"] = "{}: {}".format(type(error).__name__, error)
        streamer.stop()
        _write_log(path, summary)
        raise
    start = time.monotonic()
    motion_s = args.hold_only_s if args.hold_only else args.preset_time_scale * float(teacher["time_s"][-1])
    handoff_s = args.handoff_blend_s
    end_s, next_tick = handoff_s + args.prehold_s + motion_s, time.monotonic()
    next_remote_check = time.monotonic()
    print("LOWCMD_ACTIVE frozen_FR_preset={} hold_only={} handoff_blend_s={} prehold_s={}".format(not args.hold_only, args.hold_only, args.handoff_blend_s, args.prehold_s), flush=True)
    try:
        while True:
            now, elapsed = time.monotonic(), time.monotonic() - start
            if elapsed < handoff_s:
                # Full gain을 release 전부터 유지한 상태에서 actual q만 baseline으로 연결한다.
                desired = release_q + _minimum_jerk(elapsed / args.handoff_blend_s) * (baseline - release_q)
                gain_scale = 1.0
            else:
                teacher_elapsed = max(0.0, elapsed - handoff_s - args.prehold_s) / args.preset_time_scale
                desired = baseline if args.hold_only else _interpolate(teacher_elapsed, teacher["time_s"], target)
                gain_scale = 1.0
            streamer.set_command(desired, gain_scale)
            streamer.check()
            if now >= next_remote_check:
                _read_remote_watchdog(args.direct_remote_status, remote_event_count)
                next_remote_check = now + 0.05
            q, dq, rpy, tracking_error = _runtime_state_gate(
                buffer, desired,
                max_tracking_error_rad=args.runtime_max_tracking_error_rad,
                max_joint_speed_rad_s=args.runtime_max_joint_speed_rad_s,
                max_tilt_rad=args.runtime_max_tilt_rad,
            )
            summary["records"].append({
                "t_s": elapsed, "gain_scale": gain_scale,
                "target_sdk_q_rad": desired.tolist(), "q_sdk_q_rad": q.tolist(),
                "dq_sdk_rad_s": dq.tolist(), "rpy_rad": rpy.tolist(),
                "max_tracking_error_rad": tracking_error,
            })
            if elapsed >= end_s:
                break
            next_tick += 1.0 / CONTROL_HZ
            time.sleep(max(0.0, next_tick - time.monotonic()))
        if args.hold_after_s > 0.0:
            print("POST_HOLD_ACTIVE duration_s={}".format(args.hold_after_s), flush=True)
            deadline = time.monotonic() + args.hold_after_s
            while time.monotonic() < deadline:
                now = time.monotonic()
                streamer.set_command(baseline, 1.0)
                streamer.check()
                if now >= next_remote_check:
                    _read_remote_watchdog(args.direct_remote_status, remote_event_count)
                    next_remote_check = now + 0.05
                q, dq, rpy, tracking_error = _runtime_state_gate(
                    buffer, baseline,
                    max_tracking_error_rad=args.runtime_max_tracking_error_rad,
                    max_joint_speed_rad_s=args.runtime_max_joint_speed_rad_s,
                    max_tilt_rad=args.runtime_max_tilt_rad,
                )
                summary["records"].append({
                    "t_s": now - start, "gain_scale": 1.0,
                    "target_sdk_q_rad": baseline.tolist(), "q_sdk_q_rad": q.tolist(),
                    "dq_sdk_rad_s": dq.tolist(), "rpy_rad": rpy.tolist(),
                    "max_tracking_error_rad": tracking_error,
                })
                time.sleep(1.0 / CONTROL_HZ)
        summary["verdict"] = "PRESET_COMPLETE"
    except Exception as error:
        summary["verdict"] = "RUNTIME_ABORT"
        summary["runtime_abort"] = "{}: {}".format(type(error).__name__, error)
        print(
            "LOWCMD_RUNTIME_ABORT reason={!r}; actual-pose hold {:.1f}s, E-stop을 확인하세요".format(
                summary["runtime_abort"], args.abort_hold_s,
            ),
            file=sys.stderr, flush=True,
        )
        state, stamp = buffer.latest()
        if state is not None and time.monotonic() - stamp <= LOWSTATE_MAX_AGE_S:
            abort_q, _, _ = _read_state(state)
            streamer.set_command(abort_q, 1.0)
        time.sleep(args.abort_hold_s)
        raise
    finally:
        streamer.stop()
        _write_log(path, summary)
    print("PRESET_COMPLETE log={}".format(path), flush=True)
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
