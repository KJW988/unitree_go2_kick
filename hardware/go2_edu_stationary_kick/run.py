#!/usr/bin/env python3
"""Go2 EDU의 고정 자세 FR kick artifact를 매우 제한적으로 재생한다.

기본 모드는 `--preflight`다. 이 모드는 ``rt/lowstate``만 구독하며 DDS
publisher, MotionSwitcher, SportClient를 만들지 않는다. ``--execute``는
attestation, live pose, operator confirmation을 모두 통과한 뒤에만 LowCmd를
발행한다.

외부 출처: Unitree SDK2 Python Go2 low-level stand example
https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/low_level/go2_stand_example.py
여기서는 ChannelPublisher/Subscriber, LowCmd 초기화, CRC API만 채택했다.
공식 예제의 MotionSwitcher 해제와 임의 stand trajectory는 의도적으로 사용하지 않는다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional, Sequence, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.dry_run_go2_fr_kick_deploy import (  # noqa: E402
    EXPECTED_CANONICAL_TO_SDK,
    load_trajectory,
    validate_artifact,
    validate_attestation,
)

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
POS_STOP_F = 2.146e9
VEL_STOP_F = 16000.0
CONTROL_HZ = 250.0
POSE_TOLERANCE_RAD = 0.05
VELOCITY_TOLERANCE_RAD_S = 0.25
TILT_TOLERANCE_RAD = 0.12
STABLE_DWELL_S = 0.40
OPERATOR_CONFIRMATION = "I_UNDERSTAND_LOWCMD"


class LowStateBuffer:
    """SDK callback에서 마지막 LowState만 보관한다."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: Optional[Any] = None
        self._stamp_s = 0.0

    def callback(self, message: Any) -> None:
        with self._lock:
            self._state = message
            self._stamp_s = time.monotonic()

    def latest(self) -> Tuple[Optional[Any], float]:
        with self._lock:
            return self._state, self._stamp_s


def _require_sdk() -> Tuple[Any, Any, Any, Any, Any, Any, Any]:
    """SDK import는 실제 DDS preflight/execute에서만 수행한다."""

    try:
        from unitree_sdk2py.core.channel import (
            ChannelFactoryInitialize,
            ChannelPublisher,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC
    except ImportError as error:
        raise RuntimeError(
            "unitree_sdk2py를 찾지 못했습니다. 실제 Go2 PC에서 공식 SDK를 설치한 뒤 "
            "다시 실행하세요: {}".format(error)
        ) from error
    return (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
        unitree_go_msg_dds__LowCmd_,
        LowCmd_,
        LowState_,
        CRC,
    )


def _motor_values(state: Any) -> Tuple[np.ndarray, np.ndarray]:
    motor_state = state.motor_state
    if len(motor_state) < 12:
        raise RuntimeError("rt/lowstate motor_state가 12개보다 적습니다")
    q = np.asarray([motor_state[index].q for index in range(12)], dtype=np.float64)
    dq = np.asarray([motor_state[index].dq for index in range(12)], dtype=np.float64)
    if not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise RuntimeError("rt/lowstate에 non-finite motor 값이 있습니다")
    return q, dq


def _rpy(state: Any) -> np.ndarray:
    value = np.asarray(state.imu_state.rpy, dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise RuntimeError("rt/lowstate IMU rpy가 유효하지 않습니다")
    return value


def _load_attestation(path: Path) -> dict[str, Any]:
    try:
        with path.open(encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("hardware attestation을 읽을 수 없습니다: {}".format(error)) from error


def _device_targets(canonical: np.ndarray, attestation: dict[str, Any]) -> np.ndarray:
    """attested canonical encoder scale/offset 뒤 SDK device 순서로 변환한다."""

    scale = np.asarray(attestation["position_scale"], dtype=np.float64)
    offset = np.asarray(attestation["position_offset_rad"], dtype=np.float64)
    physical_canonical = canonical * scale[None, :] + offset[None, :]
    device = np.empty_like(physical_canonical)
    device[:, EXPECTED_CANONICAL_TO_SDK] = physical_canonical
    return device


def _validate_attested_command_speed(data: dict[str, Any], attestation: dict[str, Any]) -> None:
    """firmware 최고속도가 아닌, 무공 검증을 마친 command envelope만 허용한다."""

    try:
        scale = np.asarray(attestation["position_scale"], dtype=np.float64)
        limit = np.asarray(attestation["validated_command_speed_limit_rad_s"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError("validated_command_speed_limit_rad_s가 필요합니다") from error
    if scale.shape != (12,) or limit.shape != (12,) or not np.isfinite(scale).all() or not np.isfinite(limit).all():
        raise RuntimeError("validated command speed limit은 canonical 12개 finite 값이어야 합니다")
    physical_speed = np.abs(data["planned_qd_canonical_rad_s"] * scale[None, :])
    bad = np.argwhere(physical_speed > limit[None, :])
    if bad.size:
        sample, joint = (int(bad[0, 0]), int(bad[0, 1]))
        raise RuntimeError(
            "trajectory exceeds validated command speed at sample {}, canonical joint {}: {:.4f} > {:.4f} rad/s".format(
                sample, joint, float(physical_speed[sample, joint]), float(limit[joint])
            )
        )
    print("COMMAND_SPEED_GATE_PASS max_physical_rad_s={:.4f}".format(float(np.max(physical_speed))), flush=True)


def _preflight_reason(state: Any, desired_device_q: np.ndarray) -> Tuple[bool, str]:
    q, dq = _motor_values(state)
    rpy = _rpy(state)
    max_error = float(np.max(np.abs(q - desired_device_q)))
    max_velocity = float(np.max(np.abs(dq)))
    max_tilt = float(np.max(np.abs(rpy[:2])))
    ready = (
        max_error <= POSE_TOLERANCE_RAD
        and max_velocity <= VELOCITY_TOLERANCE_RAD_S
        and max_tilt <= TILT_TOLERANCE_RAD
    )
    return ready, (
        "max_pose_error_rad={:.5f} max_joint_velocity_rad_s={:.5f} "
        "roll_rad={:.5f} pitch_rad={:.5f}".format(max_error, max_velocity, rpy[0], rpy[1])
    )


def _wait_for_lowstate(buffer: LowStateBuffer, timeout_s: float) -> Any:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state, stamp_s = buffer.latest()
        if state is not None and time.monotonic() - stamp_s < 0.25:
            return state
        time.sleep(0.02)
    raise RuntimeError("{} s 안에 fresh rt/lowstate를 받지 못했습니다".format(timeout_s))


def _wait_for_stable_pose(buffer: LowStateBuffer, desired_device_q: np.ndarray, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    stable_since: Optional[float] = None
    latest_detail = "lowstate waiting"
    while time.monotonic() < deadline:
        state, stamp_s = buffer.latest()
        now = time.monotonic()
        if state is None or now - stamp_s >= 0.25:
            stable_since = None
            latest_detail = "rt/lowstate stale"
        else:
            ready, latest_detail = _preflight_reason(state, desired_device_q)
            if ready:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= STABLE_DWELL_S:
                    print("PREFLIGHT_PASS stable_dwell_s={:.2f} {}".format(STABLE_DWELL_S, latest_detail))
                    return
            else:
                stable_since = None
        time.sleep(0.02)
    raise RuntimeError("preflight pose/quiet gate 실패: {}".format(latest_detail))


def _make_lowcmd(command_type: Any) -> Any:
    command = command_type()
    command.head[0] = 0xFE
    command.head[1] = 0xEF
    command.level_flag = 0xFF
    command.gpio = 0
    for index in range(20):
        motor = command.motor_cmd[index]
        motor.mode = 0x01
        motor.q = POS_STOP_F
        motor.dq = VEL_STOP_F
        motor.kp = 0.0
        motor.kd = 0.0
        motor.tau = 0.0
    return command


def _attested_gains(attestation: dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    try:
        kp = np.asarray(attestation["lowcmd_kp"], dtype=np.float64)
        kd = np.asarray(attestation["lowcmd_kd"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise RuntimeError(
            "execute에는 hardware attestation의 lowcmd_kp와 lowcmd_kd(각 canonical 12개)가 필요합니다"
        ) from error
    if kp.shape != (12,) or kd.shape != (12,) or not np.isfinite(kp).all() or not np.isfinite(kd).all():
        raise RuntimeError("lowcmd_kp/lowcmd_kd는 finite canonical 12개여야 합니다")
    if np.any(kp <= 0.0) or np.any(kd <= 0.0):
        raise RuntimeError("lowcmd_kp/lowcmd_kd는 모두 양수여야 합니다")
    device_kp = np.empty(12, dtype=np.float64)
    device_kd = np.empty(12, dtype=np.float64)
    device_kp[list(EXPECTED_CANONICAL_TO_SDK)] = kp
    device_kd[list(EXPECTED_CANONICAL_TO_SDK)] = kd
    return device_kp, device_kd


def _interpolate(time_s: float, samples_time_s: np.ndarray, samples_q: np.ndarray) -> np.ndarray:
    index = int(np.searchsorted(samples_time_s, time_s, side="right"))
    if index <= 0:
        return samples_q[0]
    if index >= len(samples_time_s):
        return samples_q[-1]
    t0, t1 = samples_time_s[index - 1], samples_time_s[index]
    alpha = (time_s - t0) / (t1 - t0)
    return (1.0 - alpha) * samples_q[index - 1] + alpha * samples_q[index]


def _execute(
    publisher: Any,
    crc: Any,
    command_type: Any,
    samples_time_s: np.ndarray,
    samples_q: np.ndarray,
    device_kp: np.ndarray,
    device_kd: np.ndarray,
) -> None:
    """250 Hz linear interpolation; target은 50 Hz로 export한 frozen teacher뿐이다."""

    command = _make_lowcmd(command_type)
    start_s = time.monotonic()
    period_s = 1.0 / CONTROL_HZ
    next_tick_s = start_s
    end_s = float(samples_time_s[-1])
    print("LOWCMD_ACTIVE duration_s={:.3f} control_hz={:.1f}".format(end_s, CONTROL_HZ), flush=True)
    while True:
        now_s = time.monotonic()
        elapsed_s = now_s - start_s
        target = _interpolate(min(elapsed_s, end_s), samples_time_s, samples_q)
        for index in range(12):
            motor = command.motor_cmd[index]
            motor.q = float(target[index])
            motor.dq = 0.0
            motor.kp = float(device_kp[index])
            motor.kd = float(device_kd[index])
            motor.tau = 0.0
        command.crc = crc.Crc(command)
        publisher.Write(command)
        if elapsed_s >= end_s:
            break
        next_tick_s += period_s
        time.sleep(max(0.0, next_tick_s - time.monotonic()))
    print("LOWCMD_COMPLETE final_target_is_initial_teacher_pose=true", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="Go2 DDS network interface, e.g. enp2s0")
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--hardware-attestation", type=Path, required=True)
    parser.add_argument("--preflight-timeout-s", type=float, default=15.0)
    parser.add_argument("--execute", action="store_true", help="명시적 LowCmd mode; 기본은 구독-only")
    parser.add_argument("--operator-confirm", help="실행 시 정확히 {} 필요".format(OPERATOR_CONFIRMATION))
    args = parser.parse_args()
    if args.preflight_timeout_s <= 0.0 or not math.isfinite(args.preflight_timeout_s):
        parser.error("--preflight-timeout-s must be positive and finite")
    if args.execute and args.operator_confirm != OPERATOR_CONFIRMATION:
        parser.error("--execute requires --operator-confirm {}".format(OPERATOR_CONFIRMATION))

    data = load_trajectory(args.trajectory)
    errors = validate_artifact(data)
    errors.extend(validate_attestation(args.hardware_attestation, data["q_canonical_rad"]))
    if errors:
        for error in errors:
            print("NOT_ARMABLE: {}".format(error), file=sys.stderr)
        return 2
    attestation = _load_attestation(args.hardware_attestation)
    device_q = _device_targets(data["q_canonical_rad"], attestation)

    (
        ChannelFactoryInitialize,
        ChannelPublisher,
        ChannelSubscriber,
        lowcmd_default_type,
        lowcmd_type,
        lowstate_type,
        crc_type,
    ) = _require_sdk()
    # Unitree official SDK example과 같은 DDS factory init. 여기서 mode 전환은 하지 않는다.
    ChannelFactoryInitialize(0, args.interface)
    buffer = LowStateBuffer()
    subscriber = ChannelSubscriber(TOPIC_LOWSTATE, lowstate_type)
    subscriber.Init(buffer.callback, 10)
    _wait_for_lowstate(buffer, args.preflight_timeout_s)
    print("LOWSTATE_CONNECTED interface={} publisher_created=false".format(args.interface))
    _wait_for_stable_pose(buffer, device_q[0], args.preflight_timeout_s)
    if not args.execute:
        print("PREFLIGHT_ONLY_PASS no LowCmd publisher was created")
        return 0

    _validate_attested_command_speed(data, attestation)
    device_kp, device_kd = _attested_gains(attestation)
    print(
        "EXECUTE_ARMED: no MotionSwitcher/SportClient call was made. "
        "Operator must already have exclusive low-level ownership, E-stop and hoist ready.",
        flush=True,
    )
    for remaining in (3, 2, 1):
        print("LOWCMD starts in {} (Ctrl-C now aborts before publisher creation)".format(remaining), flush=True)
        time.sleep(1.0)
    # Publisher는 이 지점 이후에만 생성한다. 실행 중 Ctrl-C/예외는 현장에서 E-stop으로 처리한다.
    publisher = ChannelPublisher(TOPIC_LOWCMD, lowcmd_type)
    publisher.Init()
    _execute(publisher, crc_type(), lowcmd_default_type, data["time_s"], device_q, device_kp, device_kd)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("ABORTED: LowCmd publisher creation 전이면 명령은 전송되지 않았습니다. 실행 중이었다면 E-stop 상태를 확인하세요.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("NOT_ARMABLE_OR_ABORTED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
