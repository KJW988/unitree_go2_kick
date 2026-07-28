#!/usr/bin/env python3
"""Go2 물리 리모컨을 direct DDS에서 감시해 localhost 상태/즉시 event를 남긴다.

이 firmware에서는 WebRTC ``rt/wirelesscontroller`` 구독자가 물리 remote input을
되돌려 주지 않았다. 반면 unitree_sdk2py direct DDS subscriber에서는 실제 stick input을
관측했다. 따라서 이 watcher는 **publisher를 만들지 않고** direct DDS만 구독한다.

WebRTC walker의 virtual joystick도 같은 DDS topic에 echo되므로, watcher는 walker가 만든
매우 짧은 local echo-window와 정확히 일치하는 packet만 self-echo로 제외한다. 그 외 stick/
button packet은 physical input으로 기록해 UDP event를 보낸다. writer identity는 SDK callback에
노출되지 않아, physical remote가 동일한 시간 창에 완전히 같은 joystick 값까지 보낼 경우에는
구별할 수 없다. 이 limitation은 status JSON에 기록하며 physical remote/E-stop은 여전히
operator의 1차 안전 수단이다.

출처: Unitree SDK2 Python Go2 communication examples,
https://github.com/unitreerobotics/unitree_sdk2_python
이 프로젝트에서는 ``ChannelSubscriber('rt/wirelesscontroller', WirelessController_)``의
read-only 사용만 채택했고 publisher/LowCmd/MotionSwitcher/Sport API는 추가하지 않았다.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_STATUS_FILE = Path("hardware_measurements/go2_direct_remote_watchdog.json")
DEFAULT_VIRTUAL_ECHO_WINDOW_FILE = Path("hardware_measurements/go2_stage_virtual_joystick_echo_window.json")
DEFAULT_UDP_HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 18181
DEFAULT_DEADZONE = 0.15
DEFAULT_HEARTBEAT_HZ = 20.0
VIRTUAL_ECHO_PROTOCOL_VERSION = 1

# Go2 URDF의 FR chain을 그대로 옮긴 read-only forward kinematics 상수다.
# 출처: resources/robots/go1/go2/urdf/go2.urdf
# SDK motor 순서는 공식 Go2 low-level example과 이 프로젝트의 검증된 mapping에서
# FR hip/thigh/calf = 0/1/2다. WebRTC SportModeState의 foot_position_body는 이
# firmware에서 전부 0이므로 automatic geometry에 사용하지 않는다.
FR_HIP_ORIGIN_BODY_M = (0.1934, -0.0465, 0.0)
FR_HIP_TO_THIGH_Y_M = -0.0955
FR_THIGH_LENGTH_M = 0.213
FR_CALF_LENGTH_M = 0.213
FR_MOTOR_INDICES = (0, 1, 2)


def fr_foot_kinematics_body(
    joint_q_rad: tuple[float, float, float],
    joint_dq_rad_s: tuple[float, float, float],
) -> tuple[list[float], list[float]]:
    """Go2 URDF FR foot 중심의 body-frame 위치/속도를 계산한다.

    q0는 x축 hip roll, q1/q2는 y축 thigh/calf pitch다. 아래 식은 URDF의
    base→hip→thigh→calf→foot 변환을 전개한 것이며 command를 만들지 않는다.
    """
    q0, q1, q2 = joint_q_rad
    dq0, dq1, dq2 = joint_dq_rad_s
    s0, c0 = math.sin(q0), math.cos(q0)
    s1, c1 = math.sin(q1), math.cos(q1)
    s12, c12 = math.sin(q1 + q2), math.cos(q1 + q2)

    inner_x = -FR_THIGH_LENGTH_M * s1 - FR_CALF_LENGTH_M * s12
    inner_y = FR_HIP_TO_THIGH_Y_M
    inner_z = -FR_THIGH_LENGTH_M * c1 - FR_CALF_LENGTH_M * c12
    position = [
        FR_HIP_ORIGIN_BODY_M[0] + inner_x,
        FR_HIP_ORIGIN_BODY_M[1] + c0 * inner_y - s0 * inner_z,
        FR_HIP_ORIGIN_BODY_M[2] + s0 * inner_y + c0 * inner_z,
    ]

    dx_dq1 = -FR_THIGH_LENGTH_M * c1 - FR_CALF_LENGTH_M * c12
    dx_dq2 = -FR_CALF_LENGTH_M * c12
    dz_dq1 = FR_THIGH_LENGTH_M * s1 + FR_CALF_LENGTH_M * s12
    dz_dq2 = FR_CALF_LENGTH_M * s12
    inner_z_speed = dz_dq1 * dq1 + dz_dq2 * dq2
    velocity = [
        dx_dq1 * dq1 + dx_dq2 * dq2,
        (-s0 * inner_y - c0 * inner_z) * dq0 - s0 * inner_z_speed,
        (c0 * inner_y - s0 * inner_z) * dq0 + c0 * inner_z_speed,
    ]
    return position, velocity


class RemoteWatchState:
    """DDS callback과 heartbeat loop 사이의 작은 thread-safe 상태다."""

    def __init__(
        self, deadzone: float, udp_host: str, udp_port: int, virtual_echo_window: Path,
    ) -> None:
        self._lock = threading.Lock()
        self.deadzone = deadzone
        self.udp_host, self.udp_port = udp_host, udp_port
        self.ready = False
        self.virtual_echo_window = virtual_echo_window
        self.event_count = 0
        self.last_active_monotonic_s: float | None = None
        self.last_event: dict[str, float | int] | None = None
        self._last_udp_monotonic_s = float("-inf")
        self.virtual_echo_ignored_count = 0
        self.last_virtual_echo: dict[str, float | int] | None = None
        self.lowstate_sample_count = 0
        self.last_lowstate_monotonic_s: float | None = None
        self.fr_joint_q_rad: list[float] | None = None
        self.fr_joint_dq_rad_s: list[float] | None = None
        self.fr_foot_position_body_m: list[float] | None = None
        self.fr_foot_speed_body_m_s: list[float] | None = None
        self.lowstate_error: str | None = "rt/lowstate sample을 아직 받지 못했습니다"
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _is_current_virtual_echo(self, sample: dict[str, float | int], now: float) -> bool:
        """stage가 직전에 선언한 exact virtual joystick echo만 무시한다.

        SDK ChannelSubscriber는 DDS sample writer GUID를 callback에 전달하지 않는다. 따라서
        local stage process가 만든 bounded packet의 value/time window를 명시적으로 대조한다.
        같은 값의 physical packet은 구분 불가하므로 narrow window와 exact equality tolerance만
        사용한다.
        """
        try:
            payload = json.loads(self.virtual_echo_window.read_text(encoding="utf-8"))
            expires = float(payload["expires_monotonic_s"])
            expected = payload["joystick"]
            if payload.get("kind") != "go2_stage_virtual_joystick_echo_window" or now > expires:
                return False
            if not isinstance(expected, dict) or int(expected.get("keys", -1)) != int(sample["keys"]):
                return False
            return all(
                abs(float(expected[key]) - float(sample[key])) <= 1e-5
                for key in ("lx", "ly", "rx", "ry")
            )
        except (OSError, ValueError, KeyError, TypeError):
            return False

    def on_wireless(self, message: Any) -> None:
        try:
            sample: dict[str, float | int] = {
                "lx": float(message.lx), "ly": float(message.ly),
                "rx": float(message.rx), "ry": float(message.ry), "keys": int(message.keys),
            }
        except (AttributeError, TypeError, ValueError):
            return
        active = max(abs(float(sample[key])) for key in ("lx", "ly", "rx", "ry")) > self.deadzone
        if not active and int(sample["keys"]) == 0:
            return
        now = time.monotonic()
        event = {"monotonic_s": now, **sample}
        if self._is_current_virtual_echo(sample, now):
            with self._lock:
                self.virtual_echo_ignored_count += 1
                self.last_virtual_echo = event
            return
        send_udp = False
        with self._lock:
            self.event_count += 1
            self.last_active_monotonic_s = now
            self.last_event = event
            # 연속 stick frame은 상태 파일에 모두 기록하되 UDP는 최대 20 Hz로 제한한다.
            if now - self._last_udp_monotonic_s >= 0.05:
                self._last_udp_monotonic_s = now
                send_udp = True
        if send_udp:
            try:
                self._udp.sendto(json.dumps(event, separators=(",", ":")).encode("utf-8"),
                                 (self.udp_host, self.udp_port))
            except OSError:
                # stage가 아직 UDP listener를 열기 전인 것은 정상이다. 상태 파일은 유지한다.
                pass

    def on_lowstate(self, message: Any) -> None:
        """FR motor 0/1/2를 읽어 URDF FK만 계산한다; publisher는 만들지 않는다."""
        try:
            motor_state = message.motor_state
            if len(motor_state) < 3:
                raise ValueError("motor_state가 3개보다 적습니다")
            q = tuple(float(motor_state[index].q) for index in FR_MOTOR_INDICES)
            dq = tuple(float(motor_state[index].dq) for index in FR_MOTOR_INDICES)
            if not all(math.isfinite(value) for value in (*q, *dq)):
                raise ValueError("FR q/dq에 non-finite 값이 있습니다")
            position, velocity = fr_foot_kinematics_body(q, dq)
        except (AttributeError, IndexError, TypeError, ValueError) as error:
            with self._lock:
                self.lowstate_error = str(error)
            return
        with self._lock:
            self.lowstate_sample_count += 1
            self.last_lowstate_monotonic_s = time.monotonic()
            self.fr_joint_q_rad = list(q)
            self.fr_joint_dq_rad_s = list(dq)
            self.fr_foot_position_body_m = position
            self.fr_foot_speed_body_m_s = velocity
            self.lowstate_error = None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = time.monotonic()
            lowstate_age_s = (
                None if self.last_lowstate_monotonic_s is None
                else max(0.0, now - self.last_lowstate_monotonic_s)
            )
            return {
                "schema_version": 1,
                "kind": "go2_direct_dds_physical_remote_watchdog",
                "updated_at_utc": datetime.now(timezone.utc).isoformat(),
                "heartbeat_monotonic_s": now,
                "ready": self.ready,
                "deadzone": self.deadzone,
                "physical_input_event_count": self.event_count,
                "last_active_monotonic_s": self.last_active_monotonic_s,
                "last_event": self.last_event,
                "virtual_echo_window": str(self.virtual_echo_window),
                "virtual_echo_protocol_version": VIRTUAL_ECHO_PROTOCOL_VERSION,
                "virtual_echo_ignored_count": self.virtual_echo_ignored_count,
                "last_virtual_echo": self.last_virtual_echo,
                "writer_identity_available": False,
                "same_value_physical_input_during_echo_window_unobservable": True,
                "udp_target": {"host": self.udp_host, "port": self.udp_port},
                "fr_foot_kinematics": {
                    "valid": self.lowstate_error is None,
                    "source": "direct DDS rt/lowstate motor_state[0:3] + Go2 URDF FK",
                    "motor_indices": list(FR_MOTOR_INDICES),
                    "sample_count": self.lowstate_sample_count,
                    "receipt_monotonic_s": self.last_lowstate_monotonic_s,
                    "age_s": lowstate_age_s,
                    "joint_q_rad": self.fr_joint_q_rad,
                    "joint_dq_rad_s": self.fr_joint_dq_rad_s,
                    "foot_position_body_m": self.fr_foot_position_body_m,
                    "foot_speed_body_m_s": self.fr_foot_speed_body_m_s,
                    "urdf_geometry_m": {
                        "hip_origin_body": list(FR_HIP_ORIGIN_BODY_M),
                        "hip_to_thigh_y": FR_HIP_TO_THIGH_Y_M,
                        "thigh_length": FR_THIGH_LENGTH_M,
                        "calf_length": FR_CALF_LENGTH_M,
                    },
                    "error": self.lowstate_error,
                },
                "motion_commands_sent": False,
            }


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name("." + path.name + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--status-file", type=Path, default=DEFAULT_STATUS_FILE)
    parser.add_argument("--virtual-echo-window", type=Path, default=DEFAULT_VIRTUAL_ECHO_WINDOW_FILE)
    parser.add_argument("--udp-host", default=DEFAULT_UDP_HOST)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE)
    parser.add_argument("--heartbeat-hz", type=float, default=DEFAULT_HEARTBEAT_HZ)
    args = parser.parse_args()
    if not 0.0 < args.deadzone < 1.0 or not 1 <= args.udp_port <= 65535 or args.heartbeat_hz <= 0.0:
        parser.error("deadzone/udp-port/heartbeat-hz 값이 유효하지 않습니다")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, WirelessController_
    except ImportError as error:
        raise RuntimeError(
            "이 watcher는 .conda-unitree-sdk-py311의 unitree_sdk2py가 필요합니다: {}".format(error)
        ) from error

    state = RemoteWatchState(args.deadzone, args.udp_host, args.udp_port, args.virtual_echo_window)
    ChannelFactoryInitialize(0, args.interface)
    subscriber = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
    subscriber.Init(state.on_wireless, 10)
    lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_subscriber.Init(state.on_lowstate, 10)
    state.ready = True
    atomic_write_json(args.status_file, state.snapshot())
    print(
        "DIRECT_DDS_REMOTE_WATCHDOG_READY interface={} status_file={} echo_window={} udp={}:{}; "
        "FR FK는 rt/lowstate motor[0:3]에서 read-only로 갱신됩니다. "
        "physical remote stick/button을 한 번 입력해 proof를 남기세요.".format(
            args.interface, args.status_file, args.virtual_echo_window, args.udp_host, args.udp_port
        ),
        flush=True,
    )
    try:
        while True:
            atomic_write_json(args.status_file, state.snapshot())
            time.sleep(1.0 / args.heartbeat_hz)
    except KeyboardInterrupt:
        state.ready = False
        atomic_write_json(args.status_file, state.snapshot())
        print("INTERRUPTED: direct DDS remote watchdog stopped", flush=True)
        return 130


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("FAILED: {}".format(error), flush=True)
        raise SystemExit(2)
