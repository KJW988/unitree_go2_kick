#!/usr/bin/env python3
"""Go2 물리 리모컨을 direct DDS에서 감시해 localhost 상태/즉시 event를 남긴다.

이 firmware에서는 WebRTC ``rt/wirelesscontroller`` 구독자가 물리 remote input을
되돌려 주지 않았다. 반면 unitree_sdk2py direct DDS subscriber에서는 실제 stick input을
관측했다. 따라서 이 watcher는 **publisher를 만들지 않고** direct DDS만 구독한다.

후속 WebRTC walker는 이 파일의 fresh heartbeat와 prior physical-input proof를 요구하고,
UDP event 또는 상태 파일의 fresh input을 받으면 다음 virtual joystick packet 전에
neutralize한다. 이것은 firmware 내부 arbitration API가 아니라 user-space fail-closed
guard이므로, physical remote/E-stop은 여전히 operator의 1차 안전 수단이다.

출처: Unitree SDK2 Python Go2 communication examples,
https://github.com/unitreerobotics/unitree_sdk2_python
이 프로젝트에서는 ``ChannelSubscriber('rt/wirelesscontroller', WirelessController_)``의
read-only 사용만 채택했고 publisher/LowCmd/MotionSwitcher/Sport API는 추가하지 않았다.
"""
from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_STATUS_FILE = Path("hardware_measurements/go2_direct_remote_watchdog.json")
DEFAULT_UDP_HOST = "127.0.0.1"
DEFAULT_UDP_PORT = 18181
DEFAULT_DEADZONE = 0.15
DEFAULT_HEARTBEAT_HZ = 20.0


class RemoteWatchState:
    """DDS callback과 heartbeat loop 사이의 작은 thread-safe 상태다."""

    def __init__(self, deadzone: float, udp_host: str, udp_port: int) -> None:
        self._lock = threading.Lock()
        self.deadzone = deadzone
        self.udp_host, self.udp_port = udp_host, udp_port
        self.ready = False
        self.event_count = 0
        self.last_active_monotonic_s: float | None = None
        self.last_event: dict[str, float | int] | None = None
        self._last_udp_monotonic_s = float("-inf")
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "schema_version": 1,
                "kind": "go2_direct_dds_physical_remote_watchdog",
                "updated_at_utc": datetime.now(timezone.utc).isoformat(),
                "heartbeat_monotonic_s": time.monotonic(),
                "ready": self.ready,
                "deadzone": self.deadzone,
                "physical_input_event_count": self.event_count,
                "last_active_monotonic_s": self.last_active_monotonic_s,
                "last_event": self.last_event,
                "udp_target": {"host": self.udp_host, "port": self.udp_port},
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
    parser.add_argument("--udp-host", default=DEFAULT_UDP_HOST)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    parser.add_argument("--deadzone", type=float, default=DEFAULT_DEADZONE)
    parser.add_argument("--heartbeat-hz", type=float, default=DEFAULT_HEARTBEAT_HZ)
    args = parser.parse_args()
    if not 0.0 < args.deadzone < 1.0 or not 1 <= args.udp_port <= 65535 or args.heartbeat_hz <= 0.0:
        parser.error("deadzone/udp-port/heartbeat-hz 값이 유효하지 않습니다")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
    except ImportError as error:
        raise RuntimeError(
            "이 watcher는 .conda-unitree-sdk-py311의 unitree_sdk2py가 필요합니다: {}".format(error)
        ) from error

    state = RemoteWatchState(args.deadzone, args.udp_host, args.udp_port)
    ChannelFactoryInitialize(0, args.interface)
    subscriber = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
    subscriber.Init(state.on_wireless, 10)
    state.ready = True
    atomic_write_json(args.status_file, state.snapshot())
    print(
        "DIRECT_DDS_REMOTE_WATCHDOG_READY interface={} status_file={} udp={}:{}; "
        "physical remote stick/button을 한 번 입력해 proof를 남기세요.".format(
            args.interface, args.status_file, args.udp_host, args.udp_port
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
