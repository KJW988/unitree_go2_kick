#!/usr/bin/env python3
"""D435i ball/AprilTag state와 내장 LiDAR odom으로 Go2를 안전 staging 위치까지 보행시킨다.

이 프로그램은 frozen FR LowCmd kick을 호출하지 않는다. Unitree MCF SportClient의
고수준 ``Move(vx, vy, vyaw)``만 사용해 공과 Tag가 동시에 보이는 구간에서 정렬하고,
설정한 camera depth standoff에서 반드시 멈춘다. 그래서 공이 camera 아래로 사라지는
최종 FR docking과 LowCmd kick은 이 프로그램의 범위 밖이다.

Unitree SDK2 Go2 SportClient의 공식 ``Move`` API를 채택했다.
출처: https://github.com/unitreerobotics/unitree_sdk2_python
변경점: remote input, stale perception, runtime expiry, target loss가 모두 StopMove로
fail-closed 되며 원격 조종 입력 후에는 이 process가 자동 재개하지 않는다.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any, Optional


CONFIRMATION = "SPORT_APPROACH_READY"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def remote_is_active(message: Any, deadzone: float) -> bool:
    return (
        max(abs(float(message.lx)), abs(float(message.ly)), abs(float(message.rx)), abs(float(message.ry)))
        > deadzone
        or int(message.keys) != 0
    )


class OdomStore:
    """DDS callback의 최신 base pose만 보존한다."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stamp = float("-inf")
        self._position: Optional[tuple[float, float, float]] = None
        self._yaw: Optional[float] = None

    def update(self, message: Any) -> None:
        position = message.pose.pose.position
        q = message.pose.pose.orientation
        siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy = 1.0 - 2.0 * (float(q.y) ** 2 + float(q.z) ** 2)
        with self._lock:
            self._stamp = time.monotonic()
            self._position = (float(position.x), float(position.y), float(position.z))
            self._yaw = math.atan2(siny, cosy)

    def snapshot(self) -> tuple[float, Optional[tuple[float, float, float]], Optional[float]]:
        with self._lock:
            return self._stamp, self._position, self._yaw


class RemoteLatch:
    """physical remote input은 즉시 정지하고 process 종료까지 resume을 금지한다."""

    def __init__(self, deadzone: float) -> None:
        self.deadzone = deadzone
        self._lock = threading.Lock()
        self._tripped = False

    def update(self, message: Any) -> None:
        if remote_is_active(message, self.deadzone):
            with self._lock:
                self._tripped = True

    def tripped(self) -> bool:
        with self._lock:
            return self._tripped


@dataclass(frozen=True)
class Perception:
    ball_range_m: float
    ball_ground_x_m: float
    heading_error_rad: float
    age_s: float


def fetch_perception(url: str, tag_id: int, timeout_s: float) -> Optional[Perception]:
    try:
        with urllib.request.urlopen(url, timeout=timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (OSError, ValueError, urllib.error.URLError):
        return None
    if not payload.get("ready"):
        return None
    ball = payload.get("ball") or {}
    ground = ball.get("ground_camera_xyz_m")
    target = payload.get("target_line") or {}
    if (
        ground is None
        or len(ground) != 3
        or int(target.get("tag_id", -1)) != tag_id
        or len(target.get("unit_camera_xyz", [])) != 3
    ):
        return None
    stamp = float(payload.get("stamp_monotonic_s", float("-inf")))
    target_unit = target["unit_camera_xyz"]
    # RealSense optical frame: +x is image-right, +z is forward.
    heading = math.atan2(float(target_unit[0]), float(target_unit[2]))
    return Perception(
        ball_range_m=float(ball.get("depth_range_m", float("nan"))),
        ball_ground_x_m=float(ground[0]),
        heading_error_rad=heading,
        age_s=time.monotonic() - stamp,
    )


def plan_command(state: Perception, args: argparse.Namespace) -> tuple[float, float, float, str]:
    """camera-frame target ray로 보수적 SportClient velocity를 만든다.

    D435i→base extrinsic의 yaw/FR lateral offset이 아직 고정 보정되지 않았으므로,
    기본값은 yaw+forward staging만 한다. lateral Move는 명시적 opt-in이다.
    """
    if not math.isfinite(state.ball_range_m) or not math.isfinite(state.ball_ground_x_m):
        return 0.0, 0.0, 0.0, "invalid_ball_range"
    if state.ball_range_m <= args.stop_ball_range_m:
        return 0.0, 0.0, 0.0, "staging_ready"
    yaw = -clamp(args.yaw_gain * state.heading_error_rad, args.max_yaw_rps * -1.0, args.max_yaw_rps)
    lateral_error = state.ball_ground_x_m - args.ball_camera_lateral_target_m
    lateral = 0.0
    if args.enable_lateral:
        # Unitree Move convention is +vy left. Optical +x is image-right.
        lateral = -clamp(args.lateral_gain * lateral_error, args.max_lateral_mps * -1.0, args.max_lateral_mps)
    aligned = abs(state.heading_error_rad) <= args.heading_tolerance_rad
    lateral_aligned = (not args.enable_lateral) or abs(lateral_error) <= args.lateral_tolerance_m
    forward = args.max_forward_mps if aligned and lateral_aligned else 0.0
    phase = "advance" if forward > 0.0 else "align"
    return forward, lateral, yaw, phase


def require_sdk() -> tuple[Any, Any, Any, Any, Any]:
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
    except ImportError as error:
        raise RuntimeError("unitree_sdk2py SDK environment가 필요합니다: {}".format(error)) from error
    return ChannelFactoryInitialize, ChannelSubscriber, SportClient, Odometry_, WirelessController_


def result_code(result: Any) -> int:
    """SDK Python의 int 또는 ``(int, payload)`` 반환을 같은 방식으로 판정한다."""
    return int(result[0] if isinstance(result, tuple) else result)


def planar_distance(first: Optional[tuple[float, float, float]],
                    second: Optional[tuple[float, float, float]]) -> Optional[float]:
    if first is None or second is None:
        return None
    return math.hypot(second[0] - first[0], second[1] - first[1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--perception-url", default="http://127.0.0.1:8080/state.json")
    parser.add_argument("--tag-id", type=int, required=True)
    parser.add_argument("--execute", action="store_true", help="없으면 command plan만 출력한다")
    parser.add_argument("--operator-confirm", help="--execute에는 {} 필요".format(CONFIRMATION))
    parser.add_argument("--max-runtime-s", type=float, default=8.0)
    parser.add_argument("--tick-hz", type=float, default=10.0)
    parser.add_argument("--perception-max-age-s", type=float, default=0.35)
    parser.add_argument("--odom-max-age-s", type=float, default=0.50)
    parser.add_argument("--stop-ball-range-m", type=float, default=0.55)
    parser.add_argument("--max-travel-m", type=float, default=0.35,
                        help="start odom에서 이 평면 거리만큼 이동하면 무조건 정지")
    parser.add_argument("--motion-confirm-s", type=float, default=1.5,
                        help="advance command 뒤 실제 odom progress를 확인할 시간")
    parser.add_argument("--min-motion-progress-m", type=float, default=0.01)
    parser.add_argument("--range-progress-s", type=float, default=3.0,
                        help="forward 중 ball range가 줄지 않으면 정지하는 시간")
    parser.add_argument("--min-range-progress-m", type=float, default=0.03)
    parser.add_argument("--max-forward-mps", type=float, default=0.08)
    parser.add_argument("--max-lateral-mps", type=float, default=0.05)
    parser.add_argument("--max-yaw-rps", type=float, default=0.20)
    parser.add_argument("--yaw-gain", type=float, default=1.0)
    parser.add_argument("--lateral-gain", type=float, default=0.5)
    parser.add_argument("--heading-tolerance-rad", type=float, default=0.08)
    parser.add_argument("--lateral-tolerance-m", type=float, default=0.04)
    parser.add_argument("--ball-camera-lateral-target-m", type=float, default=0.0)
    parser.add_argument("--enable-lateral", action="store_true")
    parser.add_argument("--remote-deadzone", type=float, default=0.15)
    args = parser.parse_args()
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("--execute에는 --operator-confirm {} 필요".format(CONFIRMATION))
    if not 0.0 < args.tick_hz <= 30.0 or args.max_runtime_s <= 0.0:
        parser.error("--tick-hz/max-runtime-s 범위를 확인하세요")
    if not 0.30 <= args.stop_ball_range_m <= 2.0:
        parser.error("--stop-ball-range-m은 D435i valid range 안에서 설정하세요")
    if min(args.max_travel_m, args.motion_confirm_s, args.min_motion_progress_m,
           args.range_progress_s, args.min_range_progress_m) <= 0.0:
        parser.error("motion/range watchdog 값은 양수여야 합니다")

    ChannelFactoryInitialize, ChannelSubscriber, SportClient, Odometry_, WirelessController_ = require_sdk()
    odom, remote = OdomStore(), RemoteLatch(args.remote_deadzone)
    ChannelFactoryInitialize(0, args.interface)
    odom_subscriber = ChannelSubscriber("rt/utlidar/robot_odom", Odometry_)
    remote_subscriber = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
    odom_subscriber.Init(odom.update, 10)
    remote_subscriber.Init(remote.update, 10)
    sport = SportClient()
    sport.SetTimeout(1.0)
    sport.Init()

    print(
        "APPROACH_READY execute={} tag={} perception={} max_runtime_s={}".format(
            args.execute, args.tag_id, args.perception_url, args.max_runtime_s
        ),
        flush=True,
    )
    start, period, last_reason = time.monotonic(), 1.0 / args.tick_hz, ""
    start_position: Optional[tuple[float, float, float]] = None
    advance_started_at: Optional[float] = None
    advance_start_position: Optional[tuple[float, float, float]] = None
    advance_start_range: Optional[float] = None
    best_range: Optional[float] = None
    last_telemetry_at = float("-inf")
    try:
        while time.monotonic() - start < args.max_runtime_s:
            perception = fetch_perception(args.perception_url, args.tag_id, timeout_s=min(0.2, period))
            odom_stamp, position, _ = odom.snapshot()
            now = time.monotonic()
            if start_position is None and position is not None:
                start_position = position
            reason = ""
            command = (0.0, 0.0, 0.0)
            if remote.tripped():
                reason = "remote_preempted"
            elif perception is None:
                reason = "perception_missing"
            elif perception.age_s > args.perception_max_age_s:
                reason = "perception_stale"
            elif time.monotonic() - odom_stamp > args.odom_max_age_s:
                reason = "odom_stale"
            else:
                vx, vy, wz, reason = plan_command(perception, args)
                command = (vx, vy, wz)
                travelled = planar_distance(start_position, position)
                if travelled is not None and travelled >= args.max_travel_m:
                    reason, command = "travel_limit", (0.0, 0.0, 0.0)
                elif reason == "advance":
                    if advance_started_at is None:
                        advance_started_at = now
                        advance_start_position = position
                        advance_start_range = perception.ball_range_m
                        best_range = perception.ball_range_m
                    else:
                        best_range = min(best_range if best_range is not None else perception.ball_range_m,
                                         perception.ball_range_m)
                        elapsed = now - advance_started_at
                        moved = planar_distance(advance_start_position, position)
                        if elapsed >= args.motion_confirm_s and (moved is None or moved < args.min_motion_progress_m):
                            reason, command = "motion_not_confirmed", (0.0, 0.0, 0.0)
                        elif elapsed >= args.range_progress_s and (
                            advance_start_range is None or best_range is None
                            or advance_start_range - best_range < args.min_range_progress_m
                        ):
                            reason, command = "ball_range_no_progress", (0.0, 0.0, 0.0)
                else:
                    advance_started_at = None
                    advance_start_position = None
                    advance_start_range = None
                    best_range = None
            if reason != last_reason:
                print(
                    "STATE reason={} command=[{:.3f},{:.3f},{:.3f}]".format(reason, *command),
                    flush=True,
                )
                last_reason = reason
            if perception is not None and now - last_telemetry_at >= 1.0:
                travelled = planar_distance(start_position, position)
                print(
                    "TELEMETRY reason={} range_m={:.3f} heading_rad={:.3f} travel_m={} command=[{:.3f},{:.3f},{:.3f}]".format(
                        reason, perception.ball_range_m, perception.heading_error_rad,
                        "unknown" if travelled is None else "{:.3f}".format(travelled), *command
                    ),
                    flush=True,
                )
                last_telemetry_at = now
            if args.execute:
                if reason in ("advance", "align"):
                    code = result_code(sport.Move(*command))
                    if code != 0:
                        sport.StopMove()
                        print("APPROACH_STOP reason=move_rejected code={}".format(code), flush=True)
                        return 2
                else:
                    sport.StopMove()
                    if reason in (
                        "staging_ready", "remote_preempted", "perception_missing", "perception_stale", "odom_stale",
                        "travel_limit", "motion_not_confirmed", "ball_range_no_progress",
                    ):
                        print("APPROACH_STOP reason={}".format(reason), flush=True)
                        return 0 if reason == "staging_ready" else 2
            else:
                # preview에서는 어떤 SportClient movement API도 호출하지 않는다.
                if reason == "staging_ready":
                    return 0
            time.sleep(period)
    finally:
        if args.execute:
            sport.StopMove()
    print("APPROACH_STOP reason=runtime_expired", flush=True)
    return 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
