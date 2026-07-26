#!/usr/bin/env python3
"""D435i 정지 IMU 통계를 read-only로 기록한다.

Go2 DDS, LowCmd, SportClient, MotionSwitcher를 사용하지 않는다. D435i의
accelerometer와 gyroscope stream만 열어 정지 시 IMU 통계를 기록한다. RGB+IMU
동시 stream은 USB 2.x 연결에서 timeout 날 수 있어 의도적으로 열지 않는다.

출처: Intel librealsense SDK 2.0 motion stream/extrinsics API
https://github.com/realsenseai/librealsense/tree/v2.56.5/wrappers/python
여기서는 camera→Go2 base transform을 추정하지 않는다. D435i IMU는 정지 중
roll/pitch 확인에는 쓰지만, 자력계가 없으므로 base 기준 yaw나 x/y/z를 제공하지 않는다.
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
from typing import Any


def _require_runtime() -> tuple[Any, Any]:
    try:
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "D435i IMU probe에는 project perception env의 numpy와 pyrealsense2가 필요합니다: {}".format(error)
        ) from error
    return np, rs


def _vector_stats(np: Any, samples: list[list[float]]) -> dict[str, Any]:
    array = np.asarray(samples, dtype=np.float64)
    norms = np.linalg.norm(array, axis=1)
    return {
        "sample_count": int(len(array)),
        "mean": [float(value) for value in np.mean(array, axis=0)],
        "std": [float(value) for value in np.std(array, axis=0)],
        "norm_mean": float(np.mean(norms)),
        "norm_std": float(np.std(norms)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--accel-fps", type=int, default=100)
    parser.add_argument("--gyro-fps", type=int, default=200)
    parser.add_argument("--output-dir", type=Path, default=Path("hardware_measurements"))
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        parser.error("--duration-s must be positive and finite")
    if not math.isfinite(args.warmup_s) or args.warmup_s < 0.0:
        parser.error("--warmup-s must be finite and non-negative")
    if min(args.accel_fps, args.gyro_fps) <= 0:
        parser.error("--accel-fps and --gyro-fps must be positive")

    np, rs = _require_runtime()
    devices = list(rs.context().query_devices())
    if not devices:
        raise RuntimeError("D435i device를 찾지 못했습니다")
    device = devices[0]
    motion_sensor, accel_profile, gyro_profile = None, None, None
    for sensor in device.query_sensors():
        profiles = list(sensor.get_stream_profiles())
        accel_candidates = [
            profile for profile in profiles
            if profile.stream_type() == rs.stream.accel
            and profile.format() == rs.format.motion_xyz32f
            and profile.fps() == args.accel_fps
        ]
        gyro_candidates = [
            profile for profile in profiles
            if profile.stream_type() == rs.stream.gyro
            and profile.format() == rs.format.motion_xyz32f
            and profile.fps() == args.gyro_fps
        ]
        if accel_candidates and gyro_candidates:
            motion_sensor = sensor
            accel_profile, gyro_profile = accel_candidates[0], gyro_candidates[0]
            break
    if motion_sensor is None or accel_profile is None or gyro_profile is None:
        raise RuntimeError(
            "요청한 D435i motion profile을 찾지 못했습니다 (accel={} Hz, gyro={} Hz)".format(
                args.accel_fps, args.gyro_fps
            )
        )

    accel_samples: list[list[float]] = []
    gyro_samples: list[list[float]] = []
    last_motion_stamp_ms: dict[Any, float] = {}
    duplicate_motion_frame_count = [0]
    sample_lock = threading.Lock()

    def on_motion_frame(frame: Any) -> None:
        if not frame.is_motion_frame():
            return
        stream_type = frame.get_profile().stream_type()
        stamp_ms = float(frame.get_timestamp())
        motion = frame.as_motion_frame().get_motion_data()
        vector = [float(motion.x), float(motion.y), float(motion.z)]
        with sample_lock:
            if last_motion_stamp_ms.get(stream_type) == stamp_ms:
                duplicate_motion_frame_count[0] += 1
                return
            last_motion_stamp_ms[stream_type] = stamp_ms
            if stream_type == rs.stream.accel:
                accel_samples.append(vector)
            elif stream_type == rs.stream.gyro:
                gyro_samples.append(vector)

    sensor_started = False
    try:
        # Motion Module callback을 직접 사용한다. 이 D435i에서는 rs.pipeline의
        # wait_for_frames가 IMU-only stream을 timeout 내지만 sensor callback은
        # motion frame을 독립적으로 전달한다.
        motion_sensor.open([accel_profile, gyro_profile])
        motion_sensor.start(on_motion_frame)
        sensor_started = True
        warmup_deadline = time.monotonic() + args.warmup_s
        while time.monotonic() < warmup_deadline:
            time.sleep(0.02)
        with sample_lock:
            accel_samples.clear()
            gyro_samples.clear()
            last_motion_stamp_ms.clear()
            duplicate_motion_frame_count[0] = 0
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            time.sleep(0.02)
        with sample_lock:
            captured_accel = list(accel_samples)
            captured_gyro = list(gyro_samples)
            captured_duplicate_count = duplicate_motion_frame_count[0]
        if not captured_accel or not captured_gyro:
            raise RuntimeError(
                "D435i accelerometer/gyroscope sample을 모두 받지 못했습니다 "
                "(accel={}, gyro={})".format(len(captured_accel), len(captured_gyro))
            )

        created = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output_path = args.output_dir / ("d435i_imu_extrinsics_" + created + ".json")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "kind": "read_only_d435i_imu_extrinsics_probe",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "device": {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            },
            "stream_mode": "motion_only_usb2_compatible",
            "color_stream": None,
            "accelerometer_stream_fps": int(accel_profile.fps()),
            "gyroscope_stream_fps": int(gyro_profile.fps()),
            "duration_s": args.duration_s,
            "accel_to_color": None,
            "gyro_to_color": None,
            "accelerometer_m_s2": _vector_stats(np, captured_accel),
            "gyroscope_rad_s": _vector_stats(np, captured_gyro),
            "duplicate_motion_frame_count": captured_duplicate_count,
            "camera_to_base_extrinsic": None,
        }
        output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(
            "D435I_IMU_PROBE_OK accel={} gyro={} output={}".format(
                len(captured_accel), len(captured_gyro), output_path
            )
        )
        return 0
    finally:
        if sensor_started:
            motion_sensor.stop()
        if motion_sensor is not None:
            motion_sensor.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("INTERRUPTED: D435i IMU stream을 종료합니다.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
