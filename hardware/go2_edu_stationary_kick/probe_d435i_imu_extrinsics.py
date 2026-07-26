#!/usr/bin/env python3
"""D435i factory IMU↔RGB 변환과 정지 IMU 통계를 read-only로 기록한다.

Go2 DDS, LowCmd, SportClient, MotionSwitcher를 사용하지 않는다. D435i의 color,
accelerometer, gyroscope stream만 열어 camera 내부 factory extrinsic과 정지 시 IMU
통계를 기록한다.

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


def _extrinsics_dict(extrinsics: Any) -> dict[str, Any]:
    return {
        "rotation_row_major": [float(value) for value in extrinsics.rotation],
        "translation_m": [float(value) for value in extrinsics.translation],
    }


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
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--output-dir", type=Path, default=Path("hardware_measurements"))
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        parser.error("--duration-s must be positive and finite")
    if not math.isfinite(args.warmup_s) or args.warmup_s < 0.0:
        parser.error("--warmup-s must be finite and non-negative")
    if min(args.width, args.height, args.fps) <= 0:
        parser.error("--width, --height, and --fps must be positive")

    np, rs = _require_runtime()
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    # D435i firmware별로 가능한 motion rate 조합이 다르므로 특정 Hz를 강제하지
    # 않는다. config가 device의 기본 accel/gyro profile을 선택하게 해야 RGB와
    # 동시에 안정적으로 열 수 있다.
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)
    profile = None
    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        accel_profile = profile.get_stream(rs.stream.accel).as_motion_stream_profile()
        gyro_profile = profile.get_stream(rs.stream.gyro).as_motion_stream_profile()
        accel_to_color = accel_profile.get_extrinsics_to(color_profile)
        gyro_to_color = gyro_profile.get_extrinsics_to(color_profile)

        warmup_deadline = time.monotonic() + args.warmup_s
        while time.monotonic() < warmup_deadline:
            pipeline.wait_for_frames(timeout_ms=2000)

        accel_samples: list[list[float]] = []
        gyro_samples: list[list[float]] = []
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            for frame in frames:
                if not frame.is_motion_frame():
                    continue
                motion = frame.as_motion_frame().get_motion_data()
                vector = [float(motion.x), float(motion.y), float(motion.z)]
                stream_type = frame.get_profile().stream_type()
                if stream_type == rs.stream.accel:
                    accel_samples.append(vector)
                elif stream_type == rs.stream.gyro:
                    gyro_samples.append(vector)
        if not accel_samples or not gyro_samples:
            raise RuntimeError(
                "D435i accelerometer/gyroscope sample을 모두 받지 못했습니다 "
                "(accel={}, gyro={})".format(len(accel_samples), len(gyro_samples))
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
            "color_stream": {"width": args.width, "height": args.height, "fps": args.fps},
            "accelerometer_stream_fps": int(accel_profile.fps()),
            "gyroscope_stream_fps": int(gyro_profile.fps()),
            "duration_s": args.duration_s,
            "accel_to_color": _extrinsics_dict(accel_to_color),
            "gyro_to_color": _extrinsics_dict(gyro_to_color),
            "accelerometer_m_s2": _vector_stats(np, accel_samples),
            "gyroscope_rad_s": _vector_stats(np, gyro_samples),
            "camera_to_base_extrinsic": None,
        }
        output_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(
            "D435I_IMU_PROBE_OK accel={} gyro={} output={}".format(
                len(accel_samples), len(gyro_samples), output_path
            )
        )
        return 0
    finally:
        if profile is not None:
            pipeline.stop()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("INTERRUPTED: D435i IMU stream을 종료합니다.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
