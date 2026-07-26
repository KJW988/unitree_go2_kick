#!/usr/bin/env python3
"""D435i color-aligned RGB-D의 read-only capture와 calibration 기록.

Go2 motion, DDS, LowCmd, SportClient를 사용하지 않는다. D435i의 color/depth stream만
열어 color frame 좌표에 align된 depth frame과 device intrinsics를 기록한다.

출처: Intel librealsense SDK 2.0 pipeline/align API
https://github.com/realsenseai/librealsense/tree/v2.56.5/wrappers/python
여기서는 depth를 color image 좌표에 align해 이후 YOLO bbox 중심의 metric depth를
읽을 수 있도록 했다. camera-to-base extrinsic은 측정 전까지 의도적으로 기록하지 않는다.
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
            "D435i capture에는 project perception env의 numpy와 pyrealsense2가 필요합니다: {}".format(error)
        ) from error
    return np, rs


def _intrinsics_dict(intrinsics: Any) -> dict[str, Any]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "ppx": float(intrinsics.ppx),
        "ppy": float(intrinsics.ppy),
        "model": str(intrinsics.model),
        "coeffs": [float(value) for value in intrinsics.coeffs],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--output-dir", type=Path, default=Path("hardware_measurements"))
    args = parser.parse_args()
    if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
        parser.error("--duration-s must be positive and finite")
    if min(args.width, args.height, args.fps, args.warmup_frames) <= 0:
        parser.error("--width, --height, --fps, and --warmup-frames must be positive")

    np, rs = _require_runtime()
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = None
    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        depth_scale_m = float(depth_sensor.get_depth_scale())
        align_to_color = rs.align(rs.stream.color)
        for _ in range(args.warmup_frames):
            pipeline.wait_for_frames(timeout_ms=2000)

        deadline = time.monotonic() + args.duration_s
        frame_count, missing_pairs = 0, 0
        stamps_ms: list[float] = []
        valid_depth_fraction: list[float] = []
        last_color, last_depth, color_intrinsics = None, None, None
        while time.monotonic() < deadline:
            frames = align_to_color.process(pipeline.wait_for_frames(timeout_ms=2000))
            color_frame, depth_frame = frames.get_color_frame(), frames.get_depth_frame()
            if not color_frame or not depth_frame:
                missing_pairs += 1
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            if color.shape[:2] != depth.shape[:2] or color.ndim != 3 or depth.ndim != 2:
                missing_pairs += 1
                continue
            valid_depth_fraction.append(float(np.count_nonzero(depth) / depth.size))
            stamps_ms.append(float(color_frame.get_timestamp()))
            last_color, last_depth = color.copy(), depth.copy()
            color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            frame_count += 1
        if frame_count == 0 or last_color is None or last_depth is None or color_intrinsics is None:
            raise RuntimeError("valid aligned RGB-D frame을 얻지 못했습니다")

        created = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        capture_dir = args.output_dir / ("d435i_rgbd_capture_" + created)
        capture_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(capture_dir / "last_aligned_rgbd.npz", color_bgr=last_color, depth_raw=last_depth)
        intervals = np.diff(np.asarray(stamps_ms, dtype=np.float64))
        payload = {
            "schema_version": 1,
            "kind": "read_only_d435i_aligned_rgbd_capture",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "device": {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            },
            "requested_stream": {"width": args.width, "height": args.height, "fps": args.fps},
            "color_intrinsics": _intrinsics_dict(color_intrinsics),
            "depth_scale_m_per_unit": depth_scale_m,
            "depth_aligned_to": "color",
            "camera_to_base_extrinsic": None,
            "frame_count": frame_count,
            "missing_pair_count": missing_pairs,
            "duration_s": args.duration_s,
            "mean_valid_depth_fraction": float(np.mean(valid_depth_fraction)),
            "mean_frame_interval_ms": None if not len(intervals) else float(np.mean(intervals)),
            "last_frame_npz": "last_aligned_rgbd.npz",
        }
        (capture_dir / "metadata.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(
            "D435I_RGBD_CAPTURE_OK frames={} missing={} valid_depth={:.3f} output={}".format(
                frame_count, missing_pairs, payload["mean_valid_depth_fraction"], capture_dir
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
        print("INTERRUPTED: D435i stream을 종료합니다.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
