#!/usr/bin/env python3
"""D435i RGB에서 AprilTag pose를 read-only로 검출·기록한다.

Go2 DDS, LowCmd, SportClient, MotionSwitcher를 사용하지 않는다. D435i color
stream과 OpenCV ArUco AprilTag detector만 사용해 모든 검출 Tag의 ID와 color-camera
frame pose를 저장한다. camera→Go2 base extrinsic이 아직 검증되지 않았으므로 이 도구는
base-frame 좌표나 킥 target을 생성하지 않는다.

출처: OpenCV ArUco/AprilTag detector와 pose API
https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
기존 dribblebot.perception.apriltag_camera의 동일한 DICT_APRILTAG_36h11/pose
convention을 실제 D435i read-only probe에 적용했다.
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


def _require_runtime() -> tuple[Any, Any, Any]:
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "D435i AprilTag probe에는 perception env의 cv2, numpy, pyrealsense2가 필요합니다: {}".format(error)
        ) from error
    return cv2, np, rs


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


def _area(np: Any, corners: Any) -> float:
    points = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    return 0.5 * abs(float(np.dot(points[:, 0], np.roll(points[:, 1], -1))
                           - np.dot(points[:, 1], np.roll(points[:, 0], -1))))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag-size-m", type=float, default=0.152,
                        help="printed black outer square side length")
    parser.add_argument("--tag-center-height-m", type=float, default=0.300,
                        help="wall Tag geometric center height above floor; metadata only")
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--min-area-px", type=float, default=250.0)
    parser.add_argument("--output-dir", type=Path, default=Path("hardware_measurements"))
    args = parser.parse_args()
    numeric_values = (args.tag_size_m, args.tag_center_height_m, args.duration_s, args.min_area_px)
    if any(not math.isfinite(value) for value in numeric_values):
        parser.error("floating-point arguments must be finite")
    if args.tag_size_m <= 0.0 or args.tag_center_height_m < 0.0 or args.duration_s <= 0.0 or args.min_area_px <= 0.0:
        parser.error("tag size/duration/min area must be positive and center height must be non-negative")
    if min(args.warmup_frames, args.width, args.height, args.fps) <= 0:
        parser.error("warmup frames, width, height, and fps must be positive")

    cv2, np, rs = _require_runtime()
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, args.dictionary):
        raise RuntimeError("OpenCV ArUco AprilTag dictionary {} is unavailable".format(args.dictionary))
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    profile = None
    try:
        profile = pipeline.start(config)
        device = profile.get_device()
        for _ in range(args.warmup_frames):
            pipeline.wait_for_frames(timeout_ms=2000)

        detections: dict[int, dict[str, Any]] = {}
        frame_count, rejected_small_count = 0, 0
        largest_annotated_area_px = 0.0
        best_image, color_intrinsics = None, None
        deadline = time.monotonic() + args.duration_s
        while time.monotonic() < deadline:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            image = np.asanyarray(color_frame.get_data())
            corners, ids, _ = detector.detectMarkers(image)
            frame_count += 1
            if ids is None:
                continue
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            camera_matrix = np.array(
                [[intrinsics.fx, 0.0, intrinsics.ppx], [0.0, intrinsics.fy, intrinsics.ppy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            distortion = np.asarray(intrinsics.coeffs, dtype=np.float64).reshape(-1, 1)
            valid_corners, valid_ids, valid_areas = [], [], []
            for marker_corners, marker_id in zip(corners, ids.reshape(-1).astype(int)):
                area = _area(np, marker_corners)
                if area < args.min_area_px:
                    rejected_small_count += 1
                    continue
                valid_corners.append(marker_corners)
                valid_ids.append(int(marker_id))
                valid_areas.append(area)
            if not valid_corners:
                continue
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                valid_corners, float(args.tag_size_m), camera_matrix, distortion
            )
            annotated = image.copy()
            cv2.aruco.drawDetectedMarkers(annotated, valid_corners, np.asarray(valid_ids, dtype=np.int32).reshape(-1, 1))
            for marker_corners, marker_id, area, rvec, tvec in zip(
                valid_corners, valid_ids, valid_areas, rvecs[:, 0, :], tvecs[:, 0, :]
            ):
                camera_translation = [float(value) for value in tvec]
                camera_rotation_rvec = [float(value) for value in rvec]
                current = detections.get(marker_id)
                if current is None:
                    current = {
                        "tag_id": marker_id,
                        "detection_frame_count": 0,
                        "largest_area_px": 0.0,
                        "best_camera_translation_m": None,
                        "best_camera_rotation_rvec": None,
                        "best_optical_range_m": None,
                    }
                    detections[marker_id] = current
                current["detection_frame_count"] += 1
                if area >= current["largest_area_px"]:
                    current["largest_area_px"] = float(area)
                    current["best_camera_translation_m"] = camera_translation
                    current["best_camera_rotation_rvec"] = camera_rotation_rvec
                    current["best_optical_range_m"] = float(camera_translation[2])
                cv2.drawFrameAxes(
                    annotated, camera_matrix, distortion, np.asarray(rvec).reshape(3, 1),
                    np.asarray(tvec).reshape(3, 1), args.tag_size_m * 0.5,
                )
                if area >= largest_annotated_area_px:
                    largest_annotated_area_px = float(area)
                    best_image = annotated
                    color_intrinsics = intrinsics
        created = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        capture_dir = args.output_dir / ("d435i_apriltag_probe_" + created)
        capture_dir.mkdir(parents=True, exist_ok=True)
        image_name = None
        if best_image is not None:
            image_name = "best_detection.jpg"
            if not cv2.imwrite(str(capture_dir / image_name), best_image):
                raise RuntimeError("annotated AprilTag image 저장에 실패했습니다")
        payload = {
            "schema_version": 1,
            "kind": "read_only_d435i_apriltag_probe",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "device": {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            },
            "dictionary": args.dictionary,
            "tag_size_m": args.tag_size_m,
            "tag_center_height_m": args.tag_center_height_m,
            "color_intrinsics": None if color_intrinsics is None else _intrinsics_dict(color_intrinsics),
            "frame_count": frame_count,
            "rejected_small_count": rejected_small_count,
            "detected_tags": [detections[tag_id] for tag_id in sorted(detections)],
            "best_annotated_image": image_name,
            "camera_to_base_extrinsic": None,
            "ground_projection": None,
        }
        (capture_dir / "metadata.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if detections:
            print("D435I_APRILTAG_PROBE_OK tags={} frames={} output={}".format(
                sorted(detections), frame_count, capture_dir
            ))
        else:
            print("D435I_APRILTAG_PROBE_NO_DETECTION frames={} output={}".format(frame_count, capture_dir))
        return 0
    finally:
        if profile is not None:
            pipeline.stop()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("INTERRUPTED: D435i AprilTag stream을 종료합니다.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
