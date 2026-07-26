#!/usr/bin/env python3
"""표시된 calibration spot에서 D435i AprilTag mount baseline을 확정한다.

로봇은 사람이 표시된 바닥 위치와 정면 방향에 세워야 한다. 이 도구는 이미 생성된
read-only AprilTag probe 결과만 읽고, 로봇 camera pose의 기준값을 JSON으로 보존한다.
Go2 DDS, LowCmd, SportClient, MotionSwitcher를 사용하지 않는다.
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


def _find_tag(probe: Dict[str, Any], tag_id: int) -> Dict[str, Any]:
    for entry in probe.get("detected_tags", []):
        if int(entry.get("tag_id", -1)) == tag_id:
            return entry
    raise RuntimeError("probe에 Tag ID {}가 없습니다".format(tag_id))


def _pose(entry: Dict[str, Any], key: str, fallback_key: str) -> list[float]:
    value = entry.get(key, entry.get(fallback_key))
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError("probe의 {} pose 형식이 올바르지 않습니다".format(key))
    return [float(component) for component in value]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--tag-id", type=int, default=11)
    parser.add_argument("--output", type=Path, default=Path("hardware_measurements/d435i_tag_mount_baseline.json"))
    parser.add_argument("--operator-confirm", required=True)
    args = parser.parse_args()
    if args.operator_confirm != "CALIBRATION_SPOT_MARKED":
        parser.error("--operator-confirm CALIBRATION_SPOT_MARKED is required")
    probe = json.loads(args.probe.read_text(encoding="utf-8"))
    if probe.get("kind") != "read_only_d435i_apriltag_probe":
        raise RuntimeError("--probe가 D435i AprilTag probe JSON이 아닙니다")
    entry = _find_tag(probe, args.tag_id)
    detection_frames = int(entry.get("detection_frame_count", 0))
    if detection_frames < 30:
        raise RuntimeError("baseline에는 최소 30개 Tag detection frame이 필요합니다")
    baseline = {
        "schema_version": 1,
        "kind": "d435i_apriltag_mount_baseline",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_probe": str(args.probe),
        "device": probe.get("device"),
        "dictionary": probe.get("dictionary"),
        "tag_id": int(args.tag_id),
        "tag_size_m": float(probe["tag_size_m"]),
        "tag_center_height_m": float(probe["tag_center_height_m"]),
        "calibration_contract": "robot must stand at the marked floor spot facing this fixed upright wall Tag",
        "camera_translation_m": _pose(entry, "median_camera_translation_m", "best_camera_translation_m"),
        "camera_rotation_rvec": _pose(entry, "median_camera_rotation_rvec", "best_camera_rotation_rvec"),
        "translation_std_m": entry.get("translation_std_m"),
        "detection_frame_count": detection_frames,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(baseline, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("D435I_TAG_MOUNT_BASELINE_OK tag={} frames={} output={}".format(
        args.tag_id, detection_frames, args.output
    ))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("FAILED: {}".format(error))
        raise SystemExit(2)
