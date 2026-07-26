#!/usr/bin/env python3
"""현재 D435i AprilTag pose를 marked calibration spot baseline과 비교한다.

이 도구는 camera mount 이동을 read-only로 감지한다. translation/rotation 차이가
threshold를 넘으면 FAIL을 반환하며, 킥·보행·DDS 명령은 전혀 발행하지 않는다.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict


def _find_tag(probe: Dict[str, Any], tag_id: int) -> Dict[str, Any]:
    for entry in probe.get("detected_tags", []):
        if int(entry.get("tag_id", -1)) == tag_id:
            return entry
    raise RuntimeError("current probe에 Tag ID {}가 없습니다".format(tag_id))


def _vector(entry: Dict[str, Any], preferred: str, fallback: str) -> list[float]:
    value = entry.get(preferred, entry.get(fallback))
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError("{} pose 형식이 올바르지 않습니다".format(preferred))
    return [float(component) for component in value]


def _rotation_matrix(rvec: list[float]) -> list[list[float]]:
    theta = math.sqrt(sum(component * component for component in rvec))
    if theta < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    x, y, z = (component / theta for component in rvec)
    cross = [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]]
    cross_squared = [
        [sum(cross[row][index] * cross[index][column] for index in range(3)) for column in range(3)]
        for row in range(3)
    ]
    sine, cosine = math.sin(theta), math.cos(theta)
    return [
        [
            (1.0 if row == column else 0.0)
            + sine * cross[row][column]
            + (1.0 - cosine) * cross_squared[row][column]
            for column in range(3)
        ]
        for row in range(3)
    ]


def _rotation_delta_deg(reference: list[float], current: list[float]) -> float:
    reference_matrix, current_matrix = _rotation_matrix(reference), _rotation_matrix(current)
    relative_trace = sum(
        reference_matrix[index][row] * current_matrix[index][row]
        for row in range(3) for index in range(3)
    )
    cosine = max(-1.0, min(1.0, 0.5 * (relative_trace - 1.0)))
    return math.degrees(math.acos(cosine))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--translation-threshold-m", type=float, default=0.03)
    parser.add_argument("--rotation-threshold-deg", type=float, default=3.0)
    args = parser.parse_args()
    if args.translation_threshold_m <= 0.0 or args.rotation_threshold_deg <= 0.0:
        parser.error("thresholds must be positive")
    baseline = json.loads(args.baseline.read_text(encoding="utf-8"))
    probe = json.loads(args.probe.read_text(encoding="utf-8"))
    if baseline.get("kind") != "d435i_apriltag_mount_baseline":
        raise RuntimeError("--baseline 형식이 올바르지 않습니다")
    tag_id = int(baseline["tag_id"])
    entry = _find_tag(probe, tag_id)
    baseline_translation = _vector(baseline, "camera_translation_m", "camera_translation_m")
    current_translation = _vector(entry, "median_camera_translation_m", "best_camera_translation_m")
    baseline_rotation = _vector(baseline, "camera_rotation_rvec", "camera_rotation_rvec")
    current_rotation = _vector(entry, "median_camera_rotation_rvec", "best_camera_rotation_rvec")
    translation_delta = math.sqrt(sum(
        (current_translation[index] - baseline_translation[index]) ** 2 for index in range(3)
    ))
    rotation_delta_deg = _rotation_delta_deg(baseline_rotation, current_rotation)
    verdict = translation_delta <= args.translation_threshold_m and rotation_delta_deg <= args.rotation_threshold_deg
    payload = {
        "kind": "d435i_apriltag_mount_verification",
        "tag_id": tag_id,
        "translation_delta_m": translation_delta,
        "rotation_delta_deg": rotation_delta_deg,
        "translation_threshold_m": args.translation_threshold_m,
        "rotation_threshold_deg": args.rotation_threshold_deg,
        "verdict": "PASS" if verdict else "FAIL",
    }
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    return 0 if verdict else 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("FAILED: {}".format(error))
        raise SystemExit(2)
