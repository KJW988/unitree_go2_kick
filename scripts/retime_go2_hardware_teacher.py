#!/usr/bin/env python3
"""frozen Go2 teacher를 명시적 joint speed/acceleration/jerk envelope로 재시간화한다.

이 도구는 offline NPZ 변환만 수행한다. 각 원본 50 Hz 관절 segment를 endpoint에서
속도·가속도가 0인 minimum-jerk curve로 바꿔 hardware command envelope를 넘지 않게
한다. target geometry와 canonical joint path는 변경하지 않는다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, List

import numpy as np

from dry_run_go2_fr_kick_deploy import load_trajectory, validate_artifact

SAMPLE_HZ = 50.0
DT_S = 1.0 / SAMPLE_HZ
# s(u)=10u^3-15u^4+6u^5 의 정확한 derivative peak.
MIN_JERK_MAX_VELOCITY = 1.875
MIN_JERK_MAX_ACCELERATION = 10.0 / math.sqrt(3.0)
MIN_JERK_MAX_JERK = 60.0


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _minimum_jerk(value: float) -> float:
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def _ticks_for_segment(
    delta: np.ndarray,
    speed: np.ndarray,
    acceleration: np.ndarray,
    jerk: np.ndarray,
) -> int:
    magnitude = np.abs(delta)
    required_s = max(
        DT_S,
        float(np.max(MIN_JERK_MAX_VELOCITY * magnitude / speed)),
        float(np.max(np.sqrt(MIN_JERK_MAX_ACCELERATION * magnitude / acceleration))),
        float(np.max(np.cbrt(MIN_JERK_MAX_JERK * magnitude / jerk))),
    )
    return max(1, int(math.ceil(required_s * SAMPLE_HZ)))


def _retime(
    canonical: np.ndarray,
    speed: np.ndarray,
    acceleration: np.ndarray,
    jerk: np.ndarray,
) -> np.ndarray:
    result: List[np.ndarray] = [canonical[0].copy()]
    for previous, target in zip(canonical[:-1], canonical[1:]):
        ticks = _ticks_for_segment(target - previous, speed, acceleration, jerk)
        for tick in range(1, ticks + 1):
            result.append(previous + _minimum_jerk(tick / ticks) * (target - previous))
    return np.asarray(result, dtype=np.float64)


def _positive_vector(value: float, name: str) -> np.ndarray:
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("{} must be positive and finite".format(name))
    return np.full(12, value, dtype=np.float64)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-speed-rad-s", type=float, required=True)
    parser.add_argument("--max-acceleration-rad-s2", type=float, required=True)
    parser.add_argument("--max-jerk-rad-s3", type=float, required=True)
    args = parser.parse_args()
    if not args.output.parent.is_dir():
        parser.error("--output parent does not exist: {}".format(args.output.parent))
    try:
        speed = _positive_vector(args.max_speed_rad_s, "--max-speed-rad-s")
        acceleration = _positive_vector(args.max_acceleration_rad_s2, "--max-acceleration-rad-s2")
        jerk = _positive_vector(args.max_jerk_rad_s3, "--max-jerk-rad-s3")
    except ValueError as error:
        parser.error(str(error))

    teacher = load_trajectory(args.teacher)
    errors = validate_artifact(teacher)
    if errors:
        parser.error("teacher artifact invalid: {}".format("; ".join(errors)))
    canonical = _retime(teacher["q_canonical_rad"], speed, acceleration, jerk)
    time_s = np.arange(len(canonical), dtype=np.float64) * DT_S
    sdk = np.empty_like(canonical)
    mapping = tuple(teacher["metadata"]["canonical_to_sdk_motor_index"])
    sdk[:, mapping] = canonical
    qd = np.vstack((np.zeros((1, 12)), np.diff(canonical, axis=0) * SAMPLE_HZ))
    metadata: dict[str, Any] = dict(teacher["metadata"])
    metadata.update({
        "artifact_kind": "offline_go2_fr_kick_teacher_retimed",
        "retime_source": str(args.teacher),
        "retime_source_sha256": _sha256(args.teacher),
        "retime_sample_hz": SAMPLE_HZ,
        "retime_max_speed_rad_s": args.max_speed_rad_s,
        "retime_max_acceleration_rad_s2": args.max_acceleration_rad_s2,
        "retime_max_jerk_rad_s3": args.max_jerk_rad_s3,
        "physical_status": "unattested_do_not_send_to_robot",
        "hardware_io": "none",
    })
    np.savez_compressed(
        args.output,
        time_s=time_s,
        q_canonical_rad=canonical,
        q_sdk_motor_order_rad=sdk,
        planned_qd_canonical_rad_s=qd,
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    print(
        "RETIME_OK output={} samples={} duration_s={:.3f} max_discrete_speed_rad_s={:.6f} hardware_io=none".format(
            args.output, len(canonical), float(time_s[-1]), float(np.max(np.abs(qd)))
        )
    )


if __name__ == "__main__":
    main()
