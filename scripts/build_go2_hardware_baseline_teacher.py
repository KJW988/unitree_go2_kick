#!/usr/bin/env python3
"""read-only Go2 static capture를 시작점으로 frozen FR teacher artifact를 재표현한다.

실물에 어떠한 DDS 연결이나 명령을 만들지 않는다. 기존 teacher의 시간별 canonical
관절 변화량만 보존하고, t=0 자세를 capture의 SDK raw motor position으로 치환한다.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from dry_run_go2_fr_kick_deploy import (
    EXPECTED_CANONICAL_TO_SDK,
    load_trajectory,
    validate_artifact,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _load_capture(path: Path) -> np.ndarray:
    try:
        with path.open(encoding="utf-8") as handle:
            capture: dict[str, Any] = json.load(handle)
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError("capture를 읽을 수 없습니다: {}".format(error)) from error
    if capture.get("kind") != "read_only_go2_static_stand_capture":
        raise ValueError("read_only_go2_static_stand_capture capture가 아닙니다")
    try:
        q_sdk = np.asarray(capture["q_sdk_order_median_rad"], dtype=np.float64)
        span = np.asarray(capture["q_sdk_order_span_rad"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("capture q SDK median/span이 유효하지 않습니다") from error
    if q_sdk.shape != (12,) or span.shape != (12,) or not np.isfinite(q_sdk).all() or not np.isfinite(span).all():
        raise ValueError("capture q SDK median/span은 finite 12개여야 합니다")
    if float(np.max(span)) > 0.005:
        raise ValueError("capture가 정지 상태가 아닙니다: max q span {:.6f} rad".format(float(np.max(span))))
    return q_sdk


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher", type=Path, required=True, help="offline frozen teacher .npz")
    parser.add_argument("--capture", type=Path, required=True, help="read-only static stand JSON")
    parser.add_argument("--output", type=Path, required=True, help="new .npz; parent must exist")
    args = parser.parse_args()
    if not args.output.parent.is_dir():
        parser.error("--output parent does not exist: {}".format(args.output.parent))

    teacher = load_trajectory(args.teacher)
    errors = validate_artifact(teacher)
    if errors:
        parser.error("teacher artifact invalid: {}".format("; ".join(errors)))
    q_sdk_baseline = _load_capture(args.capture)
    canonical_baseline = q_sdk_baseline[np.asarray(EXPECTED_CANONICAL_TO_SDK, dtype=np.int64)]
    teacher_delta = teacher["q_canonical_rad"] - teacher["q_canonical_rad"][0]
    canonical = canonical_baseline[None, :] + teacher_delta
    sdk = np.empty_like(canonical)
    sdk[:, EXPECTED_CANONICAL_TO_SDK] = canonical
    qd = np.vstack((np.zeros((1, 12)), np.diff(canonical, axis=0) * 50.0))
    metadata = dict(teacher["metadata"])
    metadata.update({
        "artifact_kind": "offline_go2_fr_kick_teacher_hardware_baseline",
        "physical_status": "unattested_do_not_send_to_robot",
        "hardware_baseline_capture": str(args.capture),
        "hardware_baseline_capture_sha256": _sha256(args.capture),
        "hardware_baseline_sdk_q_rad": q_sdk_baseline.tolist(),
        "hardware_io": "none",
    })
    np.savez_compressed(
        args.output,
        time_s=teacher["time_s"],
        q_canonical_rad=canonical,
        q_sdk_motor_order_rad=sdk,
        planned_qd_canonical_rad_s=qd,
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    print(
        "HARDWARE_BASELINE_ARTIFACT_OK output={} max_teacher_delta_rad={:.6f} hardware_io=none".format(
            args.output, float(np.max(np.abs(teacher_delta)))
        )
    )


if __name__ == "__main__":
    main()
