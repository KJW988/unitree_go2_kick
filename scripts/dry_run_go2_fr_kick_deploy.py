#!/usr/bin/env python3
"""DDS/SDK 없이 Go2 FR teacher artifact와 hardware attestation을 fail-closed 검토한다.

이 파일에는 --execute 옵션도 Unitree SDK import도 없다. 실물 명령을 만들 수 없다.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

EXPECTED_DOF_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)
EXPECTED_CANONICAL_TO_SDK = (3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8)
REQUIRED = (
    "schema_version", "robot_model", "firmware_version", "verified_by", "verified_at_utc",
    "teacher_dof_names", "canonical_to_sdk_motor_index", "position_scale",
    "position_offset_rad", "position_limits_rad", "torque_limits_nm",
    "validated_command_speed_limit_rad_s",
)


def load_trajectory(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError("trajectory not found: {}".format(path))
    with np.load(path, allow_pickle=False) as archive:
        required = {"time_s", "q_canonical_rad", "q_sdk_motor_order_rad", "planned_qd_canonical_rad_s", "metadata_json"}
        missing = required.difference(archive.files)
        if missing:
            raise ValueError("trajectory missing fields: {}".format(", ".join(sorted(missing))))
        value = {name: archive[name].copy() for name in required if name != "metadata_json"}
        value["metadata"] = json.loads(str(archive["metadata_json"].item()))
        return value


def validate_artifact(data: Dict[str, Any]) -> List[str]:
    reasons: List[str] = []
    metadata, time_s = data["metadata"], data["time_s"]
    canonical, sdk, qd = data["q_canonical_rad"], data["q_sdk_motor_order_rad"], data["planned_qd_canonical_rad_s"]
    if metadata.get("schema_version") != 1:
        reasons.append("unsupported trajectory schema")
    if tuple(metadata.get("canonical_dof_names", ())) != EXPECTED_DOF_NAMES:
        reasons.append("canonical dof_names mismatch")
    if tuple(metadata.get("canonical_to_sdk_motor_index", ())) != EXPECTED_CANONICAL_TO_SDK:
        reasons.append("canonical-to-SDK motor mapping mismatch")
    if time_s.ndim != 1 or canonical.shape != (len(time_s), 12) or sdk.shape != canonical.shape or qd.shape != canonical.shape:
        return reasons + ["trajectory array shape mismatch"]
    if not all(np.isfinite(item).all() for item in (time_s, canonical, sdk, qd)):
        reasons.append("trajectory contains non-finite values")
    if len(time_s) > 1 and not np.allclose(np.diff(time_s), 1.0 / 50.0, atol=1e-9, rtol=0.0):
        reasons.append("trajectory is not exactly 50 Hz")
    expected_sdk = np.empty_like(canonical)
    expected_sdk[:, EXPECTED_CANONICAL_TO_SDK] = canonical
    if not np.allclose(sdk, expected_sdk, atol=1e-12, rtol=0.0):
        reasons.append("SDK-order positions do not match canonical positions")
    return reasons


def validate_attestation(path: Path, canonical: np.ndarray) -> List[str]:
    try:
        with path.open(encoding="utf-8") as handle:
            item = json.load(handle)
    except (OSError, json.JSONDecodeError) as error:
        return ["hardware attestation cannot be read: {}".format(error)]
    reasons = ["hardware attestation missing field: {}".format(key) for key in REQUIRED if key not in item]
    if reasons:
        return reasons
    if item["robot_model"] != "Go2 EDU":
        reasons.append("attestation robot_model must be Go2 EDU")
    if not all(str(item[key]).strip() for key in ("firmware_version", "verified_by", "verified_at_utc")):
        reasons.append("attestation identity or firmware version is empty")
    if tuple(item["teacher_dof_names"]) != EXPECTED_DOF_NAMES or tuple(item["canonical_to_sdk_motor_index"]) != EXPECTED_CANONICAL_TO_SDK:
        reasons.append("attested joint mapping mismatch")
    try:
        scale = np.asarray(item["position_scale"], dtype=np.float64)
        offset = np.asarray(item["position_offset_rad"], dtype=np.float64)
        limits = np.asarray(item["position_limits_rad"], dtype=np.float64)
        torques = np.asarray(item["torque_limits_nm"], dtype=np.float64)
        speed_limits = np.asarray(item["validated_command_speed_limit_rad_s"], dtype=np.float64)
    except (TypeError, ValueError):
        return reasons + ["attestation numeric fields are invalid"]
    if scale.shape != (12,) or offset.shape != (12,) or limits.shape != (12, 2) or torques.shape != (12,) or speed_limits.shape != (12,):
        return reasons + ["attestation shapes must be 12, 12, 12x2, 12, 12"]
    if not all(np.isfinite(value).all() for value in (scale, offset, limits, torques, speed_limits)):
        return reasons + ["attestation contains non-finite values"]
    if np.any(limits[:, 0] >= limits[:, 1]) or np.any(torques <= 0.0) or np.any(speed_limits <= 0.0):
        return reasons + ["attested position, torque, or validated command speed limits are invalid"]
    physical_q = canonical * scale[None, :] + offset[None, :]
    bad = np.argwhere((physical_q < limits[:, 0]) | (physical_q > limits[:, 1]))
    if bad.size:
        joints = sorted({EXPECTED_DOF_NAMES[int(index)] for _, index in bad})
        reasons.append("attested position limits exceeded: {}".format(", ".join(joints)))
    return reasons


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--hardware-attestation", type=Path, help="없으면 안전상 NOT_ARMABLE")
    parser.add_argument("--dry-run", action="store_true", help="명시적 no-op; 이 도구는 항상 dry-run")
    args = parser.parse_args()
    data = load_trajectory(args.trajectory)
    canonical = data["q_canonical_rad"]
    max_step = float(np.max(np.abs(np.diff(canonical, axis=0)))) if len(canonical) > 1 else 0.0
    max_speed = float(np.max(np.abs(data["planned_qd_canonical_rad_s"])))
    print("DRY_RUN artifact={} samples={} max_step_rad={:.6f} max_planned_speed_rad_s={:.6f} DDS_DISABLED=true".format(args.trajectory, len(data["time_s"]), max_step, max_speed))
    reasons = validate_artifact(data)
    if args.hardware_attestation is None:
        reasons.append("hardware mapping attestation is absent")
    else:
        reasons.extend(validate_attestation(args.hardware_attestation, canonical))
    if reasons:
        for reason in reasons:
            print("NOT_ARMABLE: {}".format(reason))
        print("DRY_RUN_BLOCKED: no SDK import, no DDS publisher, no robot command was created.")
        raise SystemExit(2)
    print("DRY_RUN_READY_NOT_EXECUTABLE: artifact and attestation are internally consistent.")
    print("DRY_RUN_ONLY: this program has no hardware execution mode.")


if __name__ == "__main__":
    main()
