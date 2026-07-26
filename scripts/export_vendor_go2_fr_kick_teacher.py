#!/usr/bin/env python3
"""DDS/Isaac Gym 없이 기본 Go2 FR teacher를 50 Hz artifact로 export한다."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TEACHER_SOURCE = ROOT / "scripts" / "eval_vendor_go2_native_kick_teacher.py"
SAMPLE_HZ = 50.0
CANONICAL_DOF_NAMES = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
)
# canonical index -> Go2 LowCmd/MotorState index. 실물 영점/부호는 별도 검증 대상이다.
CANONICAL_TO_SDK_MOTOR_INDEX = (3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8)
# Native teacher가 읽는 vendor Go2 config의 nominal pose다.
DEFAULT_Q = np.asarray((0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5), dtype=np.float64)
PHASES = (1.00, 3.25, 3.60, 4.90)
LOAD_END, LIFT_HOLD_S, LIFT_FRACTION, SWING_FRACTION = 2.80, 0.12, 0.23, 0.60
NOMINAL = np.asarray((0.3335, -0.15, -0.21), dtype=np.float64)
LIFT = np.asarray((0.2500, -0.15, -0.125), dtype=np.float64)
BEZIER = np.asarray(((0.2500, -0.15, -0.125), (0.2850, -0.15, -0.115), (0.4300, -0.15, -0.165), (0.5600, -0.15, -0.210), (0.6000, -0.15, -0.220)), dtype=np.float64)
SUPPORT_OFFSETS = {
    "FL_hip_joint": -0.08, "FL_thigh_joint": 0.04, "FL_calf_joint": -0.07,
    "RL_hip_joint": -0.08, "RL_thigh_joint": 0.17, "RL_calf_joint": -0.28,
    "RR_hip_joint": -0.08, "RR_thigh_joint": 0.08, "RR_calf_joint": -0.14,
}


def minimum_jerk(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value ** 3 * (10.0 - 15.0 * value + 6.0 * value ** 2)


def source_sha256(path: Path) -> str | None:
    """원본 evaluator가 bundle에 없을 때도 self-contained exporter를 막지 않는다."""
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def offsets_at(elapsed_s: float) -> np.ndarray:
    """teacher의 기본 환경변수 preset과 같은 순수 NumPy 관절 offset을 반환한다."""
    index = {name: i for i, name in enumerate(CANONICAL_DOF_NAMES)}
    result = np.zeros(12, dtype=np.float64)
    stand_end, lift_end, kick_end, rest_end = PHASES
    preload = minimum_jerk((elapsed_s - stand_end) / (LOAD_END - stand_end))
    recovery = minimum_jerk((elapsed_s - kick_end) / (rest_end - kick_end))
    support_scale = preload * (1.0 - recovery)
    for name, value in SUPPORT_OFFSETS.items():
        result[index[name]] = support_scale * value
    thigh_i, calf_i = index["FR_thigh_joint"], index["FR_calf_joint"]
    toe = NOMINAL.copy()
    arrive_end = lift_end - LIFT_HOLD_S
    if LOAD_END <= elapsed_s < lift_end:
        toe = NOMINAL + minimum_jerk((elapsed_s - LOAD_END) / (arrive_end - LOAD_END)) * LIFT_FRACTION * (LIFT - NOMINAL)
    elif lift_end <= elapsed_s < kick_end:
        u = minimum_jerk((elapsed_s - lift_end) / (kick_end - lift_end))
        v = 1.0 - u
        point = v ** 4 * BEZIER[0] + 4 * v ** 3 * u * BEZIER[1] + 6 * v ** 2 * u ** 2 * BEZIER[2] + 4 * v * u ** 3 * BEZIER[3] + u ** 4 * BEZIER[4]
        toe = NOMINAL + SWING_FRACTION * (point - NOMINAL)
    elif kick_end <= elapsed_s < rest_end:
        endpoint = NOMINAL + SWING_FRACTION * (BEZIER[4] - NOMINAL)
        u = minimum_jerk((elapsed_s - kick_end) / (rest_end - kick_end))
        toe = endpoint * (1.0 - u) + NOMINAL * u
        toe[2] += 0.08 * 16.0 * u ** 2 * (1.0 - u) ** 2
    else:
        return result
    # teacher와 같은 planar two-link IK와 clamp 범위를 사용한다.
    q1, q2 = float(DEFAULT_Q[thigh_i]), float(DEFAULT_Q[calf_i])
    upper, tip_x, tip_z = 0.213, 0.020, -0.148
    default_x = -upper * math.sin(q1) + tip_x * math.cos(q1 + q2) + tip_z * math.sin(q1 + q2)
    default_z = -upper * math.cos(q1) - tip_x * math.sin(q1 + q2) + tip_z * math.cos(q1 + q2)
    x, z = default_x + toe[0] - NOMINAL[0], default_z + toe[2] - NOMINAL[2]
    lower, beta = math.hypot(tip_x, -tip_z), math.atan2(-tip_z, tip_x)
    cosine = (x * x + (-z) * (-z) - upper * upper - lower * lower) / (2 * upper * lower)
    elbow = -math.acos(max(-0.999, min(0.999, cosine)))
    shoulder = math.atan2(-z, x) - math.atan2(lower * math.sin(elbow), upper + lower * math.cos(elbow))
    result[thigh_i] += max(-1.35, min(2.40, shoulder - math.pi / 2.0)) - q1
    result[calf_i] += max(-2.55, min(-0.90, elbow - beta + math.pi / 2.0)) - q2
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="출력 .npz; 같은 위치에 .csv 생성")
    parser.add_argument("--duration-s", type=float, default=6.0)
    args = parser.parse_args()
    if args.output.suffix != ".npz":
        parser.error("--output must end in .npz")
    if not args.output.parent.is_dir():
        parser.error("output directory does not exist: {}".format(args.output.parent))
    if args.duration_s < PHASES[-1] or not math.isclose(args.duration_s * SAMPLE_HZ, round(args.duration_s * SAMPLE_HZ), abs_tol=1e-9):
        parser.error("--duration-s must cover 4.90 s and be a 1/50 s multiple")
    sample_count = int(round(args.duration_s * SAMPLE_HZ))
    times = np.arange(sample_count, dtype=np.float64) / SAMPLE_HZ
    offsets = np.vstack([offsets_at(time_s) for time_s in times])
    canonical = DEFAULT_Q[None, :] + offsets
    sdk = np.empty_like(canonical)
    sdk[:, CANONICAL_TO_SDK_MOTOR_INDEX] = canonical
    planned_qd = np.vstack((np.zeros((1, 12)), np.diff(canonical, axis=0) * SAMPLE_HZ))
    metadata = {
        "schema_version": 1, "artifact_kind": "offline_go2_fr_kick_teacher", "hardware_io": "none",
        "sample_hz": SAMPLE_HZ, "duration_s": args.duration_s,
        "teacher_source": str(TEACHER_SOURCE.relative_to(ROOT)) if TEACHER_SOURCE.is_file() else "embedded_exporter",
        "teacher_source_sha256": source_sha256(TEACHER_SOURCE),
        "canonical_dof_names": list(CANONICAL_DOF_NAMES), "canonical_to_sdk_motor_index": list(CANONICAL_TO_SDK_MOTOR_INDEX),
        "sim_default_dof_pos_rad": DEFAULT_Q.tolist(), "phases_s": list(PHASES),
        "units": {"position": "rad", "planned_finite_difference_velocity": "rad/s"},
        "physical_status": "unattested_do_not_send_to_robot",
    }
    np.savez_compressed(args.output, time_s=times, q_canonical_rad=canonical, q_sdk_motor_order_rad=sdk, planned_qd_canonical_rad_s=planned_qd, metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)))
    csv_path = args.output.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s"] + ["canonical_" + name for name in CANONICAL_DOF_NAMES] + ["sdk_motor_{:02d}_q_rad".format(i) for i in range(12)])
        for time_s, q_canonical, q_sdk in zip(times, canonical, sdk):
            writer.writerow(["{:.8f}".format(time_s)] + ["{:.10f}".format(value) for value in q_canonical] + ["{:.10f}".format(value) for value in q_sdk])
    print("OFFLINE_EXPORT_OK samples={} hz={} npz={} csv={} hardware_io=none".format(sample_count, SAMPLE_HZ, args.output, csv_path))


if __name__ == "__main__":
    main()
