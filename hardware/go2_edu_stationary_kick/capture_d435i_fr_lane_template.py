#!/usr/bin/env python3
"""FR toe→ball→Tag 선상에서의 camera-visible staging template을 read-only로 기록한다.

이 파일은 D435i stream의 ``state.json``만 읽으며 WebRTC, DDS, LowCmd, robot motion을
전혀 사용하지 않는다. template을 만들기 전에 operator가 실제 바닥에서 다음을 직접 맞춘다.

1. frozen FR kick의 **FR toe swing/contact lane**, ball center, Tag 지면투영점이 한 선이다.
2. 그 최종 FR kick pose에서 robot을 같은 yaw/lateral offset으로 0.65–0.85 m만큼 뒤로
   물려, D435i가 ball과 Tag를 동시에 보는 staging spot에 둔다.

따라서 output은 camera→base→FR extrinsic을 추정하는 파일이 아니라, 실물 FR lane에서
관측된 ball/Tag camera bearing의 template이다. 후속 MCF stage controller는 이 template의
ball bearing을 0으로 만들지 않는다.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from stage_go2_mcf_ball_tag_webrtc import Perception, angle_distance, fetch_perception


CONFIRMATION = "FR_LANE_TEMPLATE_SPOT_MARKED"


def circular_median(values: list[float]) -> float:
    """작은 staging bearing spread에서 wrap-safe median을 고른다."""
    if not values:
        raise ValueError("circular median에는 sample이 필요합니다")
    return min(values, key=lambda value: sum(abs(angle_distance(value, other)) for other in values))


def collect(args: argparse.Namespace) -> tuple[list[Perception], str]:
    samples: list[Perception] = []
    for index in range(args.sample_count):
        sample = fetch_perception(args.perception_url, args.tag_id, args.http_timeout_s)
        if sample is None:
            return [], "perception_missing"
        if sample.age_s > args.perception_max_age_s:
            return [], "perception_stale"
        if sample.ball_confidence < args.min_ball_confidence:
            return [], "ball_confidence_low"
        if not args.stage_range_min_m <= sample.ball_range_m <= args.stage_range_max_m:
            return [], "ball_not_in_camera_staging_range"
        samples.append(sample)
        if index + 1 < args.sample_count:
            time.sleep(args.sample_interval_s)
    ball_bearing = circular_median([sample.ball_bearing_rad for sample in samples])
    target_bearing = circular_median([sample.target_bearing_rad for sample in samples])
    range_m = float(statistics.median(sample.ball_range_m for sample in samples))
    if max(abs(sample.ball_range_m - range_m) for sample in samples) > args.max_range_jitter_m:
        return [], "ball_range_unstable"
    if max(abs(angle_distance(sample.ball_bearing_rad, ball_bearing)) for sample in samples) > args.max_bearing_jitter_rad:
        return [], "ball_bearing_unstable"
    if max(abs(angle_distance(sample.target_bearing_rad, target_bearing)) for sample in samples) > args.max_bearing_jitter_rad:
        return [], "target_bearing_unstable"
    return samples, "template_samples_stable"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag-id", type=int, required=True)
    parser.add_argument("--perception-url", default="http://127.0.0.1:8080/state.json")
    parser.add_argument("--operator-confirm", default="")
    parser.add_argument("--sample-count", type=int, default=15)
    parser.add_argument("--sample-interval-s", type=float, default=0.10)
    parser.add_argument("--http-timeout-s", type=float, default=0.25)
    parser.add_argument("--perception-max-age-s", type=float, default=0.35)
    parser.add_argument("--min-ball-confidence", type=float, default=0.015)
    parser.add_argument("--stage-range-min-m", type=float, default=0.65)
    parser.add_argument("--stage-range-max-m", type=float, default=0.85)
    parser.add_argument("--max-range-jitter-m", type=float, default=0.08)
    parser.add_argument("--max-bearing-jitter-rad", type=float, default=0.10)
    parser.add_argument("--range-tolerance-m", type=float, default=0.10)
    parser.add_argument("--bearing-tolerance-rad", type=float, default=0.06)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.operator_confirm != CONFIRMATION:
        parser.error("template에는 --operator-confirm {}가 정확히 필요합니다".format(CONFIRMATION))
    if args.sample_count < 5 or args.sample_interval_s <= 0.0:
        parser.error("sample-count는 5 이상이고 sample interval은 양수여야 합니다")
    if not 0.30 <= args.stage_range_min_m < args.stage_range_max_m <= 2.0:
        parser.error("stage range를 확인하세요")
    if args.range_tolerance_m <= 0.0 or args.bearing_tolerance_rad <= 0.0:
        parser.error("template tolerance는 양수여야 합니다")

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "read_only_d435i_fr_ball_tag_camera_staging_template",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "tag_id": args.tag_id,
        "motion_commands_sent": False,
        "manual_contract": {
            "required_geometry": "FR toe swing/contact lane -> ball center -> Tag ground projection",
            "capture_pose": "same yaw/lateral offset as valid FR lane, backed 0.65-0.85m so ball+Tag are visible",
            "not_a_claim": "camera_to_base_or_FR_extrinsic",
        },
    }
    samples, reason = collect(args)
    if samples:
        desired_ball_bearing = circular_median([sample.ball_bearing_rad for sample in samples])
        desired_target_bearing = circular_median([sample.target_bearing_rad for sample in samples])
        desired_range = float(statistics.median(sample.ball_range_m for sample in samples))
        payload["result"] = {
            "template": {
                "tag_id": args.tag_id,
                "desired_ball_range_m": desired_range,
                "range_tolerance_m": args.range_tolerance_m,
                "desired_ball_bearing_rad": desired_ball_bearing,
                "desired_target_bearing_rad": desired_target_bearing,
                "ball_bearing_tolerance_rad": args.bearing_tolerance_rad,
                "target_bearing_tolerance_rad": args.bearing_tolerance_rad,
            },
            "sample_count": len(samples),
            "samples": [
                {
                    "ball_range_m": sample.ball_range_m,
                    "ball_bearing_rad": sample.ball_bearing_rad,
                    "target_bearing_rad": sample.target_bearing_rad,
                    "ball_confidence": sample.ball_confidence,
                }
                for sample in samples
            ],
            "reason": reason,
        }
        payload["verdict"] = "FR_LANE_TEMPLATE_CAPTURED"
    else:
        payload["result"] = {"reason": reason}
        payload["verdict"] = "TEMPLATE_CAPTURE_REJECTED"
    output = args.output or Path("hardware_measurements") / (
        "d435i_fr_ball_tag_camera_stage_template_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("D435I_FR_LANE_TEMPLATE_{} output={}".format(payload["verdict"], output))
    print(json.dumps(payload, ensure_ascii=False))
    return 0 if payload["verdict"] == "FR_LANE_TEMPLATE_CAPTURED" else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("INTERRUPTED: read-only template capture stopped", file=sys.stderr)
        raise SystemExit(130)
