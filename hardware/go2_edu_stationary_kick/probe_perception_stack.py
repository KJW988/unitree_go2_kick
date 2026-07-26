#!/usr/bin/env python3
"""D435i·Go2 LiDAR perception transport의 read-only inventory를 기록한다.

이 probe는 USB/ROS2 graph만 조회한다. DDS publisher, SportClient,
MotionSwitcher, LowCmd를 만들거나 robot service를 호출하지 않는다.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence


def _run(argv: Sequence[str], timeout_s: float = 5.0) -> dict[str, Any]:
    """외부 진단 명령의 결과를 기록하되, 없는 도구는 실패 원인으로만 남긴다."""
    if shutil.which(argv[0]) is None:
        return {"argv": list(argv), "available": False, "returncode": None, "stdout": "", "stderr": "not found"}
    try:
        completed = subprocess.run(
            list(argv), check=False, text=True, capture_output=True, timeout=timeout_s
        )
    except subprocess.TimeoutExpired:
        return {"argv": list(argv), "available": True, "returncode": None, "stdout": "", "stderr": "timeout"}
    return {
        "argv": list(argv),
        "available": True,
        "returncode": completed.returncode,
        "stdout": completed.stdout,
        "stderr": completed.stderr,
    }


def _ros_topics() -> dict[str, Any]:
    result = _run(("ros2", "topic", "list", "-t"), timeout_s=8.0)
    candidates: list[dict[str, str]] = []
    keywords = ("camera", "image", "depth", "point", "cloud", "lidar", "utlidar", "odom", "tf")
    if result["returncode"] == 0:
        for raw_line in result["stdout"].splitlines():
            line = raw_line.strip()
            if not line or " [" not in line or not line.endswith("]"):
                continue
            topic, type_name = line.rsplit(" [", 1)
            if any(keyword in topic.lower() for keyword in keywords):
                candidates.append({"topic": topic, "type": type_name[:-1]})
    return {"command": result, "candidate_topics": candidates}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=None, help="기본값: hardware_measurements/perception_stack_probe_*.json")
    args = parser.parse_args()

    payload: dict[str, Any] = {
        "schema_version": 1,
        "kind": "read_only_go2_d435i_lidar_transport_probe",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "environment": {
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        },
        "system": {
            "uname": _run(("uname", "-a")),
            "usb": _run(("lsusb",)),
            "video": _run(("v4l2-ctl", "--list-devices")),
            "realsense": _run(("rs-enumerate-devices", "-s")),
        },
        "ros2": _ros_topics(),
    }
    output = args.output or Path("hardware_measurements") / (
        "perception_stack_probe_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("PERCEPTION_PROBE_OK output={}".format(output))
    print(json.dumps({"ros2_candidate_topics": payload["ros2"]["candidate_topics"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
