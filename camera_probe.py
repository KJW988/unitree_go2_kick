#!/usr/bin/env python3
"""정지 Go2의 RGB/Depth/CameraInfo 후보 topic type만 read-only로 조사한다.

일반 실행: `python3 camera_probe.py`
이 스크립트는 ROS2 graph metadata만 읽으며 image subscribe/record, control publish,
service/action 호출을 하지 않는다.
"""

import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import List

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_SETUP = (
    Path.home() / "Desktop/Jiwon/go2_lidar_ros2_ws/unitree_ros2/"
    "cyclonedds_ws/install/setup.bash"
)
KEYWORDS = ("camera", "image", "video", "depth", "rgb", "stream", "front")


def _enter_sensor_only_ros_environment() -> None:
    setup = Path(os.environ.get("GO2_CAMERA_PROBE_SETUP", str(DEFAULT_SETUP))).expanduser()
    if not setup.is_file():
        raise RuntimeError(f"Go2 DDS setup을 찾지 못했습니다: {setup}")
    interface = os.environ.get("GO2_CAMERA_PROBE_INTERFACE", "eth0")
    environment = {
        "HOME": os.environ.get("HOME", ""),
        "USER": os.environ.get("USER", "unitree"),
        "TERM": os.environ.get("TERM", "xterm-256color"),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "GO2_CAMERA_PROBE_RUNNER": str(REPO_ROOT / "camera_probe.py"),
        "GO2_CAMERA_PROBE_SETUP": str(setup),
        "GO2_CAMERA_PROBE_INTERFACE": interface,
    }
    # eth0 DDS graph을 읽기 위한 config다. 어떤 publish/control 명령도 실행하지 않는다.
    command = "\n".join((
        "source /opt/ros/foxy/setup.bash",
        'source "$GO2_CAMERA_PROBE_SETUP"',
        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        "export ROS_LOCALHOST_ONLY=0",
        "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>'\"$GO2_CAMERA_PROBE_INTERFACE\"'</NetworkInterfaceAddress></General></Domain></CycloneDDS>'",
        'exec /usr/bin/python3 "$GO2_CAMERA_PROBE_RUNNER" --ros-ready',
    ))
    os.execvpe("/bin/bash", ["bash", "--noprofile", "--norc", "-c", command], environment)


def _run(command: List[str], timeout_s: float = 12.0) -> str:
    completed = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout_s, check=False)
    return completed.stdout.rstrip()


def _probe() -> int:
    topics_text = _run(["ros2", "topic", "list", "--no-daemon"])
    topics = sorted(line.strip() for line in topics_text.splitlines() if line.strip().startswith("/"))
    candidates = [topic for topic in topics if any(word in topic.lower() for word in KEYWORDS)]
    lines = [
        "Go2 camera/depth sensor-only probe",
        f"timestamp_utc={datetime.now(timezone.utc).isoformat()}",
        f"candidate_count={len(candidates)}",
        "",
    ]
    for topic in candidates:
        lines.extend((f"## {topic}", _run(["ros2", "topic", "info", "-v", topic]), ""))
    # Unitree front video는 custom message라 field/codec 단서를 별도로 기록한다.
    if "/frontvideostream" in candidates:
        lines.extend(("## interface unitree_go/msg/Go2FrontVideoData",
                      _run(["ros2", "interface", "show", "unitree_go/msg/Go2FrontVideoData"]), ""))
    if not candidates:
        lines.append("No camera/depth-like topics found. Full topic list follows:")
        lines.append(topics_text)
    report = "\n".join(lines) + "\n"
    output = REPO_ROOT / "camera_probe_report.txt"
    output.write_text(report, encoding="utf-8")
    print(report, end="")
    print(f"Saved read-only report: {output}")
    return 0


def main() -> int:
    if len(sys.argv) == 1:
        _enter_sensor_only_ros_environment()
    if sys.argv[1:] == ["--ros-ready"]:
        return _probe()
    raise SystemExit("실행은 `python3 camera_probe.py`입니다.")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"camera probe 실패: {error}", file=sys.stderr)
        raise SystemExit(2)
