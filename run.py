#!/usr/bin/env python3
"""Go2 LiDAR ball/empty rosbag을 한 번에 검증하는 sensor-only runner.

일반 실행: `python3 run.py`
저장된 bag을 localhost DDS에서만 재생하고 `/utlidar/cloud_base`만 구독한다.
robot-control publisher/service/action, policy, motor API는 만들지 않는다.
"""

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, Tuple

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_BAG_ROOT = Path.home() / "Desktop/Jiwon/lidar_bags"
DEFAULT_SETUP = (
    Path.home() / "Desktop/Jiwon/go2_lidar_ros2_ws/unitree_ros2/"
    "cyclonedds_ws/install/setup.bash"
)
CLOUD_TOPIC = "/utlidar/cloud_deskewed"
ODOM_TOPIC = "/utlidar/robot_odom"
# Jetson CPU detector가 15.4 Hz playback을 놓치지 않도록 저장 bag만 감속한다.
PLAYBACK_RATE = 0.25
PLAYBACK_TIMEOUT_S = 300.0
BAGS: Dict[str, Tuple[str, int, str]] = {
    "ball": ("go2_static_ball_1m_20260726_215950", 603, "ball_1m_deskewed_analysis.json"),
    "empty": ("go2_static_empty_20260726_220040", 804, "empty_deskewed_analysis.json"),
}


def _enter_isolated_ros_environment() -> None:
    """Foxy/DDS 환경만 준비한 뒤 system Python으로 이 파일을 재실행한다."""

    setup = Path(os.environ.get("GO2_LIDAR_SETUP", str(DEFAULT_SETUP))).expanduser()
    if not setup.is_file():
        raise RuntimeError(
            "Go2 DDS setup을 찾지 못했습니다. 다음 경로를 확인하거나 GO2_LIDAR_SETUP을 설정하세요: "
            f"{setup}"
        )
    # Noetic/Conda의 ROS/library 변수를 물려받지 않는 최소 shell 환경이다.
    environment = {
        "HOME": os.environ.get("HOME", ""),
        "USER": os.environ.get("USER", "unitree"),
        "TERM": os.environ.get("TERM", "xterm-256color"),
        "LANG": os.environ.get("LANG", "C.UTF-8"),
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "GO2_LIDAR_RUNNER": str(REPO_ROOT / "run.py"),
        "GO2_LIDAR_SETUP": str(setup),
    }
    # Bag playback/analysis를 loopback으로 고정: 실제 로봇 DDS를 보지 않는다.
    command = "\n".join((
        "source /opt/ros/foxy/setup.bash",
        'source "$GO2_LIDAR_SETUP"',
        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        "export ROS_LOCALHOST_ONLY=0",
        "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'",
        'exec /usr/bin/python3 "$GO2_LIDAR_RUNNER" --ros-ready',
    ))
    os.execvpe(
        "/bin/bash",
        ["bash", "--noprofile", "--norc", "-c", command],
        environment,
    )


def _run_one_bag(label: str, bag_root: Path) -> Dict[str, object]:
    import rclpy
    from scripts.analyze_lidar_deskewed_topic_ros2 import DeskewedTopicAnalyzer

    bag_name, expected_frames, output_name = BAGS[label]
    bag_path = bag_root / bag_name
    if not bag_path.is_dir():
        raise FileNotFoundError(f"{label} bag을 찾지 못했습니다: {bag_path}")

    node = rclpy.create_node(f"validated_lidar_offline_{label}")
    analyzer = DeskewedTopicAnalyzer(node, CLOUD_TOPIC, ODOM_TOPIC)
    playback = None
    try:
        # DDS discovery를 끝낸 뒤 저장 bag만 localhost에 재생한다.
        time.sleep(1.5)
        playback = subprocess.Popen(
            [
                "ros2", "bag", "play", str(bag_path), "--topics", CLOUD_TOPIC, ODOM_TOPIC,
                "--rate", str(PLAYBACK_RATE),
            ],
            env=os.environ.copy(),
        )
        deadline = time.monotonic() + PLAYBACK_TIMEOUT_S
        while playback.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.20)
        if playback.poll() is None:
            playback.terminate()
            playback.wait(timeout=5.0)
            timed_out = True
        else:
            timed_out = False
        # playback 종료 직전 DDS queue에 들어온 마지막 point cloud를 처리한다.
        quiet_deadline = time.monotonic() + 1.0
        while time.monotonic() < quiet_deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
        result = analyzer.result(CLOUD_TOPIC)
        result.update({
            "bag": str(bag_path),
            "expected_frames": expected_frames,
            "complete": len(analyzer.stamps) == expected_frames,
            "timed_out": timed_out,
            "playback_returncode": playback.returncode,
        })
        output = bag_root / output_name
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("\n[{}] {}".format(label, json.dumps(
            {key: value for key, value in result.items() if key != "detection_samples"},
            ensure_ascii=False,
        )))
        return result
    finally:
        if playback is not None and playback.poll() is None:
            playback.terminate()
        node.destroy_node()


def _run_ros_ready(argv: Iterable[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", type=Path, default=Path(
        os.environ.get("GO2_LIDAR_BAG_ROOT", str(DEFAULT_BAG_ROOT))
    ))
    parser.add_argument("--only", choices=("ball", "empty", "both"), default="both")
    args = parser.parse_args(list(argv))
    labels = ("ball", "empty") if args.only == "both" else (args.only,)

    import rclpy

    rclpy.init()
    try:
        results = [_run_one_bag(label, args.bag_root.expanduser()) for label in labels]
    finally:
        rclpy.shutdown()
    if any(not result["complete"] or result["timed_out"] for result in results):
        print("\n분석이 완전하지 않습니다. 생성된 JSON을 그대로 보내주세요.", file=sys.stderr)
        return 2
    print("\n완료: JSON 두 개를 보내주세요. 성공 판정 전에는 kick input에 연결하지 않습니다.")
    return 0


def main() -> int:
    if len(sys.argv) == 1:
        _enter_isolated_ros_environment()
    if sys.argv[1:] == ["--ros-ready"]:
        return _run_ros_ready(())
    if sys.argv[1] == "--ros-ready":
        return _run_ros_ready(sys.argv[2:])
    raise SystemExit("실행은 `python3 run.py`입니다.")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"LiDAR offline validation 실패: {error}", file=sys.stderr)
        raise SystemExit(2)
