#!/usr/bin/env python3
"""Go2 front video message 한 frame의 압축 포맷 metadata만 read-only로 확인한다."""

import os
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_SETUP = Path.home() / "Desktop/Jiwon/go2_lidar_ros2_ws/unitree_ros2/cyclonedds_ws/install/setup.bash"


def _bootstrap() -> None:
    setup = Path(os.environ.get("GO2_CAMERA_PROBE_SETUP", str(DEFAULT_SETUP))).expanduser()
    if not setup.is_file():
        raise RuntimeError(f"Go2 DDS setup을 찾지 못했습니다: {setup}")
    env = {
        "HOME": os.environ.get("HOME", ""), "USER": os.environ.get("USER", "unitree"),
        "TERM": os.environ.get("TERM", "xterm-256color"), "LANG": os.environ.get("LANG", "C.UTF-8"),
        "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
        "GO2_CAMERA_FRAME_PROBE": str(REPO_ROOT / "camera_frame_probe.py"),
        "GO2_CAMERA_PROBE_SETUP": str(setup),
        "GO2_CAMERA_PROBE_INTERFACE": os.environ.get("GO2_CAMERA_PROBE_INTERFACE", "eth0"),
    }
    command = "\n".join((
        "source /opt/ros/foxy/setup.bash", 'source "$GO2_CAMERA_PROBE_SETUP"',
        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp", "export ROS_LOCALHOST_ONLY=0",
        "export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>'\"$GO2_CAMERA_PROBE_INTERFACE\"'</NetworkInterfaceAddress></General></Domain></CycloneDDS>'",
        'exec /usr/bin/python3 "$GO2_CAMERA_FRAME_PROBE" --ros-ready',
    ))
    os.execvpe("/bin/bash", ["bash", "--noprofile", "--norc", "-c", command], env)


def _codec_hint(data: bytes) -> str:
    if data.startswith(b"\xff\xd8\xff"):
        return "jpeg"
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        return "png"
    if data.startswith(b"\x00\x00\x00\x01") or data.startswith(b"\x00\x00\x01"):
        return "annex-b-h264-or-h265"
    return "unknown"


def _probe() -> int:
    import rclpy
    from unitree_go.msg import Go2FrontVideoData

    received = []
    rclpy.init()
    node = rclpy.create_node("go2_front_video_metadata_probe")
    def callback(message):
        if not received:
            received.append(message)
    subscription = node.create_subscription(Go2FrontVideoData, "/frontvideostream", callback, 1)
    deadline = time.monotonic() + 10.0
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    try:
        if not received:
            raise RuntimeError("10초 안에 /frontvideostream frame을 받지 못했습니다")
        message = received[0]
        print(f"time_frame={message.time_frame}")
        for name in ("video720p", "video360p", "video180p"):
            data = bytes(getattr(message, name))
            print(f"{name}: bytes={len(data)} codec_hint={_codec_hint(data)} magic_hex={data[:16].hex()}")
        print("No frame bytes were saved; this probe only printed metadata.")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    if len(sys.argv) == 1:
        _bootstrap()
    if sys.argv[1:] == ["--ros-ready"]:
        return _probe()
    raise SystemExit("실행은 `python3 camera_frame_probe.py`입니다.")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"camera frame probe 실패: {error}", file=sys.stderr)
        raise SystemExit(2)
