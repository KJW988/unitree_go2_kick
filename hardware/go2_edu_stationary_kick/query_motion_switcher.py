#!/usr/bin/env python3
"""Go2 MotionSwitcher mode를 읽기 전용으로 조회한다. 모드 전환·LowCmd는 수행하지 않는다."""
import argparse
import json

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    args = parser.parse_args()
    ChannelFactoryInitialize(0, args.interface)
    client = MotionSwitcherClient()
    client.SetTimeout(3.0)
    client.Init()
    status, result = client.CheckMode()
    print(json.dumps({"status": status, "result": result}, ensure_ascii=False))


if __name__ == "__main__":
    main()
