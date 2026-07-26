#!/usr/bin/env python3
"""LowCmd 없이 Go2 MotionSwitcher의 공식 controller alias를 확인한다."""
from __future__ import annotations

import argparse
import json
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

CONFIRMATION = "SELECT_NORMAL_MODE_READY"


def _status(value):
    """unitree_sdk2py RPC의 (code, payload) 또는 scalar code를 정규화한다."""
    return value[0] if isinstance(value, tuple) else value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--execute", action="store_true", help="없으면 CheckMode만 조회")
    parser.add_argument("--operator-confirm", help="execute에는 {} 필요".format(CONFIRMATION))
    args = parser.parse_args()
    if args.execute and args.operator_confirm != CONFIRMATION:
        parser.error("--execute requires --operator-confirm {}".format(CONFIRMATION))

    ChannelFactoryInitialize(0, args.interface)
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(3.0)
    switcher.Init()
    before_status, before = switcher.CheckMode()
    if not args.execute:
        print(json.dumps({"kind": "motion_switcher_normal_probe", "execute": False, "before": before, "before_status": before_status}, ensure_ascii=False))
        return 0

    # 출처: Unitree SDK2 Python motion_switcher_example.py. Go2 alias ``normal``은
    # official C++ Go2 stand example에서 sport_mode로 매핑된다. 이 probe는 LowCmd나
    # Sport motion 명령을 보내지 않고 mode selection 결과만 확인한다.
    select_status = _status(switcher.SelectMode("normal"))
    deadline = time.monotonic() + 2.0
    after_status, after = switcher.CheckMode()
    while time.monotonic() < deadline:
        after_status, after = switcher.CheckMode()
        if after_status == 0 and after and after.get("name"):
            break
        time.sleep(0.05)
    print(json.dumps({
        "kind": "motion_switcher_normal_probe", "execute": True,
        "before_status": before_status, "before": before,
        "select_alias": "normal", "select_status": select_status,
        "after_status": after_status, "after": after,
        "lowcmd_publisher_created": False,
    }, ensure_ascii=False))
    return 0 if select_status == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
