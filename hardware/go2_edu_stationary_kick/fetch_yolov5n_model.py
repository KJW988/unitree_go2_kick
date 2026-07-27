#!/usr/bin/env python3
"""공 검출용 공식 YOLOv5n ONNX를 project-local 경로로 내려받는다.

이 도구는 로봇 DDS/LowCmd/SportClient를 사용하지 않는다. 모델은 git에 넣지 않고
실제 PC의 ``hardware_models/`` 아래에만 둔다. URL과 SHA-256은 Ultralytics YOLOv5
v7.0 공식 release artifact 기준이다.
https://github.com/ultralytics/yolov5/releases/tag/v7.0
"""
from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path


MODEL_URL = "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx"
MODEL_SHA256 = "04f0e55c26f58d17145b36045780fe1250d5bd2187543e11568e5141d05b3262"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("hardware_models/yolov5n-v7.0.onnx"))
    args = parser.parse_args()
    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() and _sha256(output) == MODEL_SHA256:
        print("YOLOV5N_MODEL_READY existing={} sha256={}".format(output, MODEL_SHA256))
        return 0
    partial = output.with_name(output.name + ".partial")
    try:
        with urllib.request.urlopen(MODEL_URL, timeout=30) as response, partial.open("wb") as handle:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                handle.write(chunk)
        actual = _sha256(partial)
        if actual != MODEL_SHA256:
            raise RuntimeError("SHA-256 mismatch: expected {} got {}".format(MODEL_SHA256, actual))
        partial.replace(output)
    except Exception:
        partial.unlink(missing_ok=True)
        raise
    print("YOLOV5N_MODEL_READY downloaded={} sha256={}".format(output, MODEL_SHA256))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
