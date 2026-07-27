#!/usr/bin/env python3
"""공식 YOLO11n ONNX를 project-local hardware_models 경로로 내려받는다.

이 downloader는 robot DDS나 motion API를 사용하지 않는다. Ultralytics 공식 v8.3.0
release의 COCO pretrained ONNX artifact와 SHA-256을 고정해, 같은 파일만 deployment에
사용한다. 출처: https://github.com/ultralytics/assets/releases/tag/v8.3.0
"""
from __future__ import annotations

import argparse
import hashlib
import sys
import urllib.request
from pathlib import Path


MODEL_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx"
MODEL_SHA256 = "634279b40c07c6391472c51ad45b81ebc48706a9a1fe72dd3396322acd0c053b"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("hardware_models/yolo11n-v8.3.0.onnx"))
    args = parser.parse_args()
    output = args.output.expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.is_file() and sha256(output) == MODEL_SHA256:
        print("YOLO11N_MODEL_READY existing={} sha256={}".format(output, MODEL_SHA256))
        return 0
    partial = output.with_name(output.name + ".partial")
    try:
        with urllib.request.urlopen(MODEL_URL, timeout=60) as response, partial.open("wb") as handle:
            while True:
                block = response.read(1024 * 1024)
                if not block:
                    break
                handle.write(block)
        actual = sha256(partial)
        if actual != MODEL_SHA256:
            raise RuntimeError("SHA-256 mismatch: expected {} got {}".format(MODEL_SHA256, actual))
        partial.replace(output)
    except Exception:
        partial.unlink(missing_ok=True)
        raise
    print("YOLO11N_MODEL_READY downloaded={} sha256={}".format(output, MODEL_SHA256))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
