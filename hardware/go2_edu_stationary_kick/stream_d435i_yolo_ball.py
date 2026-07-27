#!/usr/bin/env python3
"""D435i RGB-D + YOLOv5n 공 검출을 브라우저로 read-only 스트리밍한다.

YOLO가 COCO ``sports ball`` 2D 후보를 만들고, D435i의 color-aligned depth가
동일 bbox 내부의 metric range를 검증한다. Go2 LiDAR는 camera-to-base extrinsic과
시간 동기화가 아직 없으므로 이 도구에서 공 range로 거짓 결합하지 않는다. LiDAR는
후속 접근 단계의 장애물/odometry 용도로만 별도 사용한다.

이 도구는 D435i USB만 열며 Go2 DDS, LowCmd, SportClient, MotionSwitcher를 전혀
생성하지 않는다. 따라서 browser video에서 공과 Tag가 동시에 보이는지, RGB 후보와
depth range가 일치하는지 먼저 확인하는 용도다.

YOLOv5 ONNX/OpenCV DNN decoder는 Ultralytics YOLOv5 v7.0 export 형식(1x25200x85)을
따른다. 출처: https://github.com/ultralytics/yolov5/tree/v7.0
"""
from __future__ import annotations

import argparse
import html
import math
import socket
import sys
import threading
import time
from http import server
from pathlib import Path
from typing import Any, Optional


COCO_SPORTS_BALL_CLASS_ID = 32
LETTERBOX_SIZE = 640


def _require_runtime() -> tuple[Any, Any, Any]:
    try:
        import cv2
        import numpy as np
        import pyrealsense2 as rs
    except ImportError as error:
        raise RuntimeError(
            "project perception env의 cv2, numpy, pyrealsense2가 필요합니다: {}".format(error)
        ) from error
    return cv2, np, rs


class YoloV5BallDetector:
    """OpenCV DNN only YOLOv5n COCO sports-ball detector."""

    def __init__(self, cv2: Any, np: Any, model: Path, confidence: float, nms: float):
        if not model.is_file():
            raise RuntimeError("YOLO model이 없습니다: {} (fetch_yolov5n_model.py를 먼저 실행)".format(model))
        self.cv2, self.np = cv2, np
        self.net = cv2.dnn.readNetFromONNX(str(model))
        self.confidence, self.nms = float(confidence), float(nms)

    def detect(self, image_bgr: Any) -> Optional[tuple[int, int, int, int, float]]:
        height, width = image_bgr.shape[:2]
        scale = min(LETTERBOX_SIZE / width, LETTERBOX_SIZE / height)
        resized_width, resized_height = int(round(width * scale)), int(round(height * scale))
        resized = self.cv2.resize(image_bgr, (resized_width, resized_height), interpolation=self.cv2.INTER_LINEAR)
        canvas = self.np.full((LETTERBOX_SIZE, LETTERBOX_SIZE, 3), 114, dtype=self.np.uint8)
        pad_x, pad_y = (LETTERBOX_SIZE - resized_width) // 2, (LETTERBOX_SIZE - resized_height) // 2
        canvas[pad_y:pad_y + resized_height, pad_x:pad_x + resized_width] = resized
        blob = self.cv2.dnn.blobFromImage(canvas, 1.0 / 255.0, (LETTERBOX_SIZE, LETTERBOX_SIZE), swapRB=True)
        self.net.setInput(blob)
        raw = self.np.asarray(self.net.forward())
        if raw.shape != (1, 25200, 85):
            raise RuntimeError("unexpected YOLOv5n ONNX output shape: {}".format(tuple(raw.shape)))
        rows = raw[0]
        scores = rows[:, 4] * rows[:, 5 + COCO_SPORTS_BALL_CLASS_ID]
        candidate_indices = self.np.flatnonzero(scores >= self.confidence)
        boxes, confidences = [], []
        for index in candidate_indices.tolist():
            cx, cy, box_width, box_height = (float(value) for value in rows[index, :4])
            x0 = (cx - box_width * 0.5 - pad_x) / scale
            y0 = (cy - box_height * 0.5 - pad_y) / scale
            x1 = (cx + box_width * 0.5 - pad_x) / scale
            y1 = (cy + box_height * 0.5 - pad_y) / scale
            x0, x1 = max(0.0, x0), min(float(width - 1), x1)
            y0, y1 = max(0.0, y0), min(float(height - 1), y1)
            if x1 - x0 < 4.0 or y1 - y0 < 4.0:
                continue
            boxes.append([int(round(x0)), int(round(y0)), int(round(x1 - x0)), int(round(y1 - y0))])
            confidences.append(float(scores[index]))
        if not boxes:
            return None
        kept = self.cv2.dnn.NMSBoxes(boxes, confidences, self.confidence, self.nms)
        if len(kept) == 0:
            return None
        selected = int(self.np.asarray(kept).reshape(-1)[0])
        x, y, box_width, box_height = boxes[selected]
        return x, y, x + box_width, y + box_height, confidences[selected]


def _depth_range_m(np: Any, depth_raw: Any, depth_scale_m: float, intrinsics: Any,
                   detection: Optional[tuple[int, int, int, int, float]]) -> Optional[float]:
    if detection is None:
        return None
    x0, y0, x1, y1, _ = detection
    margin_x = max(2, int((x1 - x0) * 0.28))
    margin_y = max(2, int((y1 - y0) * 0.28))
    inner = depth_raw[y0 + margin_y:y1 - margin_y, x0 + margin_x:x1 - margin_x]
    if inner.size < 20:
        return None
    depth_m = inner.astype(np.float64) * depth_scale_m
    depth_m = depth_m[(depth_m >= 0.25) & (depth_m <= 3.0)]
    if len(depth_m) < 20:
        return None
    axial_depth_m = float(np.median(depth_m))
    center_x, center_y = (x0 + x1) * 0.5, (y0 + y1) * 0.5
    ray_norm = math.sqrt(
        ((center_x - float(intrinsics.ppx)) / float(intrinsics.fx)) ** 2
        + ((center_y - float(intrinsics.ppy)) / float(intrinsics.fy)) ** 2 + 1.0
    )
    return axial_depth_m * ray_norm


def _draw_tag_overlay(cv2: Any, image: Any) -> list[int]:
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "DICT_APRILTAG_36h11"):
        return []
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return []
    cv2.aruco.drawDetectedMarkers(image, corners, ids)
    return [int(value) for value in ids.reshape(-1)]


class _FrameStore:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.jpeg: Optional[bytes] = None
        self.sequence = 0

    def publish(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg, self.sequence = jpeg, self.sequence + 1
            self.condition.notify_all()

    def wait_next(self, previous: int) -> tuple[int, Optional[bytes]]:
        with self.condition:
            self.condition.wait_for(lambda: self.sequence != previous, timeout=2.0)
            return self.sequence, self.jpeg


class _Handler(server.BaseHTTPRequestHandler):
    server: Any

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            page = (
                "<!doctype html><html><head><meta charset='utf-8'><title>Go2 D435i YOLO</title>"
                "<style>body{background:#111;color:#eee;font-family:sans-serif;margin:16px}img{max-width:100%;height:auto}</style>"
                "</head><body><h3>D435i RGB-D + YOLO sports-ball (read-only)</h3>"
                "<img src='/stream.mjpg' alt='D435i stream'></body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
            return
        if self.path != "/stream.mjpg":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        previous = -1
        try:
            while True:
                previous, jpeg = self.server.frames.wait_next(previous)
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write("Content-Length: {}\r\n\r\n".format(len(jpeg)).encode("ascii"))
                self.wfile.write(jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            return


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path("hardware_models/yolov5n-v7.0.onnx"))
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument("--nms", type=float, default=0.45)
    parser.add_argument("--jpeg-quality", type=int, default=80)
    parser.add_argument("--inference-every", type=int, default=1)
    args = parser.parse_args()
    if not (1 <= args.port <= 65535 and min(args.width, args.height, args.fps, args.inference_every) > 0):
        parser.error("port/width/height/fps/inference-every must be positive")
    if not (0.0 < args.confidence <= 1.0 and 0.0 < args.nms <= 1.0 and 1 <= args.jpeg_quality <= 100):
        parser.error("confidence/nms/jpeg-quality range is invalid")

    cv2, np, rs = _require_runtime()
    detector = YoloV5BallDetector(cv2, np, args.model, args.confidence, args.nms)
    frames = _FrameStore()
    httpd = server.ThreadingHTTPServer((args.host, args.port), _Handler)
    httpd.frames = frames
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    addresses = ["http://127.0.0.1:{}".format(args.port)]
    try:
        addresses.append("http://{}:{}".format(socket.gethostbyname(socket.gethostname()), args.port))
    except OSError:
        pass
    print("D435I_YOLO_STREAM_READY urls={} model={}".format(", ".join(sorted(set(addresses))), args.model), flush=True)

    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = None
    latest_detection: Optional[tuple[int, int, int, int, float]] = None
    try:
        profile = pipeline.start(config)
        depth_scale_m = float(profile.get_device().first_depth_sensor().get_depth_scale())
        align = rs.align(rs.stream.color)
        for _ in range(20):
            align.process(pipeline.wait_for_frames(timeout_ms=3000))
        count = 0
        while True:
            aligned = align.process(pipeline.wait_for_frames(timeout_ms=3000))
            color_frame, depth_frame = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            color = np.asanyarray(color_frame.get_data())
            depth = np.asanyarray(depth_frame.get_data())
            if color.shape[:2] != depth.shape[:2]:
                continue
            count += 1
            if count % args.inference_every == 0:
                latest_detection = detector.detect(color)
            rendered = color.copy()
            tag_ids = _draw_tag_overlay(cv2, rendered)
            intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
            depth_range = _depth_range_m(np, depth, depth_scale_m, intrinsics, latest_detection)
            if latest_detection is not None:
                x0, y0, x1, y1, confidence = latest_detection
                cv2.rectangle(rendered, (x0, y0), (x1, y1), (40, 220, 40), 2)
                range_text = "depth=invalid" if depth_range is None else "range={:.2f}m".format(depth_range)
                cv2.putText(rendered, "ball {:.2f} {}".format(confidence, range_text), (x0, max(22, y0 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 220, 40), 2, cv2.LINE_AA)
            else:
                cv2.putText(rendered, "ball: not detected", (14, 28), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (40, 40, 235), 2, cv2.LINE_AA)
            cv2.putText(rendered, "AprilTag: {}".format(tag_ids if tag_ids else "none"), (14, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 210, 255), 2, cv2.LINE_AA)
            ok, encoded = cv2.imencode(".jpg", rendered, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality])
            if ok:
                frames.publish(encoded.tobytes())
    finally:
        if profile is not None:
            pipeline.stop()
        httpd.shutdown()
        httpd.server_close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("INTERRUPTED: D435i YOLO stream stopped", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print("FAILED: {}".format(error), file=sys.stderr)
        raise SystemExit(2)
