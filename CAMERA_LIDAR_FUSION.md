# Camera–Depth/LiDAR ball fusion (검증 전용)

## 역할 분리

- YOLO nano frontend: soccer ball class의 2D bbox와 confidence만 제공한다.
- Depth frontend: bbox 중심 ray의 metric range를 제공한다.
- LiDAR: bbox에 투영되는 base-frame point의 range mode로 Depth를 cross-check하거나 Depth 부재 시 fallback range를 제공한다.
- fusion core: 검증된 camera intrinsics/extrinsics로 base-frame `ball_base_xy`, confidence, timestamp를 만든다.

`dribblebot/perception/camera_lidar_ball_fusion.py`는 ROS, YOLO, OpenCV, supervisor를 import하지 않는다. Depth–LiDAR range 차이가 0.25 m를 넘거나, LiDAR 지지점이 부족하거나, timestamp가 stale이거나, 위치 jump가 크면 observation을 거부한다.

## 현재 제한

Go2 physical front camera의 image type, camera intrinsics, camera-to-base extrinsic, Depth topic은 아직 측정·검증되지 않았다. 따라서 현재 core는 합성 테스트 전용이며 `RelativeTargetObservation`/`build_targeted_snapshot` 또는 kick pipeline에는 연결되어 있지 않다. YOLO weights와 새 Python dependency도 설치·다운로드하지 않았다.

## 검증

```bash
PYTHONPATH=. python3 scripts/test_camera_lidar_ball_fusion.py
```

합성 검증은 Depth–LiDAR 일치 위치 회복, 불일치 거부, LiDAR-only low-support 경로, jump/stale fail-close를 검사한다.
