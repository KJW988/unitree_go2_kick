# Go2 EDU 내장 LiDAR: 센서 전용 검증 runbook

## 범위와 안전 경계

- 이 문서는 정지 로봇과 확보된 E-stop을 전제로 한다.
- 실행자는 point cloud/IMU/odometry를 구독·기록만 한다. `/lowcmd`,
  `/api/sport/request`, stand/walk/kick/policy 예제는 실행하지 않는다.
- `dribblebot/perception/lidar_ball_detector.py`는 실공 검증 이력이 없는
  참고 코드다. 이 문서의 새 detector와 검증 결과가 통과하기 전에는 킥 입력으로
  사용하지 않는다.

## 실측 bring-up 결과 (2026-07-26)

Go2 EDU의 DDS를 개인 Foxy/CycloneDDS workspace에서 `eth0`으로 수신했다.

- `/utlidar/cloud`, `/utlidar/cloud_base`, `/utlidar/cloud_deskewed`를 발견했다.
- `/utlidar/cloud_base`는 `base_link` frame이고 약 15.4 Hz다. 이 토픽만이
  base-frame ball 위치 후보의 기본 입력이다.
- `/utlidar/cloud_deskewed`는 `odom` frame이므로 base-frame detector 입력이
  아니다. bag cross-check용으로만 기록한다.
- ball 39.155 s bag에서 `cloud/cloud_base/cloud_deskewed`는 각각 603개,
  empty 52.264 s bag에서는 각각 805/805/804개였다.

## 개인 ROS2 workspace

공용 장비의 ROS/Conda와 분리하기 위해 사용자 workspace에 Foxy용
`rmw_cyclonedds`와 CycloneDDS 0.10.x를 빌드한다. 기존 `~/.bashrc`,
`~/unitree_ros2`, system package는 변경하지 않는다. `CYCLONEDDS_URI`에는
Go2 Ethernet IP 대역의 host NIC만 지정한다.

`NetworkInterfaceAddress` compatibility config는 현재 동작하지만 deprecated다.
bag 검증이 완료된 뒤에만 개인 XML을 `Interfaces/NetworkInterface name="eth0"`
형식으로 전환한다.

## 검증 순서

1. `scripts/test_validated_lidar_ball_detector.py`를 `go2kick` 환경에서 실행한다.
   합성 지면 + 11 cm 구 + noise/clutter에서 ground removal, clustering, sphere fit,
   base-frame 제약, stale fail-closed를 검증한다.
2. 정지 ball/empty rosbag 각각에
   `scripts/analyze_lidar_ball_bag_ros2.py`를 실행한다. 이 script는
   `/utlidar/cloud_base`만 deserialize하며 control API를 만들지 않는다.
3. ball bag detection recall, empty bag false positive, radius/residual/range와
   timestamp drop을 비교한다. 실제 kick lane의 p95 XY 오차 2 cm 및 충분한
   연속 high-confidence 관측을 별도로 입증하기 전에는
   `RelativeTargetObservation.ball_base_xy`로 연결하지 않는다.

## off-line bag 명령

개인 DDS workspace를 source한 Foxy shell에서 실행한다.

```bash
python3 scripts/analyze_lidar_ball_bag_ros2.py \
  "$HOME/Desktop/Jiwon/lidar_bags/go2_static_ball_1m_20260726_215950" \
  --output "$HOME/Desktop/Jiwon/lidar_bags/ball_1m_analysis.json"

python3 scripts/analyze_lidar_ball_bag_ros2.py \
  "$HOME/Desktop/Jiwon/lidar_bags/go2_static_empty_20260726_220040" \
  --output "$HOME/Desktop/Jiwon/lidar_bags/empty_analysis.json"
```

두 JSON은 후보 통계일 뿐이며, 이 단계에서 supervisor/locomotion/kick 경로는
어떤 값도 받지 않는다.
