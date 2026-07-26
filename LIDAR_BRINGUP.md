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
- `/utlidar/cloud_deskewed`는 `odom` frame이다. sparse `cloud_base`가 검증에
  실패했으므로, 정지 bag에서만 `/utlidar/robot_odom` pose로 명시적으로 `base_link`로
  역변환한 dense 검증 경로를 사용한다. 변환 오차·false positive 검증 전에는 runtime 입력이 아니다.
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
2. 정지 ball/empty rosbag 각각을 loopback DDS에서 재생하고
   `scripts/analyze_lidar_ball_topic_ros2.py`로 구독한다. 이 경로는 Foxy에
   `rosbag2_py` Python binding이 없어도 되며, `/utlidar/cloud_base`만 구독하고
   control API를 만들지 않는다.
3. ball bag detection recall, empty bag false positive, radius/residual/range와
   timestamp drop을 비교한다. 실제 kick lane의 p95 XY 오차 2 cm 및 충분한
   연속 high-confidence 관측을 별도로 입증하기 전에는
   `RelativeTargetObservation.ball_base_xy`로 연결하지 않는다.

## off-line bag 명령

Foxy 설치에 `rosbag2_py`가 없으면, 아래 두 터미널 방식으로 bag을 loopback에서만 재생·분석한다. 두 터미널은 반드시 `ROS_LOCALHOST_ONLY=0`과 loopback `lo` 전용 `CYCLONEDDS_URI`를 적용한다. 따라서 재생기와 분석기만 서로 발견하며 실제 Go2 DDS에는 연결되지 않는다.

두 터미널 공통:

```bash
env -i HOME="$HOME" USER="$USER" TERM="$TERM" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash --noprofile --norc
source /opt/ros/foxy/setup.bash
source "$HOME/Desktop/Jiwon/go2_lidar_ros2_ws/unitree_ros2/cyclonedds_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>lo</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
cd "$HOME/Desktop/Jiwon/soccer/unitree_go2_kick"
```

터미널 A (먼저 시작):

```bash
python3 scripts/analyze_lidar_ball_topic_ros2.py \
  --topic /utlidar/cloud_base --max-messages 603 --timeout-s 90 \
  --output "$HOME/Desktop/Jiwon/lidar_bags/ball_1m_analysis.json"
```

터미널 B (그 다음, 저장 bag만 재생):

```bash
ros2 bag play "$HOME/Desktop/Jiwon/lidar_bags/go2_static_ball_1m_20260726_215950" \
  --topics /utlidar/cloud_base
```

empty bag은 `--max-messages 805`, bag 경로와 JSON 이름을 `empty`로 바꿔 같은 순서로 실행한다.

`rosbag2_py`가 설치된 별도 환경에서만 아래 직접 reader를 사용한다.

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

## 가장 쉬운 실행 (권장)

저장된 ball/empty bag이 이미 있는 Jetson의 repo root에서 다음 한 줄만 실행한다.

```bash
python3 run.py
```

`run.py`는 Foxy와 개인 DDS workspace를 자동으로 source한 뒤, `NetworkInterfaceAddress=lo`로 고정된 localhost에서 bag을 순서대로 재생·분석한다. 실제 Go2 DDS, motion, motor, policy에는
접근하지 않는다. Jetson CPU에서 모든 frame을 처리하기 위해 재생 속도를 0.25×로 낮추므로 전체 실행에는 약 6분이 걸린다. 결과는 `~/Desktop/Jiwon/lidar_bags/ball_1m_analysis.json` 및
`empty_analysis.json`에 저장된다. 경로가 다르면 `GO2_LIDAR_SETUP` 또는
`GO2_LIDAR_BAG_ROOT` 환경변수로만 바꿀 수 있다.

`run.py`가 생성하는 JSON에는 point field와 sampled ROI/ground/cluster/sphere-fit 단계 통계도 포함된다. 실공 미검출 시 이 통계를 사용해 탈락 단계를 재설계하며, 검증 전에는 킥 입력으로 연결하지 않는다.

## 실공 bag 재설계 상태

단일-frame sphere-fit은 완전한 ball/empty bag에서 ball recall 0/603, empty false positive 0/805로 실패했다. 정지 로봇에서는 65 ms 단위의 희소 반사점을 12 frame non-overlapping base-frame window로 누적하고, self-leg 영역을 제외한 1 m 전방 lane에서만 sphere-fit하는 validation profile을 사용한다. 이는 다음 bag 결과로 recall·false positive·위치 오차를 다시 판정하기 위한 실험용 설정이며, runtime ball search나 kick interface에는 아직 연결하지 않는다.

## Dense deskewed bag 검증

`run.py`의 다음 단계는 `/utlidar/cloud_deskewed`와 `/utlidar/robot_odom`만 재생한다. cloud가 `odom` frame이므로 최신 timestamp의 odometry pose로 `base_link`로 역변환하고, pose 차이가 50 ms를 넘는 frame은 버린다. 이는 저장된 정지 bag의 실험이며 실제 DDS·제어 경로와 연결되지 않는다. 결과 파일은 `ball_1m_deskewed_analysis.json`과 `empty_deskewed_analysis.json`이다.
