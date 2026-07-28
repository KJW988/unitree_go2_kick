# Go2 MCF data-channel preflight

실제 Go2의 Service Status에서 `mcf`, `webrtc_bridge`, `webrtc_signal_server`가
Functional이고 legacy `sport_mode`가 Close인 firmware에서는 `SportClient.Move()`의
return code만으로 실제 보행을 판단하면 안 된다. `Move()`는 no-reply 호출이며, closed
legacy service가 MCF gait로 command를 채택한다는 보장은 없다.

이 프로젝트에서 확인한 실제 robot firmware는 Go2 hardware v2.0 / software 1.1.11,
`mcf / 1.0.0.75`, `webrtc_bridge / 1.3.0.4`다. MCF는 firmware 1.1.7부터 쓰이는
main motion service다. 이 상태에서 `sport_mode`, `ai_sport`, `advanced_sport`를 App이나
`RobotStateClient.ServiceSwitch()`로 임의 활성화하면 parallel motion controller가 될 수
있으므로, 이 demo의 보행 해결책으로 사용하지 않는다.

## Transport 선택

Unitree App 연결 경로와 같은 WebRTC data channel을 사용한다. 이 선택은
[`unitree_webrtc_connect` v2.1.2](https://github.com/legion1581/unitree_webrtc_connect)의
Go2 MCF 예제를 참조했다. 그 예제의 `rt/api/sport/request`, MCF `GetState` API 1034와
`rt/lf/sportmodestate` 구독 형식만 먼저 채택한다. 이 repository의 probe에는 `Move`,
wireless-controller publish, obstacle toggle을 넣지 않는다.

Go2 1.1.11은 해당 driver가 AES-128 per-device key가 필요하다고 명시한 Go2 1.1.15+
이전 firmware다. 따라서 첫 local connection에는 Unitree App password, cloud token,
AES key를 사용하거나 기록하지 않는다.

## 격리 environment

기존 `.conda-unitree-sdk-py311`에는 `cyclonedds`와 `unitree_sdk2py`가 이미 고정되어
있다. WebRTC transport dependency를 섞지 않는다. 실제 robot PC에서 아래 environment를
**한 번만** 만든다.

```bash
cd ~/Desktop/Jiwon/soccer/unitree_go2_kick
source ~/miniconda3/etc/profile.d/conda.sh
conda create -y --prefix "$PWD/.conda-go2-mcf-webrtc-py311" python=3.11 pip
conda activate "$PWD/.conda-go2-mcf-webrtc-py311"
python -m pip install "unitree-webrtc-connect==2.1.2"
```

이 install은 project-local environment만 변경한다. Unitree SDK, ROS, D435i, 기존
kick environment는 변경하지 않는다.

## 첫 검증은 read-only

`ROBOT_WIFI_IP`는 Unitree App이 실제로 연결하는 robot의 Wi-Fi LAN IP다. 현재 robot PC의
`wlan0` IP와 혼동하지 말고 App device details에서 확인한다. 이 probe는 WebRTC connection,
MCF `GetState`, low-frequency state subscription만 수행한다.

```bash
cd ~/Desktop/Jiwon/soccer/unitree_go2_kick
source ~/miniconda3/etc/profile.d/conda.sh
conda activate "$PWD/.conda-go2-mcf-webrtc-py311"

python hardware/go2_edu_stationary_kick/probe_go2_mcf_webrtc.py \
  --robot-ip ROBOT_WIFI_IP
```

성공 조건은 `MCF_WEBRTC_PROBE_OK`, `mcf_get_state.status_code=0`, 그리고 가능하면
`lf_sportmodestate_count >= 1`이다. 이 검증이 통과하기 전에는 보행/joystick publisher를
구현하거나 실행하지 않는다.

## 이후 순서

1. MCF read-only probe 통과
2. empty-floor에서 data-channel `Move` 1회만 사용하는 별도 gait proof
3. physical remote preempt가 gait proof 중에도 우선하는지 검증
4. 그 다음에만 D435i ball/AprilTag planner를 MCF walker에 연결
5. FR LowCmd kick은 별도 one-shot 실험으로 유지하며 자동 handback은 연결하지 않음

## Minimal forward gait proof

실제 robot에서 read-only probe가 `192.168.123.161`으로 통과했다. 이 IP는 robot PC의
`192.168.123.18`이나 PC WLAN IP와 다르며 Go2 MCU WebRTC endpoint다. 다음 script는
**기본값으로 read-only preflight만** 한다. preflight는 `mode/progress/body height/velocity`를
확인하고, 운동 command를 전송하지 않는다.

```bash
python hardware/go2_edu_stationary_kick/prove_go2_mcf_webrtc_walk.py \
  --robot-ip 192.168.123.161
```

하네스가 아닌 실제 바닥에서는 robot 전방 1 m 이상을 완전히 비우고, physical remote/E-stop을
손에 든 operator가 robot을 계속 볼 때만 아래 실행을 허용한다. 정확히 `0.05 m/s`, 10 Hz,
1.0초만 전진 command를 보내며 바로 MCF `StopMove` acknowledgement를 기다린다. legacy
`SportClient`, MotionSwitcher, LowCmd 및 obstacle setting은 건드리지 않는다.

```bash
python hardware/go2_edu_stationary_kick/prove_go2_mcf_webrtc_walk.py \
  --robot-ip 192.168.123.161 \
  --execute \
  --operator-confirm MCF_EMPTY_FLOOR_ESTOP_READY
```

성공 판정은 log의 `COMMAND_AND_STOP_ACKED`와 operator의 실제 전진 관찰을 **둘 다** 만족하는
것이다. SDK return code/telemetry만으로 physical gait 성공이라고 판단하지 않는다.

### `Move(1008)` rejection pattern과 joystick fallback

이 robot에서는 MCF `Move(1008)`와 `StopMove(1003)` 모두 acknowledgement를 반환하고
`lf_sportmodestate.velocity`도 변했지만 physical gait가 관찰되지 않았다. 이 값은 실제 body
translation의 증거가 아니므로 `COMMAND_AND_STOP_ACKED`만으로 success를 선언하지 않는다.

그 WebRTC driver의 Go2 obstacle-avoid example은 보행을 `Move(1008)`이 아니라 App-equivalent
`rt/wirelesscontroller` joystick payload로 수행한다. 이 repository는 **직접 DDS publisher를
만들지 않고**, WebRTC bridge에서만 이 경로를 쓴다. LiDAR obstacle avoidance는 절대 끄거나
변경하지 않는다.

```bash
# 먼저 운동 command 없는 preflight
python hardware/go2_edu_stationary_kick/prove_go2_mcf_webrtc_joystick.py \
  --robot-ip 192.168.123.161

# empty floor, physical remote/E-stop 준비 후에만 실제 burst
python hardware/go2_edu_stationary_kick/prove_go2_mcf_webrtc_joystick.py \
  --robot-ip 192.168.123.161 \
  --execute \
  --operator-confirm MCF_JOYSTICK_EMPTY_FLOOR_ESTOP_READY
```

실행 범위는 `ly=0.20`, 50 Hz, 0.40초이고 즉시 neutral joystick 3회를 보낸다. operator의
물리적 이동 관찰이 유일한 gait 성공 판정이다.

## LiDAR odometry bounded calibration

짧은 0.40초 burst는 gait initiation 위상에 따라 한 발이 나가지 않을 수 있다. 이 때문에
approach 제어에는 고정 시간을 쓰지 않는다. `calibrate_go2_mcf_websocket_joystick_odom.py`는
WebRTC의 `rt/utlidar/robot_pose`를 먼저 1초간 구독해 static baseline을 만들고, initial yaw
방향 progress가 0.20 m에 도달하면 neutral joystick으로 정지한다. hard maximum은 2.0초다.

```bash
# LiDAR pose shape 및 static baseline만 확인: 운동 command 없음
python hardware/go2_edu_stationary_kick/calibrate_go2_mcf_websocket_joystick_odom.py \
  --robot-ip 192.168.123.161

# 전방 1 m 이상 빈 바닥과 physical remote/E-stop을 준비했을 때만 실행
python hardware/go2_edu_stationary_kick/calibrate_go2_mcf_websocket_joystick_odom.py \
  --robot-ip 192.168.123.161 \
  --execute \
  --operator-confirm MCF_ODOM_CALIBRATION_EMPTY_FLOOR_ESTOP_READY
```

`TARGET_REACHED_NEUTRALIZED` 또는 `MAX_DURATION_NEUTRALIZED`는 control command가 neutral로
끝났다는 뜻일 뿐이다. 기록된 `measured_forward_m`와 operator의 실제 측정값을 비교해야
calibration success다. static LiDAR drift는 수 cm 가능하므로 이 단계는 target-kick alignment
판정이 아니라 safe locomotion scale 확인에만 사용한다.

## Physical remote direct-DDS watchdog

실측 결과 이 firmware의 WebRTC data-channel subscriber는 physical remote의 실제
non-neutral input을 전달하지 않았다. 따라서 `probe_go2_mcf_webrtc_remote_preempt.py`의
`NO_PHYSICAL_INPUT_OBSERVED`는 remote 고장이 아니라 transport 한계이며, stage execute의
evidence로 사용하지 않는다.

대신 `watch_go2_physical_remote_dds.py`가 이미 검증된 SDK direct DDS
`rt/wirelesscontroller`를 read-only로 구독한다. watcher는 status JSON heartbeat와 localhost
UDP event만 만들며 publisher, LowCmd, MotionSwitcher, Sport API를 사용하지 않는다.

```bash
cd ~/Desktop/Jiwon/soccer/unitree_go2_kick
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ~/Desktop/Jiwon/soccer/unitree_go2_kick/.conda-unitree-sdk-py311
unset PYTHONPATH
python hardware/go2_edu_stationary_kick/watch_go2_physical_remote_dds.py \
  --interface eth0
```

`DIRECT_DDS_REMOTE_WATCHDOG_READY` 뒤 empty floor/E-stop 상태에서 physical remote stick을
짧게 입력하고 중립으로 돌린다. `hardware_measurements/go2_direct_remote_watchdog.json`의
`physical_input_event_count >= 1`, fresh `heartbeat_monotonic_s`, 그리고 0.6초보다 오래된
`last_active_monotonic_s`가 execute의 필수 조건이다. 실행 중 새 direct DDS input이 오면 stage는
다음 50 Hz virtual packet 전에 neutralize한다. 이것은 firmware-level controller arbitration이
아닌 user-space fail-closed guard이므로 physical remote/E-stop은 계속 operator의 1차 안전 수단이다.

## D435i ball/Tag camera staging (FR kick 전용 아님)

`approach_ball_to_tag.py`는 legacy `SportClient/ObstaclesAvoidClient.Move` 경로를 사용하며
이 firmware에서는 acknowledgement만 있고 실제 gait가 없었다. **실물에서 실행하지 않는다.**
대신 `stage_go2_mcf_ball_tag_webrtc.py`는 이미 physical gait가 검증된 WebRTC
`rt/wirelesscontroller`만 사용한다.

이 stage는 D435i `state.json`에서 3개의 안정 frame에 대해 ball depth/YOLO confidence,
ball bearing, Tag 지면투영 target bearing을 확인하고, LiDAR odometry static baseline과
0.35 m travel hard limit을 통과할 때만 동작한다. continuous drive가 아니라 0.20 magnitude의
짧은 yaw/forward pulse 뒤 neutral 3회와 재관측을 반복한다. 기본 staging depth는 0.65–0.85 m다.
따라서 공과 Tag가 camera에서 보이는 정렬 준비까지만 수행하며, camera→base→FR transform이
없는 현재 상태에서는 FR foot lane/LowCmd kick을 호출하지 않는다.

먼저 D435i stream을 별도 perception terminal에서 유지한다. 다음 command는 motion command를
보내지 않는 dry-run이며 `DRY_RUN_READY`와 다음 pulse reason만 출력한다.

```bash
python hardware/go2_edu_stationary_kick/stage_go2_mcf_ball_tag_webrtc.py \
  --robot-ip 192.168.123.161 --tag-id 11
```

dry-run이 통과하고, 전방 1 m 이상 clear floor, physical remote/E-stop, direct DDS watcher가
준비된 경우에만 아래처럼 explicit arm한다.

```bash
status=hardware_measurements/go2_direct_remote_watchdog.json
python hardware/go2_edu_stationary_kick/stage_go2_mcf_ball_tag_webrtc.py \
  --robot-ip 192.168.123.161 --tag-id 11 \
  --direct-remote-status "$status" --execute \
  --operator-confirm MCF_CAMERA_STAGE_CLEAR_FLOOR_ESTOP_READY
```

`CAMERA_STAGING_READY`만 stage success다. `PHYSICAL_REMOTE_PREEMPTED`,
`DIRECT_REMOTE_WATCHDOG_LOST`, `ODOM_STALE`,
`PERCEPTION_REJECTED_*`, `TRAVEL_LIMIT_REACHED`, `BALL_TOO_CLOSE_NO_REVERSE`,
`CYCLE_LIMIT_REACHED`는 모두 neutral 상태로 멈춘 fail-closed 결과다. 특히
`CAMERA_STAGING_READY`는 final FR kick lane도 target-hit success도 아니다.
