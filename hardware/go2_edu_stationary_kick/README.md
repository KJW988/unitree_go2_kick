# Go2 EDU stationary FR kick runner

이 폴더의 `run.py`는 `scripts/export_vendor_go2_fr_kick_teacher.py`가 만든 frozen
FR teacher artifact만 재생한다. 자율 보행, bridge, AprilTag 추적, 공 인식, 방향 재계획은
포함하지 않는다. 실제 데모에서는 사람이 로봇/공/Tag를 이미 검증된 정지 teacher lane에
배치해야 한다.

Unitree의 공식 Python low-level 예제에서 DDS channel, `LowCmd_`, CRC 사용법만 채택했다.
공식 예제와 달리 `MotionSwitcherClient.ReleaseMode()`나 `SportClient.StandDown()`은 절대
호출하지 않는다. 출처와 차이는 [Unitree SDK2 Python Go2 stand example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/go2/low_level/go2_stand_example.py)에 기록되어 있다.

## 먼저 실제 PC에서 할 확인

실제 PC에서만 공식 SDK clone 경로에서 설치 여부와 NIC 이름을 확인한다. 이는 구독-only
확인이고 로봇 명령을 보내지 않는다.

```bash
python3 - <<'PY'
import unitree_sdk2py
print("unitree_sdk2py=", unitree_sdk2py.__file__)
PY
ip -br link
```

SDK가 없을 때에는 사용자가 공식 repository의 README대로 실제 PC에 설치해야 한다. 이
저장소는 SDK/dependency를 설치하지 않는다.

## D435i + LiDAR perception transport probe

YOLO는 D435i RGB에서 공 후보를 만들고, D435i depth로 거리를 우선 측정한다. Go2
LiDAR는 작은 공의 단독 검출을 보장하지 않으므로 depth cross-check, 장애물, odometry
기반 접근/추적용으로 사용한다. AprilTag는 벽 위 tag의 pose를 측정한 뒤 지면 투영점을
가상 골문 목표로 쓴다. 이 runner는 perception이나 robot motion을 수행하지 않는다.

실제 frontend를 연결하기 전에 아래 **read-only** probe로 D435i USB 장치와 ROS2 graph의
image/depth/cloud/odom/tf topic 및 type을 기록한다. 이 명령은 LowCmd, SportClient,
MotionSwitcher, DDS publisher를 만들지 않는다.

```bash
python hardware/go2_edu_stationary_kick/probe_perception_stack.py
```

출력 JSON의 `ros2_candidate_topics`와 `system.realsense`를 기준으로 camera intrinsics,
camera→base extrinsic, LiDAR→base transform, timestamp source를 명시한다. D435i의 공식
깊이 동작 범위는 약 0.3–3 m이므로, 최종 킥 거리에서 시야를 벗어난 공은 마지막 신뢰
측정과 odometry로만 짧게 유지하고 stale/jump gate를 통과하지 못하면 kick을 금지한다.
출처: [Intel D435i specifications](https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html).

`pyrealsense2`가 project perception env에 준비된 뒤에는 ROS2 camera wrapper 없이도 아래
read-only capture로 color-aligned depth, factory color intrinsics, depth scale과 frame 안정성을
기록한다. `last_aligned_rgbd.npz`에는 마지막 aligned color/depth pair만 저장한다. 이 단계는
공/Tag 추론이나 robot command를 전혀 수행하지 않는다.

```bash
python hardware/go2_edu_stationary_kick/capture_d435i_rgbd.py --duration-s 10
```

## D435i YOLO 공 검출 + browser stream

공은 LiDAR 단독 검출 대상으로 삼지 않는다. 먼저 D435i RGB의 공식 YOLO11n COCO
`sports ball` 후보를 얻고, 같은 D435i의 color-aligned depth로 bbox 내부의 metric
range를 확인한다. 따라서 이 단계는 RGB+depth 결합이며, camera-to-base extrinsic과
timestamp 정합 전에는 LiDAR 점군을 공 위치에 거짓으로 결합하지 않는다. LiDAR는 이후
장애물/odometry 용도로 유지한다.

모델은 git에 넣지 않는다. 실제 PC의 project-local `hardware_models/`에만 한 번
받는다. 다음 두 명령 모두 D435i를 읽기만 하며 Go2 motion/DDS를 전혀 사용하지 않는다.

```bash
python hardware/go2_edu_stationary_kick/fetch_yolo11n_model.py

python hardware/go2_edu_stationary_kick/stream_d435i_yolo_ball.py \
  --model hardware_models/yolo11n-v8.3.0.onnx --host 0.0.0.0 --port 8080 \
  --confidence 0.015 --detection-hold-s 0.50
```

같은 네트워크의 노트북 browser에서 `http://ROBOT_IP:8080`을 연다. 예를 들어 SSH가
`192.168.0.90`이면 `http://192.168.0.90:8080`이다. 초록 bbox의 `ball`은 YOLO
confidence와 aligned-depth range를 함께 표시하고, 주황 표시의 `AprilTag: [11]`은
벽 Tag가 동시에 보인다는 뜻이다. 초기에 이 화면으로 공과 Tag가 모두 들어오는 D435i
각도만 조정한다. 이 stream은 보행/킥을 절대 시작하지 않는다.

`http://ROBOT_IP:8080/state.json`은 후속 high-level walker가 읽을 수 있게 bbox,
YOLO confidence, D435i range, 검출 Tag ID를 localhost JSON으로 제공한다. stream의
낮은 기본 confidence(`0.015`)는 현장 generic COCO model이 실제 축구공에 준 약한
score를 화면에서 확인하기 위한 후보 임계값이다. 이 값 하나는 motion 권한이 아니며,
보행 전에는 range·연속성·Tag geometry를 모두 통과해야 한다.

YOLO의 단일-frame miss가 `ball: null`로 즉시 바뀌지 않도록 마지막 bbox를 최대 0.50초만
보존한다. 보존 중에도 aligned depth와 floor projection은 현재 frame에서 다시 계산하며,
`ball.detection_age_s`를 함께 내보낸다. staging은 이 값이 0.50초를 넘으면 거부하고 각
관측에서 최대 5초만 기다린다. 따라서 짧은 frame drop은 흡수하지만 오래 사라진 공 위치로
계속 움직이지 않는다. 이 변경을 적용하려면 기존 stream process를 종료하고 다시 시작해야 한다.

## LiDAR odometry bridge

공이 FR 최종 킥 위치에서 camera 아래로 사라지는 것은 정상이다. camera가 공/Tag를
볼 때 target ray를 freeze한 뒤, 마지막 짧은 docking displacement는 Go2 LiDAR odometry로
추적한다. 아래 bridge는 ROS2 CLI/RMW를 거치지 않고, 검증된 Unitree DDS
`rt/utlidar/robot_odom`을 localhost JSON으로 읽어 내보낸다. motion command를 만들지
않는다. D435 perception env와 섞지 말고 SDK conda environment의 별도 terminal에서 실행한다.

```bash
cd ~/Desktop/Jiwon/soccer/unitree_go2_kick
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ~/Desktop/Jiwon/soccer/unitree_go2_kick/.conda-unitree-sdk-py311
unset PYTHONPATH
python hardware/go2_edu_stationary_kick/bridge_utlidar_odom.py --interface eth0
```

다른 terminal에서 `curl -s http://127.0.0.1:8081/state.json`으로 `position_xyz_m`,
`yaw_rad`, `receipt_monotonic_s`가 갱신되는지 확인한다. 이 bridge는 camera↔base
extrinsic 보정 전에는 공 좌표와 직접 결합하지 않는다.

## 공/Tag 정렬 staging 보행

`approach_ball_to_tag.py`는 legacy `SportClient/ObstaclesAvoidClient.Move` 경로를 쓰며,
현재 Go2 MCF firmware에서는 API acknowledgement가 있어도 실제 보행이 일어나지 않았다.
**이 script는 실물에서 더 이상 실행하지 않는다.** frozen FR LowCmd kick과 MotionSwitcher
ownership 변경도 자동화하지 않는다.

실물에서 gait가 확인된 경로는 WebRTC bridge의 App-equivalent
`rt/wirelesscontroller`뿐이다. 새 `stage_go2_mcf_ball_tag_webrtc.py`는 D435i의 ball/Tag
camera-frame geometry와 WebRTC LiDAR odometry를 gate로 쓰고, 0.20 joystick의 짧은 pulse마다
neutral 3회와 재관측을 한다. yaw pulse는 0.50초다(이 실물에서 0.20초는 gait initiation 전에
끝날 수 있었다). 순서는 **Tag ground-ray yaw로 body/FR 방향을 kick lane과 평행하게 정렬 →
FR toe→ball→Tag 상대 bearing의 측방 보정 → yaw 재확인 → 전진**이다. robot yaw는 공과 Tag
bearing을 거의 함께 움직여 둘의 상대적인 옆 간격을 해결하지 못하므로, yaw만으로 끝내지 않고
그 다음 게걸음(`lx`)으로 FR을 공 뒤쪽 선에 맞춘다. 측방 방향은 가정하지 않고
`--allow-lateral-search`에서만 0.50초 probe 한 번을 보낸 뒤 다음 D435i depth observation으로
실제 relative-bearing 개선 여부를 확인한다.

먼저 D435 perception terminal에서 stream을 유지한다.

```bash
cd ~/Desktop/Jiwon/soccer/unitree_go2_kick
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ~/Desktop/Jiwon/soccer/unitree_go2_kick/.conda-go2-perception-py38
unset PYTHONPATH
python hardware/go2_edu_stationary_kick/stream_d435i_yolo_ball.py \
  --model hardware_models/yolo11n-v8.3.0.onnx --host 0.0.0.0 --port 8080 \
  --confidence 0.015 --detection-hold-s 0.50
```

이 firmware에서 WebRTC data-channel 구독은 physical remote input을 되돌려 주지 않는다.
따라서 virtual joystick 보행과 별개로, SDK environment의 direct DDS watcher를 먼저 유지한다.
watcher는 publisher를 만들지 않는다. 같은 read-only process가 `rt/lowstate`의 FR
motor `0/1/2`를 구독하고 Go2 URDF chain으로 `fr_foot_kinematics`를 상태 JSON에 기록한다.
이 firmware의 WebRTC `SportModeState.foot_position_body`는 12개 모두 0이어서 FR 거리 계산에
사용하지 않는다. direct DDS 결과의 `valid=true`, 증가하는 `sample_count`, 작은 `age_s`,
물리적으로 타당한 `foot_position_body_m`가 실물에서 확인되어 통합 runner의
automatic final kick distance gate에 명시적으로 연결했다. 매 실행에서 fresh FK가
없으면 fail-closed한다. `READY` 뒤 empty floor/E-stop 상태에서 physical remote
stick을 잠깐 입력해 `physical_input_event_count`를 1 이상으로 만들고, 실행 전에는 stick을
중립으로 돌린 뒤 0.6초 이상 기다린다.

WebRTC virtual joystick도 같은 `rt/wirelesscontroller` DDS topic에 echo되므로, walker는
pulse 직전에 좁은 local echo-window를 쓰고 watcher는 그 window의 **동일 값**만 self-echo로
제외한다. 다른 stick/button 값은 계속 preempt한다. SDK callback에는 DDS writer identity가
없어서 해당 매우 짧은 window 동안 physical remote가 virtual packet과 완전히 같은 값까지
보내면 구별 불가하다. 이 한계는 watchdog status JSON에도 기록되며 E-stop/physical remote는
여전히 operator의 1차 안전 수단이다.
watcher는 실행 중 Python code가 갱신되지 않으므로 `git pull` 뒤 반드시 재시작한다. stage는
status의 `virtual_echo_protocol_version`과 echo-window 경로가 최신 계약과 일치하지 않으면
execute를 거부한다.

```bash
cd ~/Desktop/Jiwon/soccer/unitree_go2_kick
source ~/miniconda3/etc/profile.d/conda.sh
conda activate ~/Desktop/Jiwon/soccer/unitree_go2_kick/.conda-unitree-sdk-py311
unset PYTHONPATH
python hardware/go2_edu_stationary_kick/watch_go2_physical_remote_dds.py \
  --interface eth0
```

다른 terminal에서 다음 read-only 확인으로 FR FK가 fresh한지 본다.

```bash
python - <<'PY'
import json
p = json.load(open("hardware_measurements/go2_direct_remote_watchdog.json"))
print(json.dumps(p["fr_foot_kinematics"], indent=2))
PY
```

watcher는 별도 terminal에서 계속 유지한다. 다른 WebRTC MCF terminal에서 아래 dry-run이
`DRY_RUN_READY`를 내는지 확인한다. 이 명령은 robot publisher를 만들지 않는다.

```bash
template=hardware_measurements/d435i_fr_ball_tag_camera_stage_template_20260728T042707Z.json
python hardware/go2_edu_stationary_kick/stage_go2_mcf_ball_tag_webrtc.py \
  --robot-ip 192.168.123.161 --tag-id 11 --fr-lane-template "$template"
```

clear floor, physical remote/E-stop, 위 direct DDS watchdog이 heartbeat와 physical input proof를
유지하는 경우에만 명시적으로 arm한다.
continuous drive가 아니라 최대 5개의 bounded pulse와 start-pose에서 최대 0.35m travel만
허용한다. `CAMERA_STAGING_READY`가 아닌 모든 결과는 neutral로 중단한다. 이 성공은 실측
camera FR lane template의 `kick_ready.eligible`일 뿐 LowCmd를 자동 시작하지 않는다. 이 EDU
firmware에서 MCF→LowCmd release와 LowCmd 종료 뒤 MCF 복귀는 토크 공백/떨림/주저앉음을 실제로
보였으므로, ownership handoff는 `live_baseline_fr_preset.py`의 별도 harness 실행으로 유지한다.

시간 제한이 있는 rough demonstration에서만
`rough_stage_then_fr_kick.py`가 bounded WebRTC staging 뒤 이미 검증한
`live_baseline_fr_preset.py` direct-release harness를 별도 SDK Python으로 실행할 수 있다.
새 LowCmd path를 만들지 않으며, strict template 오차가 남은 상태의 kick은
`--allow-rough-kick`을 별도로 요구한다. kick child는 지정한 `--kick-hold-after-s` 동안만
LowCmd baseline hold를 유지하고, **MCF 자동 handback은 하지 않는다.** 종료 뒤
ownership/자세 복구는 operator가 해야 한다.

실물 로그에서 0.50초 forward pulse는 매번 gait initiation 직후 neutral로 끊겨 2회 합계
약 0.03m만 이동했다. 따라서 forward command 상한은 calibration에서 검증된 2.0초로 두되,
각 pulse는 LiDAR odometry가 계산한 목표 거리(기본 최대 0.12m)에 도달하는 즉시 중간에
neutralize한다. duration을 길게 만든 것이 open-loop 이동거리 증가를 뜻하지 않는다.

추가 실물 로그에서 0.03m 목표를 robot_odom의 gait/body transient 한 샘플이 먼저 넘겨
전진·회전이 실제 step 전에 끊기는 현상을 확인했다. 이제 odometry stop은 active command를
최소 0.60초 유지한 뒤 서로 다른 새 sample 3개가 연속으로 목표를 확인해야 작동한다.
yaw pulse 상한은 0.80초다. lateral은 0.80초 pulse 6회에도 LiDAR 실측 횟이동이
4.5mm에 그친 실물 로그에 따라 2.0초 continuous command로 바꾸었다. 단, open-loop로
2초를 강제하지 않고 최대 0.08m의 LiDAR body-frame 횟변위가 새 sample 3개에서
연속 확인되면 즉시 neutralize한다. 남은 bearing 오차를 거리로 근사해 0.04–0.08m
범위로 한 번만 이동한 뒤 다음 D435i frame에서 재관측한다. camera-visible 목표까지
0.04m 이내인 작은 잔여 거리는 짧은 pulse를 반복하지 않고, 정렬을 다시 gate한 뒤 기존
LiDAR-odometry bounded final dock으로 넘긴다. joystick magnitude 0.20과 전체 travel/duration
hard limit은 그대로다.

통합 runner는 고정 `camera→FR` 거리 대신 direct DDS `rt/lowstate`+Go2 URDF FK로
현재 FR foot body x를 매번 다시 읽는다. D435i mount는 body에 고정되므로, 동시
실측한 `FR body x=0.185136m` 와 렌즈가 FR보다 0.15m 앞이라는 값에서
`camera body x=0.335136m`를 1회 보정했다. 5호 공 반지름 0.11m를 따로 반영해
`FR toe→공 표면 0.15m`, 즉 `FR→공 중심 0.26m`를 final dock 목표로 쓴다.
정지 뒤 fresh FK로 다시 계산한 표면 gap이 0.10–0.17m이고 FR foot speed가
0.05m/s 이하인 경우에만 `FINAL_DOCKING_READY`가 된다. 실패하면 LowCmd child를
시작하지 않는다.

실물 최종 dock에서 FR→공 중심 약 0.20m까지 gait를 유지하자 마지막 FR swing이
LowCmd보다 먼저 공을 접촉했다. 따라서 동적 FK 목표는 중심 0.26m(표면 gap 0.15m)로
두어 기존 접촉점보다 0.06m 보수적으로 멈춘다. 이것은 이전 약한 접촉 지점
0.29m보다는 0.03m 가깝다. duration hard limit은 6.0초로 유지하되 LiDAR odometry
목표 도달 즉시 neutralize한다.
duration hard limit은 6.0초로 유지하되 LiDAR odometry 목표 도달 즉시 neutralize한다.
neutral 뒤에는 최근 0.40초의 MCF planar/yaw velocity와 LiDAR odometry span이 모두 정지
gate를 통과해야만 LowCmd child를 시작한다. 이 clearance는 첫 보수값이며 실물 접촉/미접촉
결과로만 줄인다.

5초 pulse는 Python scheduling 지연으로 nominal wall time보다 길어질 수 있다. 고정 expiry가
먼저 끝나 virtual `ly=0.2` echo를 물리 remote로 오인하지 않도록, stage는 active 동안
0.20초마다 0.50초 exact-value lease를 갱신하고 neutral 뒤 즉시 해제한다. writer identity가
없는 한 동일 값의 동시 물리 입력을 구분할 수 없다는 기존 제한과 E-stop 우선 원칙은 같다.

통합 runner의 기본 `--lane-axis-bearing-rad 0.0`은 D435i가 robot 전진축과 평행하게
장착됐다는 현장 조건에서 ball→Tag 지면축을 body/FR 전진축과 평행하게 만든다. captured
template의 range와 ball-Tag 상대 bearing은 그대로 사용한다. 통합 runner의 heading deadband는
0.04rad이며, yaw command는 LiDAR odometry가 요청 yaw 변화량을 확인하면 pulse 상한 전에
neutralize한다.
카메라 yaw가 body와 평행하지 않다면 0.0을 임의로 쓰지 말고 mount yaw를 실측해 이 값을
바꿔야 한다.

```bash
python hardware/go2_edu_stationary_kick/rough_stage_then_fr_kick.py --help
```

통합 runner는 전진 pulse를 최대 0.15m까지 유지해 짧은 neutral 반복을 줄이고, 마지막
continuous final dock이 끝나면 최근 1.0초의 **새로운** `rt/lowstate` sample만 사용해 기존
`0.006rad` joint-span gate를 통과할 때 바로 LowCmd로 전환한다. 고정 4초 capture 전체에 gait
transient를 섞던 동작은 제거했다. 통합 기본은 추가 countdown 0.5초, frozen trajectory
`time-scale=0.95`, FR swing delta `1.3`이며 `Kp=60`, `Kd=5`, handoff blend는 바꾸지 않는다.
즉 작은 회전을 억지로 반복하지 않고 실제 보행을 완료한 뒤, 정지 확인과 토크 연속성을
보존한 상태에서 강화 kick을 한 번 실행한다.

세 terminal의 perception stream과 direct remote watcher가 정상인 상태에서 통합 실행은
아래 한 명령이다. Tag 11이 보이면 tag-guided 정렬을 우선하고, Tag가 처음부터 없을 때만
명시된 ball-only fallback을 쓴다. `--allow-rough-kick`은 포함하지 않으므로 strict geometry나
final dock/settle 중 하나라도 실패하면 LowCmd는 시작되지 않는다.

```bash
template=hardware_measurements/d435i_fr_ball_tag_camera_stage_template_20260728T042707Z.json
status=hardware_measurements/go2_direct_remote_watchdog.json
trajectory=hardware_measurements/go2_fr_kick_teacher_x10.npz

python hardware/go2_edu_stationary_kick/rough_stage_then_fr_kick.py \
  --robot-ip 192.168.123.161 --tag-id 11 \
  --fr-lane-template "$template" --direct-remote-status "$status" \
  --allow-tagless-ball-kick \
  --lane-axis-bearing-rad 0.0 \
  --stage-target-bearing-tolerance-rad 0.04 \
  --stage-ball-bearing-tolerance-rad 0.03 \
  --stage-min-odom-stop-active-s 0.60 \
  --stage-odom-stop-confirm-samples 3 \
  --stage-camera-entry-slack-m 0.04 \
  --stage-forward-pulse-s 2.0 \
  --stage-max-forward-pulse-travel-m 0.15 \
  --camera-to-fr-forward-m -0.15 --fr-to-ball-forward-m 0.18 \
  --camera-body-forward-m 0.335136277245694 \
  --ball-radius-m 0.11 \
  --final-fr-to-ball-min-surface-gap-m 0.10 \
  --final-fr-to-ball-target-surface-gap-m 0.15 \
  --final-fr-gap-tolerance-m 0.02 \
  --final-fr-max-foot-speed-m-s 0.05 \
  --final-settle-timeout-s 2.0 \
  --final-dock-max-m 0.85 --final-dock-max-duration-s 6.0 \
  --max-stage-cycles 6 --max-stage-travel-m 0.35 \
  --interface eth0 \
  --lowcmd-python .conda-unitree-sdk-py311/bin/python \
  --trajectory "$trajectory" --kp 60 --kd 5 \
  --handoff-blend-s 1.2 --prehold-s 1.0 \
  --preset-time-scale 0.95 --fr-swing-scale 1.3 \
  --kick-baseline-stable-window-s 1.0 \
  --kick-baseline-stable-timeout-s 12.0 \
  --kick-start-countdown-s 0.5 --kick-hold-after-s 60 \
  --execute \
  --operator-confirm ROUGH_STAGE_THEN_FR_KICK_ESTOP_READY
```

```bash
status=hardware_measurements/go2_direct_remote_watchdog.json
python hardware/go2_edu_stationary_kick/stage_go2_mcf_ball_tag_webrtc.py \
  --robot-ip 192.168.123.161 --tag-id 11 --fr-lane-template "$template" \
  --direct-remote-status "$status" --execute \
  --operator-confirm MCF_CAMERA_STAGE_CLEAR_FLOOR_ESTOP_READY
```

### AprilTag 없는 ball-only fallback

기본 동작은 계속 Tag 11과 `target_line`을 요구한다. Tag가 없는 시연 환경에서는
`--allow-tagless-ball-kick`을 **명시한 경우에만** captured FR lane의 ball bearing과
D435i aligned-depth/floor plane을 사용한다. Tag가 보이면 기존 tag-guided 정렬이 우선되고,
처음부터 Tag가 없을 때만 `alignment_mode=ball_only`가 된다. ball-only 실행은 FR 앞의 공
위치와 거리는 확인하지만 공이 날아갈 target 방향은 확인할 수 없다. 출력에도
`target_direction_verified=false`가 남는다.

먼저 motion 없는 dry-run으로 `dry_run_next_action`, `ball_error_rad`, final dock 거리를 확인한다.
`--ball-bearing-tolerance-rad 0.03`은 captured template의 tolerance를 완화하지 않고 더 엄격한
runtime 상한으로만 작동한다.

```bash
python hardware/go2_edu_stationary_kick/stage_go2_mcf_ball_tag_webrtc.py \
  --robot-ip 192.168.123.161 --tag-id 11 --fr-lane-template "$template" \
  --direct-remote-status "$status" --allow-tagless-ball-kick \
  --lane-axis-bearing-rad 0.0 --ball-bearing-tolerance-rad 0.03 \
  --enable-final-dock --use-direct-fr-kinematics \
  --camera-body-forward-m 0.335136277245694 \
  --ball-radius-m 0.11 \
  --final-fr-to-ball-min-surface-gap-m 0.10 \
  --final-fr-to-ball-target-surface-gap-m 0.15
```

통합 runner에도 같은 이름의 `--allow-tagless-ball-kick`을 주면 stage child로 전달된다.
이 옵션은 `--allow-rough-kick`과 다르다. 전자는 Tag 없는 geometry를 명시 허용하고, 후자는
남아 있는 strict geometry 오차까지 무시하는 별도 override다. ball-only에서도 공 검출,
depth/floor plane, bearing tolerance, LiDAR odometry, final settle 및 remote watchdog gate는
그대로 유지된다.

dry-run이 `lateral_to_fr_lane`이면, 이것은 먼저 FR lane으로 게걸음해야 한다는 뜻이다. clear
floor에서 아래 command는 `lx=+0.20` pulse **1회**(최대 2.0초/횟변위 0.08m),
안정 재관측 **1회**만 수행한 뒤 neutral로
끝낸다. 출력의 `lateral_probe_results[0].improvement_rad > 0` 및
`recommended_lateral_sign`을 확인한다. `sign_confirmed`이면 그 부호를 다음 full-stage의
`--lateral-sign`과 `--lateral-sign-confirmed`에 그대로 사용한다. `flip_sign_and_retry`이면
반대 부호로 같은 probe-only command를 한 번 더 실행한다.
부호가 확인된 뒤에도 relative bearing이 0을 지나치면 관측된 lateral response 부호로
다음 `lx` 방향을 자동 반전해 overshoot를 막는다.

```bash
python hardware/go2_edu_stationary_kick/stage_go2_mcf_ball_tag_webrtc.py \
  --robot-ip 192.168.123.161 --tag-id 11 --fr-lane-template "$template" \
  --direct-remote-status "$status" --allow-lateral-search --lateral-probe-only \
  --max-cycles 2 --execute \
  --operator-confirm MCF_CAMERA_STAGE_CLEAR_FLOOR_ESTOP_READY
```

YOLO11n ONNX artifact 출처는
[Ultralytics assets v8.3.0](https://github.com/ultralytics/assets/releases/tag/v8.3.0)이며,
project downloader가 SHA-256을 고정한다. OpenCV DNN decoder는 기존 YOLOv5와 YOLO11
출력 형식을 모두 지원하지만 실물 실행 기본 artifact는 `yolo11n-v8.3.0.onnx`다.

출력 `metadata.json`의 `camera_to_base_extrinsic`은 의도적으로 `null`이다. 카메라 mount의
실측 rigid transform을 얻기 전에는 base-frame ball/Tag 좌표나 접근 command를 만들지 않는다.

카메라를 정지 standing 자세에서 대략 아래로 향하게 고정한 뒤에는 다음 read-only probe로
D435i accelerometer/gyroscope 통계를 기록한다. USB 2.x 연결에서는 RGB+IMU 동시
stream이 timeout 날 수 있으므로 이 probe는 IMU-only 100/200 Hz를 쓴다. factory
IMU→RGB 변환은 별도 `rs-enumerate-devices -c` 또는 이전 factory probe로 기록한다.
D435i IMU는 camera의 실제 roll/pitch 확인에는 유용하지만, 자력계가 없으므로 이것만으로는
base 기준 yaw 또는 camera 위치 `x/y/z`를 얻을 수 없다.

```bash
python hardware/go2_edu_stationary_kick/probe_d435i_imu_extrinsics.py --duration-s 10
```

벽의 Tag 중심 지면투영점을 가상 골문으로 쓸 때도, 첫 실제 검증은 camera-frame pose만
기록한다. 아래 probe는 `tag36h11`의 모든 ID를 자동 검출하고, 검은 외곽 정사각형 실제
변 길이와 Tag 중심 높이를 metadata에 보존한다. `camera_to_base_extrinsic`과
`ground_projection`은 base 정합 전까지 의도적으로 `null`이다.

```bash
python hardware/go2_edu_stationary_kick/probe_d435i_apriltag.py \
  --tag-size-m 0.152 --tag-center-height-m 0.300 --duration-s 10
```

D435i live IMU stream을 사용할 수 없는 설치에서는 고정 벽 Tag와 바닥의 표시된
calibration spot을 mount integrity 기준기로 쓴다. 이 방법은 IMU와 달리 camera의
translation과 yaw 변화도 감지한다. baseline을 만들 때와 검증할 때 모두 robot이 같은
floor mark에 같은 정면 방향으로 standing 해야 하며, PASS는 그 조건 아래의 mount 정상만
뜻한다.

```bash
python hardware/go2_edu_stationary_kick/create_d435i_tag_mount_baseline.py \
  --probe hardware_measurements/d435i_apriltag_probe_TIMESTAMP/metadata.json \
  --tag-id 11 --operator-confirm CALIBRATION_SPOT_MARKED

python hardware/go2_edu_stationary_kick/verify_d435i_tag_mount.py \
  --baseline hardware_measurements/d435i_tag_mount_baseline.json \
  --probe hardware_measurements/d435i_apriltag_probe_TIMESTAMP/metadata.json
```

## artifact와 attestation

실제 PC에 이 저장소를 복사한 뒤 artifact를 생성한다.

```bash
cd /path/to/dribblebot
python3 scripts/export_vendor_go2_fr_kick_teacher.py \
  --output /path/to/go2_fr_kick_teacher.npz
cp hardware/go2_edu_stationary_kick/hardware_attestation.template.json \
  /path/to/hardware_attestation.json
```

`hardware_attestation.json`의 12개 encoder scale/offset, position/torque limit,
`lowcmd_kp/kd`, `validated_command_speed_limit_rad_s`는 실제 Go2 EDU firmware와
무공 hoist 검증으로 측정한 값만 적는다. speed field는 firmware 최고 속도가 아니라
이 specific trajectory에 허용한 command envelope다.
template의 문자열/placeholder를 실제 값으로 바꾸지 않은 상태는 의도적으로 arm되지
않는다. 이 값은 추정하거나 이 저장소의 simulation 값으로 채우면 안 된다.

실측 정지 자세가 simulation teacher의 시작 자세와 다르면 teacher artifact를 그대로
실행하면 안 된다. read-only capture로부터 시작점만 치환하고 frozen teacher의 관절
변화량을 보존하는 artifact를 먼저 만든다. 이 명령은 DDS/SDK를 열지 않는다.

```bash
python3 scripts/build_go2_hardware_baseline_teacher.py \
  --teacher /path/to/go2_fr_kick_teacher.npz \
  --capture hardware_measurements/go2_static_stand_capture.json \
  --output /path/to/go2_fr_kick_teacher_hardware_baseline.npz
```

새 artifact도 `unattested_do_not_send_to_robot` 상태다. 이는 초기 자세 불일치만
해소하며 encoder 원점/부호, joint limit, gain, 실제 support/ball safety를 증명하지 않는다.
현재 export는 최대 약 `10.7 rad/s` 구간이 있어 command speed field가 검증되지 않으면
`--execute`가 LowCmd publisher 생성 전에 실패한다.

실물용 재시간화는 explicit envelope 없이 default로 수행하지 않는다. 먼저 baseline
artifact를 만들고, 무공 hoist 검증 계획에서 정한 speed/acceleration/jerk 값으로 변환한다.
아래는 offline 변환 형식이며 숫자는 예시가 아니라 검증 후에만 입력한다.

```bash
python3 scripts/retime_go2_hardware_teacher.py \
  --teacher /path/to/go2_fr_kick_teacher_hardware_baseline.npz \
  --output /path/to/go2_fr_kick_teacher_retimed.npz \
  --max-speed-rad-s VERIFIED_VALUE \
  --max-acceleration-rad-s2 VERIFIED_VALUE \
  --max-jerk-rad-s3 VERIFIED_VALUE
```

출력의 `max_discrete_speed_rad_s`는 attestation의 speed envelope 이하이어야 한다.

## 첫 실행: 구독-only preflight

먼저 robot app에서 low-level ownership 충돌이 없도록 사람이 준비한다. 이 runner는
ownership을 변경하지 않는다. E-stop, hoist, clear zone을 준비한 상태에서 아래를
실행한다. `enp2s0`는 예시이며 실제 NIC 이름으로 바꾼다.

```bash
python3 hardware/go2_edu_stationary_kick/run.py \
  --interface enp2s0 \
  --trajectory /path/to/go2_fr_kick_teacher.npz \
  --hardware-attestation /path/to/hardware_attestation.json
```

정상은 `LOWSTATE_CONNECTED`, `PREFLIGHT_PASS`, `PREFLIGHT_ONLY_PASS`다. 이 모드에서는
`rt/lowcmd` publisher 자체를 만들지 않는다. `max_pose_error_rad`가 0.05를 넘으면
로봇을 손으로 억지로 맞추거나 trajectory를 수정하지 말고, motor mapping/encoder offset/
teacher 시작 자세를 먼저 재검증한다.

## 실행은 마지막 단계

무공 hoist test와 별도 사람 승인 뒤에만 실행한다. 아래 명령은 약 3초 뒤 LowCmd를
발행하며, 6초 뒤 teacher initial pose로 복귀한다.

```bash
python3 hardware/go2_edu_stationary_kick/run.py \
  --interface enp2s0 \
  --trajectory /path/to/go2_fr_kick_teacher.npz \
  --hardware-attestation /path/to/hardware_attestation.json \
  --execute --operator-confirm I_UNDERSTAND_LOWCMD
```

실행 중 anomaly, support loss, unexpected motion이 보이면 software abort에 의존하지 말고
즉시 E-stop을 사용한다. 이 runner는 low-level ownership/firmware timeout/zero-torque
recovery를 임의로 변경하지 않는다.

## Harness live-baseline FR preset deploy

정지 harness에서 이미 학습·검증한 FR preset의 joint delta를 그대로 재생하면서 매 tick의
target/실제 joint state/IMU를 JSON으로 기록하는 별도 runner다. simulator default pose를
실물에 보내지 않고, 매 실행 시 4초간 읽은 standing median을 시작점으로 쓴다. 보행·공·Tag
입력은 하지 않는다. `--execute` 전 preview는 LowCmd publisher를 만들지 않는다.

```bash
python3 hardware/go2_edu_stationary_kick/live_baseline_fr_preset.py \
  --interface eth0 \
  --trajectory /path/to/go2_fr_kick_teacher.npz \
  --kp YOUR_TUNING_VALUE --kd YOUR_TUNING_VALUE
```

Harnes/E-stop/no-ball/low-level ownership이 모두 준비된 뒤에만 `--execute
--operator-confirm HARNESS_ESTOP_READY`를 추가한다. 이 runner는 gain을 숨겨진 default로
정하지 않는다. operator가 입력한 값을 그대로 log에 기록해 다음 tuning의 근거로 쓴다.

Sport/MCF가 standing pose를 소유한 경우 LowCmd는 무시된다. harness에서 standing baseline을
capture한 뒤에만 `--release-motion-owner`를 추가하면 captured baseline LowCmd stream을 먼저
200 Hz로 시작하고, 그 stream을 유지한 채 official `StandDown`/`ReleaseMode`를 수행한다.
그 뒤 `--prehold-s` 동안 hold한 다음 preset을 시작한다. release만 먼저 수동으로 실행하면
robot이 주저앉아 baseline이 사라지므로 사용하지 않는다.

`--release-motion-owner`를 처음 사용할 때는 `--hold-only --hold-only-s 3`으로 standing
handoff만 먼저 확인한다. 이 단계에서 robot이 자세를 유지하고 log의 joint tracking이
작아야만 `--hold-only`를 빼고 FR preset을 실행한다.

기본 경로는 `StandDown` 뒤 `ReleaseMode`다. 반면 Unitree의 공식 C++ Go2 stand example은
`StandDown` 없이 `ReleaseMode`만 호출한다. 이 direct 경로는 앉음/떨림을 줄일 후보지만,
공식 예제도 harness 또는 지면 조건을 요구한다. 따라서 아래처럼 explicit opt-in과 무공
`--hold-only`에서만 먼저 검증한다. 성공하기 전에는 킥에 쓰지 않는다.

```bash
python3 hardware/go2_edu_stationary_kick/live_baseline_fr_preset.py \
  --interface eth0 --trajectory hardware_measurements/go2_fr_kick_teacher_x10.npz \
  --kp 60 --kd 5 --execute --release-motion-owner \
  --release-without-stand-down --handoff-blend-s 1.2 --prehold-s 1 \
  --hold-only --hold-only-s 3 --hold-after-s 20 \
  --operator-confirm HARNESS_ESTOP_READY
```

출처: [Unitree SDK2 Go2 C++ stand example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp).

handoff 중 지지 토크를 0으로 낮추면 `StandDown` 뒤 robot이 주저앉을 수 있다. runner는
release 전부터 operator가 지정한 full `Kp/Kd` standing target을 계속 송신하고,
`--handoff-blend-s`로 actual `release_q`에서 standing baseline target만 연결한다.

명령이 단순 종료하면 LowCmd stream도 종료되어 harness robot은 다시 힘이 빠질 수 있다.
이 EDU firmware에서는 LowCmd가 살아 있는 동안
`RobotStateClient.ServiceSwitch("mcf", True)`가 `3104`으로 거부됐다. 반대로 stream이
종료된 뒤에는 MCF를 복구할 수 있지만, 그 사이 지지 토크 공백이 생긴다. 따라서 이 runner는
현재 자동 LowCmd→MCF handback을 제공하지 않는다. `--hold-after-s`는 관찰 시간을 늘릴
뿐 종료 뒤의 토크 공백을 해결하지 않는다.

즉, persistent kick session과 controller 우선권은 firmware가 제공하는 원자적 handback API
또는 MCF 내부에서 실행되는 kick action을 확보하기 전까지 이 LowCmd runner로 구현하면 안
된다. 출처: [Unitree RobotStateClient](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/robot_state/robot_state_client.py), [Unitree Go2 C++ stand example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp).

`--preset-time-scale 0.70`은 frozen FR preset의 관절 path를 바꾸지 않고 전체 시간을 70%로
줄인다. `0.20`은 5배 속도다. handoff tracking이 안정된 harness run에서만 점진적으로 올린다.

`--fr-swing-scale 1.15`는 support preload와 시작/복귀 자세를 보존한 채 kick phase의 raw
FR thigh/calf delta만 15% 키운다. 이는 forward reach physical tuning용이며 `0.8..1.3` 범위를
넘지 않는다.

발끝의 **Cartesian 종점 자체**를 더 전방으로 보내야 할 때는 export 시에만 다음처럼
`--fr-forward-extension-m`을 준다. `0.10`은 exporter의 같은 planar FK와 joint-limit
projection 기준으로 기본 teacher 종점보다 실제 toe x 종점을 10 cm 앞으로 둔다. 기본값 `0`은
frozen teacher와 완전히 같다. 이 physical-only override는 simulator teacher/checkpoint를
바꾸지 않으며, harness와 no-ball 조건에서 먼저 검증한다.

```bash
python3 scripts/export_vendor_go2_fr_kick_teacher.py \
  --output hardware_measurements/go2_fr_kick_teacher_x10.npz \
  --fr-forward-extension-m 0.10
```
