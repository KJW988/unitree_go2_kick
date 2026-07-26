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

명령이 끝나면 LowCmd stream도 종료되어 harness robot은 다시 힘이 빠질 수 있다. 관찰이나
manual handback 전에 baseline을 유지하려면 `--hold-after-s 30`처럼 추가 hold 시간을 준다.

MotionSwitcher가 반환한 owner name은 firmware별로 `SelectMode()` 가능한 alias와 다를 수
있다. 기본 handback은 하지 않는다. 먼저 아래 read-only query로 controller mode를 확인하고
확정된 alias가 있을 때만 `--handback-mode ALIAS`를 준다.

```bash
python3 hardware/go2_edu_stationary_kick/query_motion_switcher.py --interface eth0
```

Go2의 공식 MotionSwitcher 예제는 mode alias로 `normal`, `ai`, `advanced`를 사용한다.
persistent kick session의 controller 우선권을 구현하기 전에는 아래 probe로 해당 EDU
firmware에서 alias 하나의 reacquire가 성공하는지 확인한다. 이 probe는 LowCmd와 Sport
motion 명령을 만들지 않지만 mode selection은 수행하므로 robot이 정상 standing이고 clear
zone인 상태에서만 쓴다. 자동 후보 순회는 하지 않는다.

```bash
python3 hardware/go2_edu_stationary_kick/probe_motion_switcher_alias.py \
  --interface eth0 --mode ai --execute \
  --operator-confirm SELECT_MODE_READY
```

`select_status=0`, `after.name`이 비어 있지 않으면 controller-mode reacquire 후보가
확인된 것이다. 출처: [Unitree MotionSwitcher Python example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/motionSwitcher/motion_switcher_example.py), [Unitree Go2 C++ stand example](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_stand_example.cpp).

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
