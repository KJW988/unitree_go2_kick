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
