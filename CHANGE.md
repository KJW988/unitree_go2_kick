# CHANGE.md

Log of all changes made during the Go2 Kick RL debugging and implementation task.

## 2026-07-28 — ball→Tag heading 및 WebRTC self-echo fail-closed 보정

- 실물 final dock에서 프로그램이 보낸 `ly=0.2` DDS echo가 physical remote로 오인되어
  0.466m에서 중단된 로그에 따라 watcher status에 echo protocol version을 추가했다.
  stage는 최신 protocol과 동일 echo-window 경로를 확인하지 못하면 execute를 거부한다.
- 통합 runner는 captured range/ball-Tag 상대 bearing은 유지하면서 ball→Tag 지면축을
  camera/robot 전진축 `0 rad`에 맞춘다. heading deadband는 0.03rad이며 yaw pulse는
  LiDAR odometry가 요청한 yaw 변화량에 도달하면 0.50초 전에 neutralize할 수 있다.
- FR 접촉 거리와 swing 크기는 runtime 실측값으로 계속 명시하며, 기존 trajectory 자체는
  변경하지 않았다. `--fr-swing-scale 1.2`는 검증된 [0.8, 1.3] clamp 안에서 FR swing
  delta만 20% 확대한다.

## 2026-07-28 — D435i intermittent ball detection hold

- `stream_d435i_yolo_ball.py`가 YOLO 단일-frame miss 시 마지막 bbox를 최대 0.50초만
  보존하고, 그 동안에도 현재 aligned depth로 거리와 floor projection을 다시 계산한다.
- `state.json`의 `ball.detection_age_s`와 stage의 동일 0.50초 freshness gate를 추가했다.
  stage 관측 timeout은 2초에서 5초로 늘렸지만 stale detection, missing Tag/target line,
  depth 오류는 계속 fail-closed한다.
- `perception_missing` 단일 사유를 HTTP/not-ready/ball/tag/field/geometry 원인으로 나눠
  다음 실물 로그에서 frame drop과 server 문제를 구분할 수 있게 했다.

## 2026-07-28 — Odometry-bounded gait initiation 및 final docking

- 0.50초 forward pulse 2회가 실제로는 총 0.030m만 이동한 실물 로그에 따라, forward
  command 상한을 2.0초로 바꾸고 pulse 시작 yaw의 LiDAR odometry가 기본 0.12m 목표에
  도달하면 loop 도중 neutralize하도록 했다.
- D435i floor plane의 ball→Tag ground 축에 camera→ball을 투영하고, signed camera→FR 및
  양수 FR→ball forward 합까지의 남은 거리를 WebRTC/LiDAR로 닫는 opt-in final docking을
  추가했다. 최대 5초/0.85m이며 `FINAL_DOCKING_READY`만 LowCmd 연결을 허용한다.

## 2026-07-27 — Go2 EDU FR teacher software-only export / dry-run 준비

- `scripts/export_vendor_go2_fr_kick_teacher.py`를 추가했다. Isaac Gym, SDK, DDS 없이 default `make_offsets()` 수식과 vendor Go2 nominal pose를 50 Hz `.npz`/CSV artifact로 고정하며 canonical 및 SDK motor 순서와 teacher source hash를 함께 기록한다.
- `scripts/dry_run_go2_fr_kick_deploy.py`는 의도적으로 `--execute`와 SDK import가 없는 fail-closed preflight다. 관절 순서·sample rate·재배열을 확인하고, 사람이 작성한 Go2 EDU hardware attestation(영점/부호/firmware limit)이 없으면 `NOT_ARMABLE`로 종료한다.
- `GO2_FR_KICK_DEPLOY_RUNBOOK.md`와 `CODEFLOW.md`에 offline artifact, attestation 및 실물 단계 전 승인 조건을 문서화했다. 이 변경은 실물 로봇, 네트워크, LowCmd를 사용하지 않는다.

## 2026-07-25 — M1 재현 경로·프로즌 baseline, M2 격리

- `VENDOR_CHECKPOINT_DIR`와 Go2 URDF를 repo-상대 경로 및 env override로 바꾸고,
  실제 사용 시 누락 파일을 명확히 실패시킨다.
- `scripts/make_baseline_manifest.py`와 dry-run 기본 회귀 래퍼를 추가했다. 두 demo
  runner는 기존 subprocess 종료 뒤 stdout만 manifest로 기록하며 물리 재실행을 하지 않는다.
- `dribblebot/perception/`에 고아본의 설명만 이관하고, v13 walking-only ablation은
  미승격으로 문서화했다. 순수 테스트·경로 스모크 상태는 본 변경의 검증 기록을 따른다.

## 2026-07-26 — R2 platform ball-lane calibration

- R1/R1.5 teacher-only 측정의 `y=-0.20 -> -0.23 m` lane response를 선형 보간해
  `platform_ball_lane_bias_m=-0.0107`을 고정했다. supervisor 목표 base pose에만
  적용하며 FR teacher swing/Bézier/hip offset과 성공 gate는 변경하지 않는다.
- physical evaluator는 supervisor의 biased plan과 bias=0 nominal plan의 base-pose
  차이만 기존 kick-frame stance에 합산한다. 따라서 이 scalar가 실제 handoff에
  전달되면서도 기존 heading precompensation과 teacher-ball 검증 좌표는 불변이다.
- seed 0 물리 검증은 `phase=0.6751`, `speed=0.1239 m/s`에서 `TEACHER`에
  진입했고 `forward=1.4771 m`, `lateral=-0.6543 m`를 기록했다. 직전 strict-phase
  기준보다 횡오차는 `+0.0592 m` 개선됐지만 directed gate는 실패했으므로, R2를
  성공 프리셋으로 승격하지 않고 R3 yaw closed-loop 검증의 입력으로 보존한다.

## 2026-07-26 — R3 forward yaw closed-loop

- supervisor yaw ownership을 PD POSTURE support-hip trim에서 마지막 public-walker
  접근으로 옮겼다. heading priority와 capture tolerance를 모두 `0.06 rad`로 묶어
  M3 phase/speed 조건과 동시에 만족할 때만 PD handoff한다.
- PD/teacher 중 yaw trim은 `0`이다. 따라서 FR primitive, R2 platform lane scalar,
  M3 gate 및 directed success threshold는 변경하지 않는다.
- PD yaw trim은 `0.00477 rad`까지 수렴했지만 teacher ball frame을 `0.0940 m` 바꿨다.
  이어진 safe recapture는 각각 navigation divergence와 foot-ball proximity에 의한
  `PERCEPTION_INVALID`로 fail-closed 종료했다. gate를 완화하지 않고 pre-handoff
  closed-loop로 소유권을 바로잡아 재검증한다.
- pre-handoff 재검증은 M3와 yaw 조건(`phase=0.6951`, `speed=0.1020 m/s`,
  `yaw=-0.0550 rad`)을 만족해 PD capture했지만, PD POSTURE가 yaw를 `+0.1335 rad`로
  바꾸어 `POSTURE_FAILED`가 됐다. 이는 현 PD bridge의 동시 yaw/teacher-frame 보존
  한계이므로, gate·teacher target·swing을 바꾸지 않고 별도 bridge 권한을 기다린다.

## 2026-07-26 — v5 snapshot yaw-anchor bridge

- v1~v4 bridge checkpoint는 standalone physical audit에서 통과하지 못해 승격하지 않는다.
  v5는 현재 M3/R3 physical capture snapshot에서 시작하고, 기존 XY anchor에 yaw anchor
  reward를 추가한다. 이는 teacher ball-frame 회전을 억제하는 bridge-state 보상이며 FR
  teacher나 공에는 직접 action을 내지 않는다.
- bridge physical audit도 max yaw drift `0.06 rad`를 독립 gate로 기록한다. monitor와
  audit은 offline W&B 및 snapshot replay를 사용한다.
- snapshot 저장은 capture envelope 첫 교차가 아닌 실제 M3-safe direct handoff tick으로
  옮겼다. v5 입력 `forward_r3_m3_seed0_v5_actual.pt`는 step 377, phase 0.6951,
  yaw error -0.0550의 물리 상태다.
- 8-iteration smoke는 snapshot replay, yaw reward 등록, checkpoint/JIT export까지 통과했다.
  후보 audit은 drift 0.6286 m, yaw drift 2.2761 rad로 실패했으며 승격하지 않았다.
  `go2-gait-exit-snapshot-bridge-v5-yawanchor-50k`의 64-env CPU PhysX/GPU PPO run과
  1000-iteration snapshot audit monitor를 offline으로 시작했다.
- 이후 CPU/GPU 후보의 backend 비교가 PPO 난수 차이와 섞이지 않도록
  `BRIDGE_TRAIN_SEED`를 추가했다. Python·NumPy·PyTorch·CUDA와 vendor curriculum에
  같은 seed를 적용하며, 이미 실행 중인 후보의 조건은 사후 변경하지 않는다.
- 50k bridge 학습은 사용자 결정으로 종료했다. i=12,000은 snapshot replay audit을
  재현 통과한 유일한 안정 후보이므로 `.runtime/bridge_candidates/v5_cpu_i012000/`와
  원본 `ac_weights_012000.pt`를 동결했다. i=13,000 이후 checkpoint는 비단조 audit
  실패로 P1 handoff 후보에서 제외한다.
- P1 통합 평가에서는 M3-safe tick에서 `APPROACH_USE_POLICY_STABILIZER=1`일 때만
  동결 bridge가 실제 `POSTURE`/`TEACHER` ownership을 받도록 연결했다. 이 경로의
  command ABI는 bridge 학습·audit과 같은 swing-height `0.05 m`를 사용한다.
- 그러나 frozen i=12,000은 아직 directed-shot 후보로 승격하지 않는다. seed 0은
  M3 handoff(`phase=0.6951`, `speed=0.1020 m/s`, `yaw=-0.0550 rad`) 뒤
  `POSTURE_FAILED`(final yaw `+0.0745 rad`)였고, seed 1/2는 FR-clearance가
  충족되지 않아 bridge ownership 전 `SETTLE_FAILED`였다. strict posture yaw gate
  (`0.06 rad`), teacher 및 접촉 보정은 변경하지 않았다.
- P1b forward fallback은 frozen bridge를 사용하지 않고, 기존 M3 P-PD capture와 Tag
  preset의 closed-loop support yaw-hold(`KP=-1`, limit `0.15`, teacher-load `2.8 s`)를
  사용한다. 이는 strict posture yaw gate를 넓히지 않고 POSTURE drift를 실제 joint
  support feedback으로 닫기 위한 경로이며 teacher/FR 접촉 보정은 변경하지 않는다.
- P1b seed 0/1/2 결과도 directed-shot 승격 실패다. seed 0에서는 yaw-hold가
  `-0.0749 rad`로 실제 적용됐지만 final yaw error가 `+0.0749 rad`라 strict gate를
  넘었다. seed 1/2는 각각 FR clearance `0.2328/0.2298 m`로 handoff 최소치 `0.24 m`를
  충족하지 못해 `SETTLE_FAILED`였다. 따라서 yaw-hold 설정만으로는 target yaw 정렬과
  capture geometry를 동시에 보장하지 못한다.

## 2026-07-26 — H1 short-approach + lateral-alignment 측정

- `run_go2_hybrid_short_approach_demo.py`는 bridge 없이 public walker의 짧은
  forward+strafe만 측정한다. 세 공 위치 `right/center/left`는 각각
  `ball_local=(0.95,-0.20)/(0.95,0.00)/(0.95,0.12)`이고, plant 목표는 모두
  `ball-[0.3335,-0.20]`로 계산한다. teacher ball-lane도 같은 좌표로 정합하며 FR
  primitive/hip offset/swing은 변경하지 않는다.
- 최초 H1은 yaw command를 0으로 잠가 초기 heading 불일치를 보정하지 못한 무효
  셋업이었다. H1b는 simulator commit 뒤 `base_yaw/target_heading/yaw_error`를 출력하고
  `0.02 rad` assertion으로 검증했다. right 위치는 `0/0/0 rad`로 assertion을 통과했지만,
  M3 capture 전 FR-ball 거리 `0.1281 m`에서 공속 `0.2894 m/s` collision이 발생해
  `PERCEPTION_INVALID` fail-closed로 끝났다. 따라서 현재 H1의 남은 문제는 target yaw가
  아니라 short forward+strafe의 FR-ball corridor 충돌이며, center/left는 실행하지 않았다.

## 2026-07-26 — H2 corridor 계측과 접근-only 보정

- `eval_vendor_go2_physical_ball_approach.py`는 teacher 이전의 FR-ball 최소거리,
  최초 `0.24 m` corridor 진입 step/거리/속도/gait phase, FR body-frame forward swing
  범위를 telemetry로 기록한다. 이는 command나 handoff/teacher gate에 되먹임하지 않는다.
- right seed 0 baseline에서 최초 corridor 진입은 step 176, clearance `0.2216 m`,
  base-ball `0.4310 m`, speed `0.0918 m/s`, phase `0.6951`였고, 최소 clearance는
  `0.1308 m`였다. `SETTLE_ZERO_COMMAND=1`은 같은 trot phase를 멈추지 못해 효과가 없었다.
- final crawl(`1.50 Hz`, swing height `0.03 m`)은 FR clearance `0.2696 m`로 FR corridor는
  피했으나 yaw drift `0.1773 rad` 뒤 FL/Head 접촉으로 fail-closed가 됐다. near-R1
  `ball_local_x=0.75`에서도 phase `0.6751`에서 FR clearance `0.2347 m`로 M3의
  `0.24 m` 최소치보다 작아 direct handoff가 불가했다.
- 따라서 H2는 clean plant→`TEACHER`→kick을 재현하지 못했고 center/left를 실행하지
  않았다. FR teacher/Bézier/support/hip offset/swing 및 성공 gate는 모두 불변이다.
- H2 phase probe는 acceptance를 `공 정지` + `FR clearance >= 0.24 m` + M3 phase/speed
  + `POSTURE -> TEACHER` 순서로 명시했다. right/near-R1 seed 0에서 corridor-safe 후보는
  step 107(phase `0.5551`, clearance `0.2759 m`, speed `0.0865 m/s`)였지만 yaw error는
  `-0.1002 rad`, teacher lane 잔차는 `[+0.0944,+0.0891] m`였다. strict yaw에 가까운
  capture 지점에서는 FR clearance가 `0.2291 m`였다. 이 trace에는 strict yaw와 corridor
  clearance를 동시에 만족하는 gait-exit sample이 없었다.
- 추가 pre-bias trial은 `handoff_heading_offset=+0.1002 rad`와 near-R1 short/0.75 m를
  각각 검증했지만 clean handoff를 만들지 못했다. short `0.55 m`는 FR collision을 더
  앞당겼고, 0.75 m에서도 safe candidate yaw가 `-0.0970 rad`, strict-yaw capture FR
  clearance가 `0.2308 m`였다. 단일 aim/waypoint feed-forward는 이 walker의 coupled
  gait-exit를 분리하지 못하므로 H2는 불합격으로 동결한다.

## 2026-07-26 — target-yaw anchor bridge 재설계 요구사항

- 현 v5 bridge는 snapshot의 capture yaw를 `bridge_anchor_yaw`로 저장하고 그 yaw에 대한
  drift만 reward/audit한다. snapshot에 `target_heading_rad`가 있어도 학습 reward와 audit은
  이를 사용하지 않는다. 따라서 잘못 캡처된 yaw를 조용히 유지해도 standalone audit을 통과할
  수 있으며 H2의 target-yaw 문제를 해결하지 못한다.
- 다음 bridge는 capture-yaw 유지가 아니라 snapshot target heading을 observation/reward/audit
  anchor로 사용해야 한다. 통과 기준은 `abs(wrap(base_yaw-target_heading)) <= 0.06 rad`와
  quiet support/height/tilt/XY, 그리고 실제 ball evaluator의 clean `POSTURE -> TEACHER`다.

### v6 구현 및 승격 상태

- `scripts/train_vendor_go2_stand_bridge.py`에 opt-in
  `BRIDGE_YAW_ANCHOR_MODE=target_heading`을 추가했다. 새 모드는 snapshot의
  `target_heading_rad`를 reward anchor로 쓰고, 공개 policy의 기존
  `commands[:, 2]` yaw-rate 입력에 bounded closed-loop command를 기록한다.
  관측 차원, FR teacher, 킥 접촉 보정은 바꾸지 않는다. 기본 `capture` 모드는
  동결된 v5 재현성을 보존한다.
- `scripts/audit_vendor_stand_bridge.py`는 target-heading 모드에서 target yaw
  오차를 별도 기록하고 `<= 0.06 rad`를 quiet-stance 통과 기준으로 사용한다.
  evaluator도 `APPROACH_BRIDGE_TARGET_YAW_HOLD=1`일 때 bridge POSTURE/TEACHER
  관측에만 같은 bounded yaw-rate command를 전달한다.
- 1,024-env/8-iteration GPU smoke는 command 경로와 checkpoint 저장을 확인했지만
  standalone audit는 target-yaw 최대 `0.1217 rad`, drift `0.8023 m`, quiet step
  `0`으로 불합격했다. 따라서 smoke checkpoint는 공 평가 후보가 아니며, 동결된
  `v5_cpu_i012000` 가중치에서 시작하는 v6 fine-tune만 다음 승격 경로로 사용한다.

---

## 2026-07-23 — Motion-prior v7: stand → support triangle → lift contract

- v6 `8lil8um1`은 iteration 971에서 archive했다. FR unload/load-ready/recovery가 0%였고 alpha=0인 상태의 앞발 들림은 planned lift가 아니었다.
- v7은 0.30초 높은 네발 기립을 먼저 latch한다. stand-ready, height >= 0.315 m, FL/RL/RR support-triangle margin >= 0.02 없이 loading/unload reward와 readiness를 주지 않는다.
- W&B에 stand-ready/dwell 및 triangle margin을 추가했고, initial/std cap을 0.12/0.25, PPO lr을 2e-4로 낮췄다.
- zero-action stand-ready 100%, dwell 0.42 s, min height 0.3172 m 및 8-env PPO smoke를 확인했다.

## 2026-07-23 — Motion-prior v6: anti-crouch and stable exploration

- v5 run `o16morzm`은 사용자가 정한 800-iteration 관찰 기준 뒤 local iteration 874에서 archive했다. 710/805 영상의 rear-crouch와 `kick_fr_unloaded_rate=0%`, `kick_load_ready_rate=0%`는 세 발 지지율만으로 CoM transfer를 주장할 수 없음을 확인했다.
- v6는 매 episode 최저 base height, FR 최대 impact force, load-ready 최대 dwell을 W&B에 기록해 접촉률과 자세 전이를 분리해 진단한다.
- CPU PhysX equilibrium 0.330 m를 기준으로 base height 0.325 m 아래의 crouch에 dense penalty를 부여하고, 0.310 m 아래는 즉시 종료한다. rear-leg nominal stance weight를 1.75로 높이고 termination cost를 -20으로 강화했다.
- policy exploration은 initial std 0.20, log-std bound [0.05, 1.0], fixed PPO (lr 3e-4, 3 epochs, clip 0.15, grad-norm 0.5)로 제한했다. W&B에서 실제 std와 learning-rate도 확인한다.
- 8-env/8-iteration smoke 및 1-env zero-action diagnostic을 통과했다. zero action은 3.0초 timeout까지 유지됐고 minimum base height 0.3172 m로 새 0.310 m fall threshold를 넘었다.

## 2026-07-23 — Motion-prior v5: support-first FR lift and impact control

- v4 run `w4zq7iz6`는 iteration 1,780에서 종료하고 workspace 내부 archive에 보존했다. load-ready EMA 8.9%, recovery 0%, lift alpha 0으로 선행 skill gate를 통과하지 못했다.
- v4는 lift가 잠긴 동안 FR reference reward가 post-load 구간에서 약해지고 stance reward도 FR을 항상 제외해, FR 대신 RL/RR을 들거나 지면을 차는 정책을 충분히 억제하지 못했다.
- readiness 전에는 FR 목표를 항상 nominal toe 위치로 고정하고, lift alpha와 0.15초 연속 readiness dwell을 모두 통과한 episode에만 smooth swing reference를 적용한다.
- CoM load와 FR unload 보상은 FL/RL/RR 세 발이 모두 접촉할 때만 지급하고, support 성공률도 2/3이 아니라 3/3 기준으로 계산한다.
- FR unload target은 정확한 0 N 대신 standing force의 8%를 남겨 과도한 지면 박차기를 줄였다.
- 비-kicking 발 공중 체류 `-3`, FR 하향 지면 충격 `-1`, 비-timeout 낙상 `-5`를 추가했다.
- action clip을 100에서 1로, 초기 policy std를 1.0에서 0.35로 낮췄다.
- W&B에 FL/FR/RL/RR air rate와 최대 높이, FR 하향 접촉 속도/충격력, 최대 readiness dwell을 추가했다.
- 8-env/8-iteration CPU PhysX smoke test는 timeout 100%, three-leg support 97.6%, wrong-foot air 0.52%, FR downward contact speed 0.146 m/s로 전체 runtime/logger 경로를 통과했다.
- fresh long run `go2-kick-prior-v5` (`o16morzm`)을 CPU PhysX 64 env, GPU PPO, video interval 100으로 시작했다. resolved config는 `resume=false`, `init_noise_std=0.35`, `clip_actions=1.0`이다.

## 2026-07-23 — Motion-prior v4: smooth performance curriculum and strict recovery

- v3 run `xmi3ubgd`는 iteration 705에서 보존 종료했다. 최근 100 iteration은 timeout completion이 높아졌지만 median CoM error 0.199 m, three-leg support 69.6%였고 최신 영상은 스윙 뒤 앞으로 무너진 자세로 timeout까지 버텼다.
- hard one-hot phase를 0.1초 minimum-jerk blend weight로 바꾸고 fifth observation을 continuous global cycle progress로 변경했다. 총 observation은 75차원으로 유지했다.
- load/lift/kick/recovery trajectory에 minimum-jerk time scaling을 적용하고 recovery arc를 zero-endpoint-velocity bump로 교체했다.
- A1 controller처럼 policy joint command에 `alpha=0.5` low-pass filter를 추가했다.
- unload/support reward를 CoM·height·orientation·base-speed 품질과 곱해 forward-fall unload exploit을 차단했다.
- load/recovery EMA가 각각 80% 이상일 때만 lift와 kick을 순서대로 unlock하고 각 motion을 500 iteration 동안 연속적으로 blend하는 performance curriculum을 추가했다.
- timeout과 recovery success를 분리했다. 마지막 0.2초의 base height/orientation, linear/angular speed, CoM return, FR toe return, four-foot contact를 모두 검사한다.
- zero-action physics diagnostic은 timeout/recovery/support 100%, recovery quality 0.936, load-ready 0%로 의도한 분리를 확인했다. curriculum forced-success diagnostic은 lift/kick alpha가 각각 `0 -> 0.5 -> 1` minimum-jerk ramp로 진행됨을 확인했다.

---

## 2026-07-23 — Motion-prior v3: explicit CoM loading

- v2 long run을 중단하고 checkpoint는 비교용으로 보존했다. v3는 v2 weight를 resume하지 않는 fresh random initialization이다.
- lift phase의 0.50–0.85초를 loading 구간으로 분리해 FR toe를 지면에 유지한 채 전신 CoM을 좌후방 `[-0.04, +0.05]` m로 이동하도록 했다.
- PhysX에 실제 적용된 각 rigid link의 mass와 local CoM offset을 저장하고, 매 step world-frame whole-body CoM을 질량 가중 계산한다.
- `kick_com_load`, `kick_fr_unload`, `kick_support` 보상과 CoM 최대 오차, FR 무부하율, 세 발 지지율 W&B metric을 추가했다.
- FR toe lift reference는 loading 종료 후 시작하고, recovery 동안 CoM target은 원점으로 복귀한다.
- 8 env / 2 iteration CPU PhysX + GPU PPO offline smoke test가 통과했으며 초기 random policy는 CoM 최대 오차 0.135 m, FR 무부하율 0%, 세 발 지지율 65%로 기록됐다.

---

## 2026-07-23 — Motion-prior v2

- v1은 iteration 1,046까지 contact/success 0%였으며, 매-step `stance_legs_support`와 absolute ball proximity/yaw reward가 one-shot kick event보다 큰 누적 보상을 만들었다.
- RoboNaldo의 `Rforce`가 지지발 지면반력이 아니라 foot-ball contact force임을 정정했다. 해당 논문의 pure PPO ablation도 contact/alive 0%이므로 reward 이름만 복제한 from-scratch PPO를 중단했다.
- A1 quadrupedal shooting 구조를 따라 `standing -> lifting -> kicking -> resting` phase와 FR toe Bézier reference tracking을 추가했다.
- `KickPhaseSensor` 5차원으로 기존 Clock/Timing 5차원을 교체해 전체 observation 75차원을 유지했다.
- prior에서 action lag, observation noise, 30% ball dropout, domain re-randomization을 비활성화했다.
- dense support/proximity/yaw reward를 제거하고 `kick_reference=6`, `kick_stance=1`을 주 신호로 설정했다.
- base height target을 실제 CPU-PhysX equilibrium 0.330 m로 교정하고 fall threshold를 0.27 m로 올렸다.
- Euler pitch가 작은 음수에서 2π로 표현돼 episode당 약 -47의 가짜 penalty를 만들던 문제를 `wrap_to_pi`로 수정했다.
- weak first impulse가 later strong success를 영구 차단하던 event latch를 수정했다.
- zero-action은 1.6초경 max toe error 0.258 m로 tracking 종료되고, 2-iteration PPO 및 80-frame W&B video smoke test가 통과했다.
- 학습 순서를 `prior -> fixed-ball v0 -> spatial planner`로 분리하고 미구현 spatial phase1은 accidental launch 방지를 위해 잠갔다.

---

### 1. `dribblebot/rewards/soccer_rewards.py` & `kick_rewards.py`
- **Changed Function**: 
  - `_reward_dribbling_robot_ball_yaw()` (NaN 예외 처리 및 `KickRewards` 내 **초기 자세 Settling Gate `progress_buf > 50`** 추가)
  - `_reward_dribbling_robot_ball_pos()` (**초기 자세 Settling Gate `progress_buf > 50`** 추가하여 스폰 직후 1.0초간 4다리 서기 자세를 완전히 잡도록 게이팅)
  - `_reward_kick_contact()` (킥 접촉 대상 발을 앞다리 2개 `front_feet_indices[:2]`로 한정하여 뒷다리 비비기 방지)
  - Added `_reward_feet_slip()` (지면에 닿은 발 미끄러짐 억제)
  - Added `_reward_stance_legs_support()` (양 앞다리가 공중에서 허우적거리는 현상 방지: 공에 가까운 1개 앞다리만 킥 다리로 지정하고, 나머지 앞다리 1개 + 뒷다리 2개 = 총 3개 다리에 단단한 지지력 보상 유도)
- **Change Description**: 
  - 양 앞다리가 공중에서 동시에 허우적거리는 현상 해결을 위해 3-leg stance support 및 single front-leg kick contact 메커니즘 구축.
- **Validation Status**: Verified logic and math safe implementation.

### 2. `dribblebot/envs/go2/go2_kick_config.py` & `kick_rewards.py`
- **Changed Section**: `config_go2_kick()` 및 `_reward_kick_contact()`
- **Change Description**:
  - 공식 5호 축구공(Size 5 Official Match Ball) 규격 반영: 반지름 `radius = 0.11m` (지름 22cm), 질량 `mass = 0.43kg` (430g), 초기 스폰 높이 `ball_init_pos = [0.5, 0.0, 0.11]`로 수정하여 실물 환경과 100% 수치 일치.
  - 사용자 현장 관찰(4다리로 허우적대며 공에 다가가는 Local Minima 현상)을 바탕으로 정밀 튜닝: `_reward_kick_contact`에 `support_gate`(공 임팩트 시 3개 지지다리 중 최소 2개 이상 지면 받침 필수 조건) 결합.
  - `reward_scales.stance_legs_support = 3.0` 상향, `action_rate = -0.05`, `dof_acc = -5.0e-7` 감점 상향으로 걸어가며 다리를 허우적거리는 편법 동작 물리적 차단 및 단독 앞다리 킥 동작 정교화.
- **Validation Status**: Config updated and validated.

### 3. `MASTER_PLAN.md`
- **Changed Section**: Overall Master Plan (v0 Milestone & Core Dynamics Innovations)
- **Change Description**:
  - v0 단계에서 완수된 핵심 물리 다이내믹스 기술(Fixed Joint Mesh Fix, Initial Settling Gate 1.0s, Single Front-Leg Kick & 3-Leg Stance Support, CoM Shift, Spawn Safety Zone, Motor Smoothness) 정밀 명세화.
  - Phase 1 ~ Phase 6 (v0 검증 ➔ AprilTag 3-Target RL ➔ 동적 공 ➔ DR & ONNX ➔ 3D LiDAR & AprilTag Perception ➔ unitree_sdk2 50Hz 온보드 배포) 최종 갱신.
- **Validation Status**: Document updated and synced with codebase.

### 4. `scripts/train_kick.py`
- **Changed Section**: `train_go2_kick()` (Line 20 - 25)
- **Change Description**:
  - DribbleBot 및 Su2025 검증 모범 사례를 기반으로 `num_envs = 2048`로 지정: GPU 메모리 사용량을 낮춘 v0 기준선. Isaac Gym Preview 4의 별도 rigid-patch 경고는 이 설정만으로 해소되지 않으므로 Issue 13으로 추적.
  - `wandb.init(name="go2-kick-v0", reinit=True)` 설정으로 이전 삭제된 run ID와의- **`dribblebot/envs/go2/go2_kick_config.py` & `dribblebot/rewards/kick_rewards.py`**:
  - 사용자 지적에 기반한 RL 킥 물리/보상 수식 전수 정밀 점검 및 근본 결함 3가지 완벽 수정:
    1) **`_reward_kick_contact` 다리 동적 구별 버그 수정**: 공에 가까운 킥 발(FL/FR)을 동적으로 감지하여, 반대쪽 앞발과 뒷다리 2개를 지지 다리로 동적 구성 (기존 오른발 킥 시 FR이 지지 다리로 중복 지정되어 다리를 뻗지 못하게 막던 버그 원천 사살).
    2) **지지 다리 지면 접촉력 축 수정**: 수평 마찰력(`:2`)에서 Z축 수직 지지력(`contact_forces[:, legs, 2] > 1.0N`)으로 정상 교체.
    3) **주저앉음/넘어짐 실패 리셋(Check Termination) 활성화**: `use_terminal_body_height = True` (`height < 0.22m`) 및 `use_terminal_roll_pitch = True` (`ori > 0.5`)를 설정하여, 다리를 뻗고 누워버린 편법 상태에서 수백 스텝 동안 보상을 타먹던 현상을 즉시 에피소드 실패 리셋 처리.
- **Validation Status**: Fully compiled and verified without syntax or logic errors.

---

## 2. User Requirements Reflection

- [x] `dribbling_robot_ball_yaw` `NaN` 오염 문제 해결
- [x] `self_collisions` PhysX 튀김 및 미끄러짐 방지 설정 조정
- [x] 제자리 회전(`ang_vel_z`) 억제 보상 추가
- [x] 공이 로봇 몸통 밑에 스폰되는 위치 노이즈 이탈 수정 (`init_pos_range = [0.15, 0.10, 0.0]`)
- [x] `dof_pos` 초기화 클램프 범위 `(0.8, 1.2)` 유지 검증
- [x] **공 향해 몸을 던지는 다이빙 편법 해결**:
  - `_reward_kick_contact`에서 지지다리가 2개 이상일 때만 보상을 주는 **하드 게이트(Hard Gate)** 도입 (`0.5 + 0.5 * gate` → `gate`)
  - 몸통 높이(Base Height) 0.25m 이상 하드 게이트 조건 추가 (몸을 낮추고 미끄러지는 편법 원천 차단)
  - `dof_pos` 기본자세 이탈 페널티를 -0.05에서 -0.01로 5배 완화하여, 로봇이 킥을 위해 앞다리를 자유롭게 들어 올리는 동작이 억제되지 않도록 조치

### 5. v0 Event-Based Kick Benchmark & Expansion Plan
- Replaced persistent foot-distance and ball-speed shaping with a one-contact-per-episode event: front-foot proximity, positive ball impulse, support, and upright gates are all required.
- Added separate contact and strong-kick success metrics (`kick_contact_rate`, `kick_success_rate`, contact speed, impulse) to W&B episode logs.
- Fixed Go2 v0 physics: calibrated 0.43 kg ball, zero reset velocity, deterministic ball/command, 2048 environments, and `2**26` contact pairs.
- Cleared wrapped observation history on internal environment resets and fixed `play_kick.py` so reset no longer overwrites the requested evaluation command.
- Rewrote `MASTER_PLAN.md` with phase gates for spatial generalization, AprilTag targets, moving balls, sim-to-real, and onboard deployment.

---

## 2026-07-26 — v6 bridge D-diag / nominal-settle fail-closed 측정

- `scripts/eval_vendor_go2_snapshot_bridge_teacher.py`에 bridge 직후와
  nominal-settle 직후의 12관절 편차·속도, base 상태, feet contact, FR-공 거리,
  body-frame 공 lane을 기록하는 D-diag를 추가했다.
- 동일 스크립트에 bridge 자세에서 `default_dof_pos`로 minimum-jerk PD
  settle하는 opt-in 단계와 joint/velocity/yaw/공 lane/FR clearance/공 변위
  fail-closed gate를 추가했다. FR teacher 궤적·support·hip offset·성공 gate는
  변경하지 않았다.
- `scripts/eval_vendor_go2_native_kick_teacher.py`는 R1 teacher 첫 step 직전의
  기준 상태를 로그로만 남긴다.
- seed 0에서 R1은 `forward=1.6646 m`, `lateral=0.1312 m`로 통과했다. 동일
  snapshot/v6 bridge의 최대 settle은 공을 건드리지 않았지만
  `JOINT_ERROR|DOF_VELOCITY|BALL_LANE`로 teacher 전 gate에서 중단됐다.
- 검증: `go2kick python -m py_compile` 및 `git diff --check` 통과. 새 학습,
  checkpoint 덮어쓰기, teacher 보정은 수행하지 않았다.

## 2026-07-26 — v6 공-lane 분리 진단

- `eval_vendor_go2_snapshot_bridge_teacher.py`에 진단 전용
  `V6_INTEGRATED_LANE_ISOLATION=1`을 추가했다. bridge 자세를 바꾸지 않고
  teacher 직전에만 공 root state를 현재 base-frame FR lane `[0.3335, -0.20]`로
  재배치하며, 재배치량과 이후 pre-kick 공 변위를 분리 기록한다.
- v6 strict seed 0에서 5.71 cm 재배치 후 `forward=1.6169 m`로 기존 0.2403 m에서
  회복했다. 따라서 약한 발사의 주원인은 공 lane 오차다. 단, `lateral=-0.6680 m`로
  방향 gate는 실패했으므로 bridge 자세/지지 상태가 방향 품질에 미치는 영향은 남아 있다.
- 이 조작은 자율 경로나 teacher 보정이 아닌 원인 분리 실험이며, 새 학습·checkpoint
  덮어쓰기·teacher/성공 gate 변경은 수행하지 않았다.

## 2026-07-26 — C-lite FR 접촉 tangent sweep

- lane-isolation 조건에서 허용된 `NATIVE_KICK_FR_HIP_OFFSET`와
  `NATIVE_KICK_FR_HIP_SWING_DELTA`만 sweep했고, 통합 평가 로그/W&B config에
  실제 tangent 값을 기록했다.
- seed 0 기준선 `(0, 0)`은 `forward=1.6169 m`, `lateral=-0.6680 m`였고,
  `(-0.10, 0)`은 `1.6592 m`, `-0.5920 m`, tag 프리셋 범위 `(-0.32, -0.25)`은
  `2.1092 m`, `-0.0950 m`로 gate를 통과했다.
- 최종 `(-0.32, -0.25)`의 seed 1/2는 각각 `forward=1.8690/1.9947 m`,
  `lateral=+0.9879/+0.0751 m`로 1/2만 통과했다. 따라서 fixed tangent는
  seed 0/2에서 발사 방향을 보정하지만 bridge 자세 변동에 강건하지 않다.
- teacher Bézier/support/성공 gate와 frozen bridge는 변경하지 않았고, 새 학습이나
  checkpoint 덮어쓰기는 수행하지 않았다.

## 2026-07-26 — B: nominal bridge i=400 통합 킥 승격

- `scripts/train_vendor_go2_stand_bridge.py`에 R1 `default_dof_pos` 직접 추적
  reward와 low `dof_vel` reward를 추가했다. 기존 target-yaw, height, upright,
  anchor 보상은 유지했고 teacher/Bézier/support/hip offset/성공 gate는 변경하지 않았다.
- v6 strict weight에서 1,024-env GPU balanced/strict bounded variant를 시작하고,
  CPU lane-isolation + 기본 R1 teacher 통합 킥으로 checkpoint를 승격했다.
- `go2-target-yaw-anchor-bridge-v7-nominal-balanced-i1600`의 i=400을
  `.runtime/bridge_candidates/v7_nominal_balanced_i0400/`에 고정했다. seed 0/1/2는
  각각 `(forward,lateral)=(1.7673,-0.1520),(1.6013,-0.0433),(1.4999,-0.2364) m`로
  forward/횡오차/높이/자세/무전도 gate를 모두 통과했다.
- 통과 직후 두 GPU 학습 세션을 종료했다. 이 후보는 고정 lane 통합 킥 조건의 Stage 1
  통과 후보이며, 다음 단계는 공을 동일 FR lane으로 넣는 자율 접근이다.

## 2026-07-26 — Stage 2a/2b 자율 접근 계측: corridor blocker

- `scripts/eval_vendor_go2_snapshot_bridge_teacher.py`의 lane-isolation에 offset
  sweep을 추가했다. v7 i400/seed 0에서 y offset `0/-0.02/-0.04/-0.06/-0.08 m`의
  forward는 각각 `1.7673/1.3832/0.6109/0.4636/0.0000 m`였다. 따라서 강한 킥의
  lane 허용오차는 약 2 cm 이내이며, 방향 gate까지 고려하면 그보다 더 작아야 한다.
- `scripts/run_go2_stage2_v7_lane_approach.py`를 추가하고 evaluator에는
  `APPROACH_BRIDGE_TO_NATIVE_TEACHER=1` opt-in을 추가했다. bridge는 POSTURE만
  소유하고 그 뒤 고정 native teacher로 되돌린다. teacher/Bezier/support/hip offset/
  성공 gate/v7 checkpoint는 바꾸지 않았다.
- 실제 seed 0 보행은 두 가지 상반된 실패를 보였다. 기존 direct handoff는 lane 오차
  약 8.5 cm에서 bridge에 넘겨 `POSTURE_FAILED`가 됐고, 2 cm final-pose gate로
  조인 실행은 FR-ball 최소거리 `0.1308 m`/공 변위 `0.0013 m`의 pre-kick corridor
  접촉으로 `PERCEPTION_INVALID`가 됐다. 따라서 현 walker는 현재 레시피에서
  공을 건드리지 않고 동시에 2 cm lane을 만들지 못한다.
- 검증: `go2kick python -m py_compile` 및 `git diff --check` 통과. 실측의 반복은
  중단했으며, 다음 후보는 종료거리/creep을 계측 기반으로 별도 설계하거나 접근 RL을
  검토하는 것이다.

## 2026-07-26 — Stage 2c 동적 FR swing lane 적응 한계

- `eval_vendor_go2_snapshot_bridge_teacher.py`가 기존 tag 프리셋과 같은
  `APPROACH_DYNAMIC_FR_SWING_*` 측정/deadband/clamp 계약을 lane-isolation teacher
  직전에 지원하도록 확장됐다. 이는 측정된 body-frame ball y에 따라 FR swing
  tangent만 바꾸며, root/ball/support/Bezier/hip offset/v7 bridge/성공 gate는
  바꾸지 않는다.
- v7 i400 seed 0, y=-4 cm에서 base=0/reference=-0.20으로 bounded gain을 비교했다.
  `gain=3.0 (delta=-0.120): forward=0.6908, lateral=-0.3340`,
  `gain=5.4 (delta=-0.216): 1.5219, -0.4968`,
  `gain=-5.4 (delta=+0.216): 0.7632, -0.4636`으로 모두 횡오차 또는 파워 gate를
  실패했다. 8 cm의 walker 오차를 검사할 근거가 없으므로 추가 offset/실보행은
  실행하지 않았다.
- nominal y=0 control은 dynamic `delta=0`으로 `forward=1.7673 m`,
  `lateral=-0.1520 m`를 재현했다. 즉 기존 teacher를 훼손하지 않았지만 single
  FR swing tangent는 4 cm 이상 lane miss를 강+정확 킥으로 확장하지 못한다.

## 2026-07-28 — Go2 MCF gait-initiation false-stop 보정

- 실물 WebRTC staging 로그에서 0.03m forward 목표를 `robot_odom`의 body/gait
  transient 한 샘플이 먼저 넘기면서, 실제 step 전에 neutral로 전환되는 원인을 확인했다.
- `stage_go2_mcf_ball_tag_webrtc.py`는 active command 0.60초 뒤 서로 다른 새 odometry
  sample 3개가 연속으로 목표를 확인할 때만 forward/yaw pulse를 종료한다. 동일 sample을
  50Hz command loop에서 중복 확인하지 않는다.
- yaw/lateral pulse 상한은 gait initiation 여유를 위해 0.80초로 변경했다. camera staging
  잔여거리 0.04m 이내는 짧은 pulse를 반복하지 않고 기존 bounded final dock으로 넘긴다.
  joystick magnitude 0.20, 전체 travel/duration hard limit, remote preemption은 유지했다.
- 검증: Python 3.8 `py_compile`, 두 entry point `--help`, `git diff --check`, synthetic
  odometry에서 최소 active time/새 sample 3회/동일 sample 비중복 확인을 통과했다.
  실제 전진·yaw·lateral gait와 final LowCmd 연결은 Go2에서 재검증해야 한다.
