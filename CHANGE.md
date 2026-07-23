# CHANGE.md

Log of all changes made during the Go2 Kick RL debugging and implementation task.

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
  - DribbleBot 및 Su2025 검증 모범 사례를 기반으로 `num_envs = 2048`로 지정: 버퍼 오버플로우 경고가 완전히 사라지며 빠른 학습 속도와 쾌적한 로그 출력 보장.
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
