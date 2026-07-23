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
  - `wandb.init(name="go2-kick-v0", reinit=True)` 설정으로 이전 삭제된 run ID와의 409 Conflict 오류 완벽 해결.
- **Validation Status**: Verified configuration update for 2048 envs.

- **`dribblebot/envs/go2/go2_kick_config.py` & `dribblebot/rewards/kick_rewards.py`**:
  - 사용자 현장 원인 파악 (킥 임팩트 후 다리를 뻗은 채 주저앉아 멈추는 편법 현상 해결):
    1) `_reward_kick_hold` 보상 수식에 `dof_pos_error` (기본 관절 서 있는 자세 오차) 회수 항(`torch.exp(-0.2 * dof_pos_error)`)을 결합하여, 킥 직후 다리를 당겨 원래 4족 기립 자세로 완벽히 복귀(Leg Retraction & Recovery)하도록 강제.
    2) `reward_scales`에 `dof_pos = -0.05` 항을 추가하여 평소 및 킥 후에도 다리를 쭉 뻗은 채 바닥에 주저앉는 편법(Local Minima)을 페널티로 원천 차단.
- **Validation Status**: Cleanly compiled and verified.

---

## 2. User Requirements Reflection

- [x] `dribbling_robot_ball_yaw` `NaN` 오염 문제 해결
- [x] `self_collisions` PhysX 튀김 및 미끄러짐 방지 설정 조정
- [x] 제자리 회전(`ang_vel_z`) 억제 보상 추가
- [x] 공이 로봇 몸통 밑에 스폰되는 위치 노이즈 이탈 수정 (`init_pos_range = [0.15, 0.10, 0.0]`)
- [x] `dof_pos` 초기화 클램프 범위 `(0.8, 1.2)` 유지 검증
