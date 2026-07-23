# ISSUE.md

Record of all issues encountered and resolved during the Go2 Kick RL debugging task.

---

## Issue 1: `rew_dribbling_robot_ball_yaw` NaN Output in Wandb Summary

- **Problem Description**: Wandb summary log emitted `rew_dribbling_robot_ball_yaw: nan`, causing loss and advantage gradient corruption.
- **Root Cause**: In `SoccerRewards._reward_dribbling_robot_ball_yaw()`, `unit_command_vel` was calculated by dividing `commands[:, :2]` by its norm without checking for 0. When command velocity was zero (`[0.0, 0.0]`), division by zero (`0/0`) generated `NaN`.
- **Resolution**: Added `clamp(min=1e-6)` and `cmd_norm < 1e-4` check (`zero_cmd_mask`) in `soccer_rewards.py`.
- **Status**: Resolved.

---

## Issue 2: Robot Excessive Sliding & Spinning Drift (Local Optimum)

- **Problem Description**: Robot drifted >1m and continuously spun (yaw rotation) instead of maintaining a stable posture before kicking.
- **Root Cause**: 
  1. `dribbling_robot_ball_pos` and `yaw` rewards were high, but velocity suppression penalties (`ang_vel_z`, `lin_vel_z`, `ang_vel_xy`) were disabled (0.0). The RL policy exploited body sliding/spinning as an easy local optimum over complex leg locomotion.
  2. `self_collisions = 0` enabled inter-link penetration in PhysX for Go2's narrow joint gaps, causing unnatural contact impulse forces.
- **Resolution**: 
  1. Updated `self_collisions = 1` in `go2_kick_config.py`.
  2. Added `_reward_ang_vel_z()` helper and set `ang_vel_z = -0.1`, `lin_vel_z = -2.0`, `ang_vel_xy = -0.05` in `go2_kick_config.py`.
- **Status**: Resolved.

---

## Issue 3: URDF Visual Rendering / Collapse Fixed Joints Issue

- **Problem Description**: Go2 robot rendered upside down/flattened with primitive collision cylinders/spheres exposed.
- **Root Cause**: `collapse_fixed_joints = True` in Isaac Gym causes relative transform offset calculation bugs when collapsing ROS fixed joints (Head, Foot, Calflower).
- **Resolution**: Set `collapse_fixed_joints = False` in `go2_kick_config.py`.
- **Status**: Resolved.

---

## Issue 4: Robot Head Diving / Planting Exploit

- **Problem Description**: Robot dived its head onto the ground and planted its front face to push/kick the ball while losing balance.
- **Root Cause**: `terminate_after_contacts_on` only included `"base"`, so ground contact on `"Head_upper"` or `"Head_lower"` did not trigger episode termination. The policy exploited head planting to gain contact/kick rewards without maintaining upright balance.
- **Resolution**: 
  1. Updated `terminate_after_contacts_on = ["base", "Head_upper", "Head_lower"]` in `go2_kick_config.py`.
  2. Set `reward_scales.base_height = -2.0` to enforce height target (0.34m).
- **Status**: Resolved.

---

## Issue 5: Segmentation Fault (Core Dumped) on 4096 / 8192 Environments

- **Problem Description**: Increasing `num_envs` to 4096 / 8192 caused PhysX CUDA BroadPhase GPU memory allocation error: `the application need to increase the PxgDynamicsMemoryConfig::foundLostPairsCapacity parameter to 33558528, otherwise the simulation will miss interactions` resulting in Segmentation Fault.
- **Root Cause**: Ball rigid bodies introduce multiple contact pairs (Robot-Terrain, Robot-Ball, Ball-Terrain, Self-collisions). `2**25` (33,554,432) fell slightly short of PhysX required capacity `33,558,528` (4,096 difference).
- **Resolution**: Updated `go2_kick_config.py`:
  1. `Cnfg.sim.physx.max_gpu_contact_pairs = 2 ** 26` (67,108,864, well exceeding 33,558,528 requirement).
  2. `Cnfg.sim.physx.default_buffer_size_multiplier = 10`.
- **Status**: Resolved.

---

## Issue 6: Posture Collapse / Laying Down after Kick (Exploitation of Hold Reward)

- **Problem Description**: Robot performed kick, then collapsed/laid down with front leg extended onto the floor to stay still.
- **Root Cause**: `_reward_kick_hold` only rewarded low base linear velocity (`v_base ≈ 0`). The robot found a local minimum by laying down on the floor to remain still instead of staying upright.
- **Resolution**:
  1. Updated `_reward_kick_hold` to include posture recovery penalty `exp(-0.2 * dof_pos_error)`.
  2. Enforced terminal fall resets in `go2_kick_config.py` (`use_terminal_body_height = True` for height < 0.22m, `use_terminal_roll_pitch = True` for tilt > 0.50 rad).
- **Status**: Resolved.

---

## Issue 7: Forward Diving / Lunging Exploit into Ball (Exploitation of Contact Reward)

- **Problem Description**: Robot lunged/dived forward onto its stomach to bring its front legs close to the ball rather than standing and kicking.
- **Root Cause**:
  1. `_reward_kick_contact()` used soft gating `(0.5 + 0.5 * support_gate)`, giving 50% reward even when airborne/diving.
  2. `dof_pos = -0.05` penalty punished lifting front legs for a kick, favoring diving over leg swinging.
- **Resolution**:
  1. Updated `_reward_kick_contact()` to use strict hard gating (`support_gate * height_gate`), requiring ≥2 support legs with Z-force > 1.0N and base height > 0.25m.
  2. Reduced `dof_pos` penalty to `-0.01` so leg lifting for kicks is not penalized.
- **Status**: Resolved.

---

## Issue 8: Knee (Calf/Thigh) Ground Contact Exploit

- **Problem Description**: Robot crouched low and rested its knees/calves (`thigh` / `calf`) on the ground to maintain stability instead of standing upright on its foot pads (`foot`).
- **Root Cause**: `terminate_after_contacts_on` only included `["base", "Head_upper", "Head_lower"]`. Contact on `thigh` and `calf` only incurred mild collision penalties without triggering episode resets, making knee-resting an easy stability shortcut.
- **Resolution**: Updated `terminate_after_contacts_on = ["base", "thigh", "calf", "Head_upper", "Head_lower"]` in `go2_kick_config.py`. Any ground contact on knees/calves now immediately terminates and resets the episode, forcing the policy to stand strictly on its foot pads (`foot`).
- **Status**: Resolved.
