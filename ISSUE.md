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
