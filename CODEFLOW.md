# CODEFLOW.md

Summary of system architecture and execution flow for Go2 Kick RL training.

---

## 1. Overview & Entry Points

- **Main Script**: `scripts/train_kick.py`
- **Environment Config**: `dribblebot/envs/go2/go2_kick_config.py` (`config_go2_kick`)
- **Robot Asset Class**: `dribblebot/robots/go2.py` (`Go2`)
- **Reward Container**: `dribblebot/rewards/kick_rewards.py` (`KickRewards` extends `SoccerRewards`)

---

## 2. Execution & Call Flow

```
scripts/train_kick.py
 └── train_go2_kick()
      ├── config_go2_kick(Cnfg) -> go2_kick_config.py
      ├── runner = Runner(env, ...) -> dribblebot_learn/ppo_cse/
      └── runner.learn()
           └── env.step(actions) -> LeggedRobot.step()
                ├── physics_step -> IsaacGym Sim
                ├── post_physics_step()
                └── _prepare_reward_function() / KickRewards.get_reward()
                     ├── _reward_dribbling_robot_ball_pos()
                     ├── _reward_dribbling_robot_ball_yaw() [NaN Safe Clamped]
                     ├── _reward_ang_vel_z() [Spin suppression]
                     ├── _reward_kicking_ball_vel()
                     ├── _reward_kick_contact()
                     └── _reward_kick_hold()
```

---

## 3. Key Interaction & Dependencies

- **Go2 URDF**: `/root/Desktop/workspace/expo/unitree_rl_gym/resources/robots/go2/urdf/go2.urdf`
- **Fixed Joint Handling**: `collapse_fixed_joints = False` prevents mesh transform distortion.
- **Sensors**: `ObjectSensor` + `ObjectVelocitySensor` provide ball position/velocity to Policy & Critic.
