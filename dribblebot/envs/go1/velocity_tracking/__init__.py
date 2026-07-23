from isaacgym import gymutil, gymapi
import torch
from params_proto import Meta
from typing import Union

from dribblebot.envs.base.legged_robot import LeggedRobot
from dribblebot.envs.base.legged_robot_config import Cfg


class VelocityTrackingEasyEnv(LeggedRobot):
    def __init__(self, sim_device, headless, num_envs=None, prone=False, deploy=False,
                 cfg: Cfg = None, eval_cfg: Cfg = None, initial_dynamics_dict=None, physics_engine="SIM_PHYSX"):

        if num_envs is not None:
            cfg.env.num_envs = num_envs

        sim_params = gymapi.SimParams()
        gymutil.parse_sim_config(vars(cfg.sim), sim_params)
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless, eval_cfg, initial_dynamics_dict)


    def step(self, actions):
        self.obs_buf, self.privileged_obs_buf, self.rew_buf, self.reset_buf, self.extras = super().step(actions)

        self.foot_positions = self.rigid_body_state.view(self.num_envs, self.num_bodies, 13)[:, self.feet_indices,
                               0:3]

        self.extras.update({
            "privileged_obs": self.privileged_obs_buf,
            "joint_pos": self.dof_pos.cpu().numpy(),
            "joint_vel": self.dof_vel.cpu().numpy(),
            "joint_pos_target": self.joint_pos_target.cpu().detach().numpy(),
            "joint_vel_target": torch.zeros(12),
            "body_linear_vel": self.base_lin_vel.cpu().detach().numpy(),
            "body_angular_vel": self.base_ang_vel.cpu().detach().numpy(),
            "body_linear_vel_cmd": self.commands.cpu().numpy()[:, 0:2],
            "body_angular_vel_cmd": self.commands.cpu().numpy()[:, 2:],
            "contact_states": (self.contact_forces[:, self.feet_indices, 2] > 1.).detach().cpu().numpy().copy(),
            "foot_positions": (self.foot_positions).detach().cpu().numpy().copy(),
            "body_pos": self.root_states[:, 0:3].detach().cpu().numpy(),
            "torques": self.torques.detach().cpu().numpy()
        })

        return self.obs_buf, self.rew_buf, self.reset_buf, self.extras

    def update_curriculum(self, it):
        """
        강화학습 진행(iteration)에 따라 난이도와 도메인 무작위화를 순차적으로 상향 조절하는 적응형 커리큘럼
        """
        # 1. 목표 공 속도: Iteration 0 (1.0m/s) -> Iteration 2000 (3.0m/s)
        target_vel_alpha = min(1.0, it / 2000.0)
        self.cfg.rewards.kick_vel_target = 1.0 + 2.0 * target_vel_alpha

        # 2. 지면 마찰력 무작위화: Iteration 0 [0.8, 1.0] -> Iteration 3000 [0.3, 1.5]
        fric_alpha = min(1.0, it / 3000.0)
        low_fric = 0.8 - 0.5 * fric_alpha
        high_fric = 1.0 + 0.5 * fric_alpha
        self.cfg.domain_rand.friction_range = [low_fric, high_fric]

        # 3. 로봇 질량 노이즈 무작위화: Iteration 0 [0, 0]kg -> Iteration 3000 [-1.0, +2.0]kg
        mass_alpha = min(1.0, it / 3000.0)
        self.cfg.domain_rand.added_mass_range = [-1.0 * mass_alpha, 2.0 * mass_alpha]

        # 4. 공 스폰 거리 난이도: Iteration 0 (0.4m) -> Iteration 1500 (1.0m 범위 확대)
        spawn_alpha = min(1.0, it / 1500.0)
        max_x_spawn = 0.40 + 0.60 * spawn_alpha
        self.cfg.ball.init_pos_range = [max_x_spawn, 0.30, 0.0]

    def reset(self):
        self.reset_idx(torch.arange(self.num_envs, device=self.device))
        obs, _, _, _ = self.step(torch.zeros(self.num_envs, self.num_actions, device=self.device, requires_grad=False))
        return obs

