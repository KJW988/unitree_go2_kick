import torch
import numpy as np
from dribblebot.utils.math_utils import quat_apply_yaw, wrap_to_pi, get_scale_shift
from isaacgym.torch_utils import *
from .rewards import Rewards

class SoccerRewards(Rewards):
    def __init__(self, env):
        self.env = env

    def load_env(self, env):
        self.env = env

    def _reward_orientation(self):
        # Penalize non flat base orientation
        return torch.sum(torch.square(self.env.projected_gravity[:, :2]), dim=1)

    def _reward_torques(self):
        # Penalize torques
        return torch.sum(torch.square(self.env.torques), dim=1)

    def _reward_dof_vel(self):
        # Penalize dof velocities
        # k_qd = -6e-4
        return torch.sum(torch.square(self.env.dof_vel), dim=1)

    def _reward_dof_acc(self):
        # Penalize dof accelerations
        return torch.sum(torch.square((self.env.last_dof_vel - self.env.dof_vel) / self.env.dt), dim=1)

    def _reward_collision(self):
        # Penalize collisions on selected bodies
        return torch.sum(1. * (torch.norm(self.env.contact_forces[:, self.env.penalised_contact_indices, :], dim=-1) > 0.1),
                         dim=1)

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.env.last_actions - self.env.actions), dim=1)
    
    def _reward_tracking_contacts_shaped_force(self):
        foot_forces = torch.norm(self.env.contact_forces[:, self.env.feet_indices, :], dim=-1)
        desired_contact = self.env.desired_contact_states

        reward = 0
        for i in range(4):
            reward += - (1 - desired_contact[:, i]) * (
                        1 - torch.exp(-1 * foot_forces[:, i] ** 2 / self.env.cfg.rewards.gait_force_sigma))
        return reward / 4

    def _reward_tracking_contacts_shaped_vel(self):
        foot_velocities = torch.norm(self.env.foot_velocities, dim=2).view(self.env.num_envs, -1)
        desired_contact = self.env.desired_contact_states
        reward = 0
        for i in range(4):
            reward += - (desired_contact[:, i] * (
                        1 - torch.exp(-1 * foot_velocities[:, i] ** 2 / self.env.cfg.rewards.gait_vel_sigma)))
        return reward / 4

    def _reward_dof_pos_limits(self):
        # Penalize dof positions too close to the limit
        out_of_limits = -(self.env.dof_pos - self.env.dof_pos_limits[:, 0]).clip(max=0.)  # lower limit
        out_of_limits += (self.env.dof_pos - self.env.dof_pos_limits[:, 1]).clip(min=0.)
        return torch.sum(out_of_limits, dim=1)

    def _reward_dof_pos(self):
        # Penalize dof positions
        # k_q = -0.75
        return torch.sum(torch.square(self.env.dof_pos - self.env.default_dof_pos), dim=1)

    def _reward_action_smoothness_1(self):
        # Penalize changes in actions
        # k_s1 =-2.5
        diff = torch.square(self.env.joint_pos_target - self.env.last_joint_pos_target)
        diff = diff * (self.env.last_actions[:,:12] != 0)  # ignore first step
        return torch.sum(diff, dim=1)

    def _reward_action_smoothness_2(self):
        # Penalize changes in actions
        # k_s2 = -1.2
        diff = torch.square(self.env.joint_pos_target - 2 * self.env.last_joint_pos_target + self.env.last_last_joint_pos_target)
        diff = diff * (self.env.last_actions[:,:12] != 0)  # ignore first step
        diff = diff * (self.env.last_last_actions[:,:12] != 0)  # ignore second step
        return torch.sum(diff, dim=1)

    # encourage robot velocity align vector from robot body to ball
    # r_cv
    def _reward_dribbling_robot_ball_vel(self):
        FR_shoulder_idx = self.env.gym.find_actor_rigid_body_handle(self.env.envs[0], self.env.robot_actor_handles[0], "FR_hip")
        if FR_shoulder_idx == -1:
            FR_shoulder_idx = self.env.gym.find_actor_rigid_body_handle(self.env.envs[0], self.env.robot_actor_handles[0], "FR_thigh_shoulder")
        FR_HIP_positions = quat_rotate_inverse(self.env.base_quat, self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:,FR_shoulder_idx,0:3].view(self.env.num_envs,3)-self.env.base_pos)
        FR_HIP_velocities = quat_rotate_inverse(self.env.base_quat, self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:,FR_shoulder_idx,7:10].view(self.env.num_envs,3))
        
        delta_dribbling_robot_ball_vel = 1.0
        robot_ball_vec = self.env.object_local_pos[:,0:2] - FR_HIP_positions[:,0:2]
        d_robot_ball=robot_ball_vec / torch.norm(robot_ball_vec, dim=-1).unsqueeze(dim=-1)
        ball_robot_velocity_projection = torch.norm(self.env.commands[:,:2], dim=-1) - torch.sum(d_robot_ball * FR_HIP_velocities[:,0:2], dim=-1) # set approaching speed to velocity command
        velocity_concatenation = torch.cat((torch.zeros(self.env.num_envs,1, device=self.env.device), ball_robot_velocity_projection.unsqueeze(dim=-1)), dim=-1)
        rew_dribbling_robot_ball_vel=torch.exp(-delta_dribbling_robot_ball_vel* torch.pow(torch.max(velocity_concatenation,dim=-1).values, 2) )
        return rew_dribbling_robot_ball_vel

    # encourage robot near ball
    # r_cp
    def _reward_dribbling_robot_ball_pos(self):

        FR_shoulder_idx = self.env.gym.find_actor_rigid_body_handle(self.env.envs[0], self.env.robot_actor_handles[0], "FR_hip")
        if FR_shoulder_idx == -1:
            FR_shoulder_idx = self.env.gym.find_actor_rigid_body_handle(self.env.envs[0], self.env.robot_actor_handles[0], "FR_thigh_shoulder")
        FR_HIP_positions = quat_rotate_inverse(self.env.base_quat, self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:,FR_shoulder_idx,0:3].view(self.env.num_envs,3)-self.env.base_pos)

        delta_dribbling_robot_ball_pos = 4.0
        rew_dribbling_robot_ball_pos = torch.exp(-delta_dribbling_robot_ball_pos * torch.pow(torch.norm(self.env.object_local_pos - FR_HIP_positions, dim=-1), 2) )
        return rew_dribbling_robot_ball_pos 

    # encourage ball vel align with unit vector between ball target and ball current position
    # r^bv
    def _reward_dribbling_ball_vel(self):
        # target velocity is command input
        lin_vel_error = torch.sum(torch.square(self.env.commands[:, :2] - self.env.object_lin_vel[:, :2]), dim=1)
        # rew_dribbling_ball_vel = torch.exp(-lin_vel_error / (self.env.cfg.rewards.tracking_sigma*2))
        return torch.exp(-lin_vel_error / (self.env.cfg.rewards.tracking_sigma*2))
        
    def _reward_dribbling_robot_ball_yaw(self):
        robot_ball_vec = self.env.object_pos_world_frame[:, 0:2] - self.env.base_pos[:, 0:2]
        robot_ball_norm = torch.norm(robot_ball_vec, dim=-1, keepdim=True).clamp(min=1e-6)
        d_robot_ball = robot_ball_vec / robot_ball_norm

        cmd_norm = torch.norm(self.env.commands[:, :2], dim=-1, keepdim=True)
        unit_command_vel = self.env.commands[:, :2] / cmd_norm.clamp(min=1e-6)
        robot_ball_cmd_yaw_error = torch.norm(unit_command_vel, dim=-1) - torch.sum(d_robot_ball * unit_command_vel, dim=-1)
        # commands[:2]가 0인 경우 zero_cmd_mask로 처리하여 NaN 방지 예외 처리
        zero_cmd_mask = (cmd_norm.squeeze(-1) < 1e-4)
        robot_ball_cmd_yaw_error = torch.where(zero_cmd_mask, torch.zeros_like(robot_ball_cmd_yaw_error), robot_ball_cmd_yaw_error)

        # robot ball vector align with body yaw angle
        roll, pitch, yaw = get_euler_xyz(self.env.base_quat)
        body_yaw_vec = torch.zeros(self.env.num_envs, 2, device=self.env.device)
        body_yaw_vec[:, 0] = torch.cos(yaw)
        body_yaw_vec[:, 1] = torch.sin(yaw)
        robot_ball_body_yaw_error = torch.norm(body_yaw_vec, dim=-1) - torch.sum(d_robot_ball * body_yaw_vec, dim=-1)
        delta_dribbling_robot_ball_cmd_yaw = 2.0
        rew_dribbling_robot_ball_yaw = torch.exp(-delta_dribbling_robot_ball_cmd_yaw * (robot_ball_cmd_yaw_error + robot_ball_body_yaw_error))
        return rew_dribbling_robot_ball_yaw

    def _reward_ang_vel_z(self):
        # 제자리 팽이 회전 속도 억제 (Yaw 축 각속도 페널티)
        return torch.square(self.env.base_ang_vel[:, 2])

    def _reward_lin_vel_xy(self):
        # 불필요한 바닥 미끄러짐 억제 (XY 수평 선속도 페널티)
        return torch.sum(torch.square(self.env.base_lin_vel[:, :2]), dim=1)
    
    def _reward_dribbling_ball_vel_norm(self):
        # target velocity is command input
        vel_norm_diff = torch.pow(torch.norm(self.env.commands[:, :2], dim=-1) - torch.norm(self.env.object_lin_vel[:, :2], dim=-1), 2)
        delta_vel_norm = 2.0
        rew_vel_norm_tracking = torch.exp(-delta_vel_norm * vel_norm_diff)
        return rew_vel_norm_tracking

    # def _reward_dribbling_ball_vel_angle(self):
    #     angle_diff = torch.atan2(self.env.commands[:,1], self.env.commands[:,0]) - torch.atan2(self.env.object_lin_vel[:,1], self.env.object_lin_vel[:,0])
    #     angle_diff_in_pi = torch.pow(wrap_to_pi(angle_diff), 2)
    #     rew_vel_angle_tracking = torch.exp(-5.0*angle_diff_in_pi/(torch.pi**2))
    #     # print("angle_diff", angle_diff, " angle_diff_in_pi: ", angle_diff_in_pi, " rew_vel_angle_tracking", rew_vel_angle_tracking, " commands", self.env.commands[:, :2], " object_lin_vel", self.env.object_lin_vel[:, :2])
    #     return rew_vel_angle_tracking

    def _reward_dribbling_ball_vel_angle(self):
        angle_diff = torch.atan2(self.env.commands[:,1], self.env.commands[:,0]) - torch.atan2(self.env.object_lin_vel[:,1], self.env.object_lin_vel[:,0])
        angle_diff_in_pi = torch.pow(wrap_to_pi(angle_diff), 2)
        rew_vel_angle_tracking = 1.0 - angle_diff_in_pi/(torch.pi**2)
        return rew_vel_angle_tracking

    def _reward_pitch_forward_penalty(self):
        # 고개가 앞으로 숙여지는 피치각 (pitch > 0) 억제 (무게중심을 뒤쪽으로 유지하도록 페널티)
        roll, pitch, yaw = get_euler_xyz(self.env.base_quat)
        forward_pitch = torch.clamp(pitch, min=0.0)
        return torch.square(forward_pitch)

    def _reward_stance_legs_support(self):
        # 공에서 더 멀리 떨어진 앞다리 1개 + 뒷다리 2개 = 총 3개 지지 다리(Stance legs) 지지 보상
        front_feet_indices = self.env.feet_indices[:2]  # FL, FR
        foot_pos = self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:, front_feet_indices, 0:3]
        ball_pos = self.env.object_pos_world_frame.unsqueeze(1)
        dist = torch.norm(foot_pos - ball_pos, dim=-1)
        
        # 킥 다리가 아닌 반대쪽 앞다리 index (argmax)
        non_kick_front_idx = torch.argmax(dist, dim=1)
        batch_indices = torch.arange(self.env.num_envs, device=self.env.device)
        chosen_front_feet = front_feet_indices[non_kick_front_idx]
        
        contact_forces = self.env.contact_forces
        rear_forces = torch.norm(contact_forces[:, self.env.feet_indices[2:], :], dim=-1)
        front_stance_force = torch.norm(contact_forces[batch_indices, chosen_front_feet, :], dim=-1, keepdim=True)
        
        rear_support = torch.sum(rear_forces > 1.0, dim=-1).float()
        front_support = (front_stance_force > 1.0).float().squeeze(-1)
        return rear_support + front_support

    def _reward_feet_slip(self):
        # 지면에 접촉해 있는 발이 바닥에서 미끄러지는 현상 감점 (Feet slip penalty)
        feet_vel = self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:, self.env.feet_indices, 7:9]
        contact = (self.env.contact_forces[:, self.env.feet_indices, 2] > 1.).float()
        return torch.sum(torch.norm(feet_vel, dim=-1) * contact, dim=1)