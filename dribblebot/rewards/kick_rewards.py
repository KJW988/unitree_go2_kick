# 배치 위치: <your-dribblebot-clone>/dribblebot/rewards/kick_rewards.py

import torch
from isaacgym.torch_utils import *
from dribblebot.rewards.soccer_rewards import SoccerRewards


class KickRewards(SoccerRewards):
    """
    Go2 Kick 태스크용 리워드. SoccerRewards를 상속하므로
    _reward_dribbling_robot_ball_yaw, _reward_dribbling_robot_ball_pos,
    _reward_orientation, _reward_torques 등 기존 항을 그대로 reward_scales에서
    켜서 같이 쓸 수 있습니다.
    """

    # ---- 초기 자세 안정화 게이팅 (스폰 직후 1.5초간은 4다리 서기 자세를 먼저 완벽히 잡도록 모든 태스크 보상 억제) ----
    def _settled_gate(self):
        # 스폰/리셋 후 첫 1.5초(75스텝, dt=0.02) 동안은 4다리로 균형 있게 서는 착지 안정화에만 전념하도록 게이트 처리
        return (self.env.episode_length_buf > 75).float()

    def _reward_dribbling_robot_ball_pos(self):
        rew_pos = super()._reward_dribbling_robot_ball_pos()
        return rew_pos * self._settled_gate()

    def _reward_dribbling_robot_ball_yaw(self):
        rew_yaw = super()._reward_dribbling_robot_ball_yaw()
        return rew_yaw * self._settled_gate()

    # ---- 누락되었던 보상/페널티 함수 추가 (base_height, lin_vel_z, ang_vel_xy, dof_vel, still_standing) ----
    def _reward_still_standing(self):
        # 스폰 후 첫 1.5초(75스텝) 착지 구간 동안 잔발 딛기 없이 기본 자세로 미동도 없이 정지해 있으면 +3.0점 만점 부여
        unsettled_gate = 1.0 - self._settled_gate()
        dof_pos_err = torch.sum(torch.square(self.env.dof_pos - self.env.default_dof_pos), dim=1)
        dof_vel_err = torch.sum(torch.square(self.env.dof_vel), dim=1)
        return torch.exp(-0.5 * dof_pos_err) * torch.exp(-0.01 * dof_vel_err) * unsettled_gate

    def _reward_dof_vel(self):
        # 관절 미세 흔들림 / 제자리 잔발 딛기 감점
        return torch.sum(torch.square(self.env.dof_vel), dim=1)

    def _reward_base_height(self):
        # 몸통 높이가 목표 높이(base_height_target = 0.38m)에서 이탈할 때 페널티 부여
        base_height = self.env.root_states[self.env.robot_actor_idxs, 2]
        target_height = getattr(self.env.cfg.rewards, "base_height_target", 0.38)
        return torch.square(base_height - target_height)

    def _reward_lin_vel_z(self):
        # 몸통 Z축 상하 튀김/요동 페널티
        return torch.square(self.env.base_lin_vel[:, 2])

    def _reward_ang_vel_xy(self):
        # 몸통 롤/피치 각속도 흔들림 페널티
        return torch.sum(torch.square(self.env.base_ang_vel[:, :2]), dim=1)

    # ---- 내부 헬퍼 (reward 함수가 아니라 _reward_ 접두사 없음 -> 자동 등록 안 됨) ----
    def _kick_quality(self):
        """
        r_kick in [0, 1]. commands[:, :2]를 '목표 방향 단위벡터'로 해석하고,
        그 방향으로의 공 속도 성분을 kick_vel_target(m/s)으로 정규화합니다.
        Su2025의 상태조건부 게이팅(r_kick >= threshold -> Hold)에 그대로 대응됩니다.
        """
        cmd = self.env.commands[:, :2]
        cmd_dir = cmd / (torch.norm(cmd, dim=-1, keepdim=True) + 1e-6)
        ball_vel_along_cmd = torch.sum(self.env.object_lin_vel[:, :2] * cmd_dir, dim=-1)
        target = getattr(self.env.cfg.rewards, "kick_vel_target", 3.0)
        return torch.clamp(ball_vel_along_cmd / target, 0.0, 1.0)

    # ---- Pursue & Strike 단계 (r_kick < threshold일 때만 활성) ----
    def _reward_kicking_ball_vel(self):
        cmd = self.env.commands[:, :2]
        cmd_dir = cmd / (torch.norm(cmd, dim=-1, keepdim=True) + 1e-6)
        ball_vel_along_cmd = torch.sum(self.env.object_lin_vel[:, :2] * cmd_dir, dim=-1)
        r_strike = torch.clamp(ball_vel_along_cmd, min=0.0)  # 역방향으로 맞았을 때 음수 보상 방지

        threshold = getattr(self.env.cfg.rewards, "kick_quality_threshold", 0.8)
        gate = (self._kick_quality() < threshold).float()
        return r_strike * gate * self._settled_gate()

    # ---- 접촉 shaping (RoboNaldo Eq.1 Instant Interaction Reward 방식) ----
    # 발-공 접촉 보상. 반드시 서 있는 상태(3지점 지지 + 몸통 높이 유지 + 1.5초 착지 완료)에서만 보상 부여.
    def _reward_kick_contact(self):
        # 1. 두 앞다리(FL:0, FR:1) 중 공에 더 가까운 발을 킥 다리로, 다른 하나를 지지 앞다리로 동적 선택
        front_feet_indices = self.env.feet_indices[:2]  # [FL, FR]
        foot_pos = self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:, front_feet_indices, 0:3]
        ball_pos = self.env.object_pos_world_frame.unsqueeze(1)
        dist = torch.norm(foot_pos - ball_pos, dim=-1)  # (num_envs, 2)
        min_dist = torch.min(dist, dim=1).values

        # 공에 더 가까운 앞발이 킥 발(kick_foot), 반대쪽 앞발이 지지 앞발(chosen_support_front)
        kick_front_idx = torch.argmin(dist, dim=1)
        non_kick_front_idx = 1 - kick_front_idx
        batch_indices = torch.arange(self.env.num_envs, device=self.env.device)
        chosen_support_front = front_feet_indices[non_kick_front_idx]

        # 2. 지지 다리 3개의 Z축 수직 지지력 검증
        contact_forces = self.env.contact_forces
        rear_feet_indices = self.env.feet_indices[2:]  # [RL, RR]
        rear_support = torch.sum(contact_forces[:, rear_feet_indices, 2] > 1.0, dim=-1).float()
        front_support = (contact_forces[batch_indices, chosen_support_front, 2] > 1.0).float()

        # 3. 하드 게이트: 지지 다리 2개 미만이면 보상 = 0 (다이빙/몸 던지기 완전 차단)
        total_support_cnt = rear_support + front_support
        support_gate = (total_support_cnt >= 2.0).float()

        # 4. 몸통 높이 게이트: 주저앉으며 접근하는 편법도 차단 (높이 0.25m 이상 유지해야 보상)
        base_height = self.env.root_states[self.env.robot_actor_idxs, 2]
        height_gate = (base_height > 0.25).float()

        return torch.exp(-4.0 * torch.square(min_dist)) * support_gate * height_gate * self._settled_gate()

    # ---- Hold 단계 (r_kick >= threshold일 때만 활성) ----
    def _reward_kick_hold(self):
        threshold = getattr(self.env.cfg.rewards, "kick_quality_threshold", 0.6)
        gate = (self._kick_quality() >= threshold).float()
        lin_vel_error = torch.sum(torch.square(self.env.base_lin_vel[:, :2]), dim=1)
        # 킥 임팩트 후 뻗은 다리를 접어 원래 기본 서 있는 자세(default_dof_pos)로 회수(Recovery)하도록 유도
        dof_pos_error = torch.sum(torch.square(self.env.dof_pos - self.env.default_dof_pos), dim=1)
        sigma = getattr(self.env.cfg.rewards, "tracking_sigma", 0.25)
        return torch.exp(-lin_vel_error / sigma) * torch.exp(-0.2 * dof_pos_error) * gate * self._settled_gate()

    # ---- v1 스텁: 목표 지점 정밀 슈팅용 ----
    def _reward_kick_target_precision(self):
        if not hasattr(self.env, "kick_target_pos"):
            return torch.zeros(self.env.num_envs, device=self.env.device)
        dist_to_target = torch.norm(
            self.env.object_pos_world_frame[:, :2] - self.env.kick_target_pos[:, :2], dim=-1
        )
        return torch.exp(-torch.square(dist_to_target) / 1.0)