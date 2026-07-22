# 배치 위치: <your-dribblebot-clone>/dribblebot/rewards/kick_rewards.py
#
# github.com/Improbable-AI/dribblebot 를 직접 clone해서 아래 항목을 실제 코드로 확인한 뒤 작성했습니다:
#   - dribblebot/rewards/soccer_rewards.py       -> SoccerRewards 클래스, _reward_dribbling_* 실제 구현
#   - dribblebot/envs/base/legged_robot.py       -> object_local_pos / object_lin_vel / object_pos_world_frame /
#                                                     base_lin_vel / rigid_body_state / feet_indices 실제 정의·좌표계
#   - dribblebot/envs/base/legged_robot_config.py -> reward_container_name, only_positive_rewards_ji22_style 등
#   - scripts/train_dribbling.py                  -> 실제 학습에 쓰인 reward_scales 크기 (0.5~4.0대)
#
# 즉 이 파일의 부모 클래스(SoccerRewards)와 그 안에서 재사용하는 _reward_dribbling_robot_ball_yaw /
# _reward_dribbling_robot_ball_pos 는 "논문에서 묘사된 방식"이 아니라 실제 공개된 코드 그대로입니다.
# 반대로 _reward_kicking_ball_vel / _reward_kick_contact / _reward_kick_hold 는 이 리포에 없던 부분이라
# Su et al. 2025 (CoRL) 논문 Table III의 Pursue&Strike/Hold 2단계 설계와, RoboNaldo (arXiv:2606.11092)의
# Instant Interaction Reward 아이디어를 참고해 제가 새로 작성한 부분입니다. 학습 전 튜닝이 필요합니다.

import torch
from isaacgym.torch_utils import *
from dribblebot.rewards.soccer_rewards import SoccerRewards


class KickRewards(SoccerRewards):
    """
    Go2 Kick 태스크용 리워드. SoccerRewards를 상속하므로
    _reward_dribbling_robot_ball_yaw, _reward_dribbling_robot_ball_pos,
    _reward_orientation, _reward_torques 등 기존 항을 그대로 reward_scales에서
    켜서 같이 쓸 수 있습니다 (아래 kick config 참고).
    """

    # ---- 초기 자세 안정화 게이팅 (스폰 직후 1.0초간은 4다리 서기 자세를 먼저 완벽히 잡도록 접근 보상 억제) ----
    def _reward_dribbling_robot_ball_pos(self):
        settled_gate = (self.env.episode_length_buf > 50).float()
        rew_pos = super()._reward_dribbling_robot_ball_pos()
        return rew_pos * settled_gate

    def _reward_dribbling_robot_ball_yaw(self):
        settled_gate = (self.env.episode_length_buf > 50).float()
        rew_yaw = super()._reward_dribbling_robot_ball_yaw()
        return rew_yaw * settled_gate

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
    # Su2025 Table III의 kicking_ball_vel(=r^kick) 재현.
    # 접근/정렬은 부모 클래스의 dribbling_robot_ball_yaw, dribbling_robot_ball_pos를 그대로 재사용하는 걸
    # 전제로 하고 있어서 여기서는 "타격 속도"만 담당합니다.
    def _reward_kicking_ball_vel(self):
        cmd = self.env.commands[:, :2]
        cmd_dir = cmd / (torch.norm(cmd, dim=-1, keepdim=True) + 1e-6)
        ball_vel_along_cmd = torch.sum(self.env.object_lin_vel[:, :2] * cmd_dir, dim=-1)
        r_strike = torch.clamp(ball_vel_along_cmd, min=0.0)  # 역방향으로 맞았을 때 음수 보상 방지

        threshold = getattr(self.env.cfg.rewards, "kick_quality_threshold", 0.8)
        gate = (self._kick_quality() < threshold).float()
        return r_strike * gate

    # ---- 접촉 shaping (제안 추가, RoboNaldo Eq.1 Instant Interaction Reward 방식) ----
    # 발-공 접촉은 물리스텝 3~5개 수준으로 매우 짧아 kicking_ball_vel 하나만으로는
    # 크레딧 할당이 잘 안 될 수 있습니다. 접근 자체에 별도 shaping을 줘서 완화합니다.
    def _reward_kick_contact(self):
        # 킥 다리는 앞다리 2개(FL, FR)에만 한정하여 뒷다리 비비기 방지
        front_feet_indices = self.env.feet_indices[:2]
        foot_pos = self.env.rigid_body_state.view(self.env.num_envs, -1, 13)[:, front_feet_indices, 0:3]
        ball_pos = self.env.object_pos_world_frame.unsqueeze(1)
        dist = torch.norm(foot_pos - ball_pos, dim=-1)
        min_dist = torch.min(dist, dim=1).values
<<<<<<< HEAD

        # 공 접근 임팩트 시 지지다리(3개) 중 최소 2개 이상이 바닥을 단탄히 받치도록 강제 (4다리 허우적거림 편법 차단)
        support_legs = self.env.feet_indices[1:]
        support_contact = torch.norm(self.env.contact_forces[:, support_legs, :2], dim=-1) > 1.0
        support_gate = (torch.sum(support_contact.float(), dim=-1) >= 2.0).float()

        return torch.exp(-4.0 * torch.square(min_dist)) * (0.5 + 0.5 * support_gate)
=======
        return torch.exp(-4.0 * torch.square(min_dist))
>>>>>>> 0d0d8a3 (feat: Initial commit-kick RL learning)

    # ---- Hold 단계 (r_kick >= threshold일 때만 활성) ----
    # Su2025: "Once r_kick exceeds the threshold... the robot is rewarded for
    # stabilizing its posture in place." 를 그대로 구현.
    def _reward_kick_hold(self):
        threshold = getattr(self.env.cfg.rewards, "kick_quality_threshold", 0.8)
        gate = (self._kick_quality() >= threshold).float()
        lin_vel_error = torch.sum(torch.square(self.env.base_lin_vel[:, :2]), dim=1)
        sigma = getattr(self.env.cfg.rewards, "tracking_sigma", 0.25)
        return torch.exp(-lin_vel_error / sigma) * gate

    # ---- v1 스텁: 목표 지점 정밀 슈팅용 (지금은 비활성, kick_target_pos 관측 추가 후 사용) ----
    # RoboNaldo 방식(탄도 외삽으로 골라인 통과 지점을 매 스텝 예측해 조기에 보상 신호를 주는 것)을
    # 그대로 옮긴 골격만 남겨둡니다. v0가 안정적으로 돌고 나서 켜세요.
    def _reward_kick_target_precision(self):
        if not hasattr(self.env, "kick_target_pos"):
            return torch.zeros(self.env.num_envs, device=self.env.device)
        dist_to_target = torch.norm(
            self.env.object_pos_world_frame[:, :2] - self.env.kick_target_pos[:, :2], dim=-1
        )
        return torch.exp(-torch.square(dist_to_target) / 1.0)