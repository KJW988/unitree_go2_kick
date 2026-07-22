import torch
from isaacgym.torch_utils import *
from dribblebot.rewards.kick_rewards import KickRewards


class KickTargetRewards(KickRewards):
    """
    Phase 2 (v1 AprilTag 3-Target 정밀 슈팅) 전용 리워드 클래스.
    1, 2, 3번 버튼으로 지정된 AprilTag 3D 상대 위치(kick_target_pos)로 
    공을 정밀 슈팅하도록 RoboNaldo (2026) Trajectory Extrapolation 보상을 결합.
    """

    def _reward_kick_target_precision(self):
        """
        RoboNaldo 2026 방식 Densified Trajectory Extrapolation Goal Reward:
        공이 타격되는 순간, 공의 속도(v_ball) 벡터로 궤적을 1초 뒤로 외삽(Extrapolate)하여
        선택된 AprilTag 목표점(kick_target_pos)과의 임팩트 거리를 보상.
        """
        # 선택된 AprilTag 위치가 없으면 0 보상
        if not hasattr(self.env, "kick_target_pos"):
            return torch.zeros(self.env.num_envs, device=self.env.device)

        v_ball = self.env.object_lin_vel[:, :2]
        p_ball = self.env.object_pos_world_frame[:, :2]
        p_tag = self.env.kick_target_pos[:, :2]

        # 공 속도 방향 단위 벡터
        ball_speed = torch.norm(v_ball, dim=-1, keepdim=True).clamp(min=1e-5)
        dir_ball = v_ball / ball_speed

        # 공 궤적 직선과 AprilTag 목표점 사이의 최단 충돌 거리를 수식으로 연산
        vec_to_tag = p_tag - p_ball
        proj_dist = torch.sum(vec_to_tag * dir_ball, dim=-1).clamp(min=0.0)
        closest_point = p_ball + dir_ball * proj_dist.unsqueeze(-1)

        impact_error = torch.norm(closest_point - p_tag, dim=-1)

        # 킥 품질(r_kick >= threshold)이 달성되는 순간 정밀 보상 활성화
        threshold = getattr(self.env.cfg.rewards, "kick_quality_threshold", 0.6)
        gate = (self._kick_quality() >= threshold).float()

        # 목표점에 가까울수록 1.0에 수렴하는 Exponential 보상 (sigma = 0.5m)
        return torch.exp(-2.0 * torch.square(impact_error)) * gate
