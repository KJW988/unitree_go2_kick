import os
import argparse
import isaacgym
assert isaacgym
import torch
import numpy as np

from dribblebot.envs.base.legged_robot_config import Cfg
from dribblebot.envs.go2.go2_kick_config import config_go2_kick
from dribblebot.envs.go1.velocity_tracking import VelocityTrackingEasyEnv
from dribblebot.rewards.kick_rewards import KickRewards  # noqa: F401
from dribblebot.envs.wrappers.history_wrapper import HistoryWrapper
from dribblebot_learn.ppo_cse.actor_critic import ActorCritic


def play_go2_kick(checkpoint_path: str, output_video: str = "kick_demo.mp4", cmd_x: float = 1.5, cmd_y: float = 0.0):
    """
    Headless 서버 환경에서 학습된 Go2 Kick Policy(.pt)를 불러와 
    오프라인 비디오(mp4)로 렌더링하고 평가하는 스크립트.
    """
    config_go2_kick(Cfg)
    Cfg.env.num_envs = 1  # 평가 시 1개 단일 환경 사용

    device = "cuda:0"
    env = VelocityTrackingEasyEnv(sim_device=device, headless=True, cfg=Cfg)
    env = HistoryWrapper(env)

    # ActorCritic 신경망 초기화 및 체크포인트 로드
    actor_critic = ActorCritic(
        num_obs=env.num_obs,
        num_privileged_obs=env.num_privileged_obs,
        num_obs_history=env.num_obs_history,
        num_actions=env.num_actions,
    ).to(device)

    if os.path.exists(checkpoint_path):
        print(f"Loading checkpoint: {checkpoint_path}")
        loaded_dict = torch.load(checkpoint_path, map_location=device)
        actor_critic.load_state_dict(loaded_dict)
    else:
        print(f"[Warning] Checkpoint {checkpoint_path} not found! Using random policy for dry-run.")

    actor_critic.eval()

    # 오프라인 프레임 캡처용 비디오 프레임 리스트
    frames = []
    
    # 킥 목표 방향 명령 설정 (예: 정면 1.5m/s 킥)
    env.env.commands[:, 0] = cmd_x
    env.env.commands[:, 1] = cmd_y

    obs = env.reset()
    print(f"Evaluating Go2 Kick with Command: ({cmd_x}, {cmd_y})...")

    # 4초간 (200 스텝) headless 오프라인 렌더링 수행
    for i in range(200):
        with torch.no_grad():
            actions = actor_critic.act_teacher(obs)

        obs, rew, done, info = env.step(actions)

        # Offscreen Camera Render (FloatingCameraSensor 이용)
        if hasattr(env.env, "rendering_camera"):
            frame = env.env.rendering_camera.get_observation()
            if frame is not None:
                # RGB uint8 numpy 변환
                if isinstance(frame, torch.Tensor):
                    frame = frame.cpu().numpy()
                frames.append(frame)

    # imageio를 활용한 mp4 저장
    if len(frames) > 0:
        try:
            import imageio
            imageio.mimsave(output_video, frames, fps=30)
            print(f"✅ Kick demo video successfully saved to: {output_video}")
        except Exception as e:
            print(f"Failed to save video using imageio: {e}")
    else:
        print("Note: Camera frame capture skipped. Play script executed clean.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ckpt", type=str, default="./tmp/legged_data/ac_weights_latest.pt")
    parser.add_argument("--out", type=str, default="kick_demo.mp4")
    parser.add_argument("--cmd_x", type=float, default=1.5)
    parser.add_argument("--cmd_y", type=float, default=0.0)
    args = parser.parse_args()

    play_go2_kick(args.ckpt, args.out, args.cmd_x, args.cmd_y)
