# kick_quality / kicking_ball_vel / kick_hold 곡선을 따로 보면서 threshold를 튜닝

def train_go2_kick(headless=True):
    import isaacgym
    assert isaacgym
    import torch

    from dribblebot.envs.base.legged_robot_config import Cfg
    from dribblebot.envs.go2.go2_kick_config import config_go2_kick
    from dribblebot.envs.go1.velocity_tracking import VelocityTrackingEasyEnv  # 범용 클래스, 그대로 재사용
    from dribblebot.rewards.kick_rewards import KickRewards  # noqa: F401  (reward_container_name으로 참조됨)

    from dribblebot_learn.ppo_cse import Runner, RunnerArgs
    from dribblebot.envs.wrappers.history_wrapper import HistoryWrapper
    from dribblebot_learn.ppo_cse.actor_critic import AC_Args
    from dribblebot_learn.ppo_cse.ppo import PPO_Args

    config_go2_kick(Cfg)
    # RTX A6000 하드웨어 및 4096개 병렬 환경으로 학습 속도 4배 가속!
    Cfg.env.num_envs = 4096

    import wandb
    wandb.init(
        project="go2-kick",
        name="go2-kick-v0",
        reinit=True,
        config={
            "AC_Args": vars(AC_Args),
            "PPO_Args": vars(PPO_Args),
            "RunnerArgs": vars(RunnerArgs),
            "Cfg": vars(Cfg),
        },
    )

    device = "cuda:0"
    env = VelocityTrackingEasyEnv(sim_device=device, headless=headless, cfg=Cfg)
    env = HistoryWrapper(env)

    runner = Runner(env, device=device)
    runner.learn(num_learning_iterations=1_000_000, init_at_random_ep_len=True, eval_freq=100)


if __name__ == "__main__":
    train_go2_kick(headless=True)