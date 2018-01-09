from gym.envs.registration import register

register(
    id='Stage-Follower-v0',
    entry_point='stage_follower.stage_follower_env_play:StageEnvPlay',
)