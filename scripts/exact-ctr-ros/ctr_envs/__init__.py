from gym.envs.registration import register

register(
        id='Exact-Ctr-v0',
        entry_point='ctr_envs.envs:CtrEnv',
        max_episode_steps=100
        )
