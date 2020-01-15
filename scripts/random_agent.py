import gym
import ctr_envs

import numpy as np
import rospy


if __name__ == '__main__':
    env = gym.make('Exact-Ctr-v0')
    obs = env.reset()
    while True:
        action = env.action_space.sample()
        new_obs, reward, done, info = env.step(action)
        env.render()
        rospy.sleep(1.0)

