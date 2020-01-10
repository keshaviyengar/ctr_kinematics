import gym
import numpy as np

from CtrModel import TubeParameters
from CtrModel import CTRModel


# ROS environment to be used with RVim CTR kinematics library for variable curvature / stability modelling
class CtrEnv(gym.GoalEnv):
    def __init__(self):
        # Set tube parameters
        self.n = 3
        tube1 = TubeParameters(length=431e-3, length_curved=103e-3, inner_diameter=2 * 0.35e-3,
                               outer_diameter=2 * 0.55e-3,
                               stiffness=6.4359738368e+10, torsional_stiffness=2.5091302912e+10, x_curvature=21.3,
                               y_curvature=0)
        tube2 = TubeParameters(length=332e-3, length_curved=113e-3, inner_diameter=2 * 0.7e-3,
                               outer_diameter=2 * 0.9e-3,
                               stiffness=5.2548578304e+10, torsional_stiffness=2.1467424256e+10, x_curvature=13.108,
                               y_curvature=0)
        tube3 = TubeParameters(length=174e-3, length_curved=134e-3, inner_diameter=2e-3, outer_diameter=2 * 1.1e-3,
                               stiffness=4.7163091968e+10, torsional_stiffness=2.9788923392e+10, x_curvature=3.5,
                               y_curvature=0)

        # Initialize CTRModel
        self.ctr_model = CTRModel(tube1, tube2, tube3)

        # Seperately store joint values normally and also have cylindrical output for observation
        self.joint_state = np.empty([2, self.n])
        self.trig_joint_values = np.empty([3, self.n])
        # Initialize action space limits

        self.goal_tolerance = 0.001

    def reset(self):
        # Resample goal
        pass

    def seed(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

    def step(self, action):
        # Add action to current joint state
        # Compute FK
        # Convert to cylindrical representation
        # Return observation, reward, done, info
        pass

    def compute_reward(self, achieved_goal, desired_goal, info):
        assert achieved_goal.shape == desired_goal.shape
        d = np.linalg.norm(achieved_goal - desired_goal, axis=-1)
        return -(d > self.goal_tolerance).astype(np.float32)

    def render(self, mode='human'):
        pass

    def close(self):
        pass
