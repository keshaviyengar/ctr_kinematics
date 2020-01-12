import gym
import numpy as np

from CtrModel import TubeParameters
from CtrModel import CTRModel


# ROS environment to be used with RVim CTR kinematics library for variable curvature / stability modelling
class CtrEnv(gym.GoalEnv):
    def __init__(self):
        # Set tube parameters
        self.n = 3
        self.t = 0
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
        self.tube_parameters = [tube1, tube2, tube2]

        # Seperately store joint values normally and also have cylindrical output for observation
        # joint values is [beta_1, beta_2, ..., alpha_1, alpha_2, alpha_3]
        self.joint_state = np.empty(2 * self.n)
        # trig representation is [sin(alpha_1), cos(alpha_2), beta_1, sin(alpha_2]
        self.trig_joint_values = np.empty([3, self.n])

        # Initialize trig joint space
        trig_joint_low = np.array([])
        trig_joint_high = np.array([])
        for tube_parameter in [tube1, tube2, tube3]:
            trig_joint_low = np.append(trig_joint_low, [-1, -1, -tube_parameter.L])
            trig_joint_high = np.append(trig_joint_high, [1, 1, 0])
        self.trig_joint_space = gym.spaces.Box(low=trig_joint_low, high=trig_joint_high, dtype="float32")

        # Observation space (desired goal, achieved goal, observation)
        self.observation_space = gym.spaces.Dict(dict(
            desired_goal=gym.spaces.Box(low=np.array([0.01, 0.01, 0.01]),
                                        high=np.array([0.05, 0.05, 0.05]), dtype="float32"),
            achieved_goal=gym.spaces.Box(low=np.array([0.01, 0.01, 0.01]),
                                         high=np.array([0.05, 0.05, 0.05]), dtype="float32"),
            observation=gym.spaces.Box(low=np.concatenate(self.trig_joint_space.low, 0),
                                       high=np.concatenate(self.trig_joint_space.high, 10), dtype="float32")
        ))

        # Action space

        # substeps: Number of times to repeat selection action
        self.substeps = 10

        self.desired_goal = np.empty(3)
        self.desired_trig_joint = np.empty(3 * self.n)
        self.goal_tolerance = 0.001

    def reset(self):
        self.t = 0
        # Resample goal
        self.desired_goal, self.desired_trig_joint = self._sample_goal()

    def seed(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

    def step(self, action):
        # Add action to current joint state
        self.joint_state = self.joint_state + action
        # Enforce limits

        # Convert to cylindrical/trig representation
        self.trig_joint_values = self.full_joint2trig(self.joint_state)
        # Compute FK
        r, u_z_end, tip_pos = self.ctr_model.fk(self.joint_state)
        # Compute reward
        reward = self.compute_reward(tip_pos, self.desired_goal, dict())
        state = np.concatenate(tip_pos, self.desired_goal - tip_pos)

        info = {}

        return {'observation': state, 'achieved_goal': tip_pos, 'desired_goal': self.desired_goal}, reward, r == 0, info

    def compute_reward(self, achieved_goal, desired_goal, info):
        assert achieved_goal.shape == desired_goal.shape
        d = np.linalg.norm(achieved_goal - desired_goal, axis=-1)
        return -(d > self.goal_tolerance).astype(np.float32)

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    # For a single joint_state convert to trig representation
    @staticmethod
    def single_joint2trig(joint_state):
        return np.array([np.sin(joint_state[1]),
                         np.cos(joint_state[1]),
                         joint_state[0]])

    def full_joint2trig(self, joint_state):
        full_trig = np.array(3 * self.n)
        for tube in range(0, self.n):
            trig = self.single_joint2trig(joint_state[tube:tube + 1])
            full_trig[tube:tube + 2] = trig
        return full_trig

    @staticmethod
    def single_trig2joint(trig_state):
        return np.array([np.arctan2(trig_state[0], trig_state[1]), trig_state[2]])

    def full_trig2joint(self, trig_state):
        beta = np.array(self.n)
        alpha = np.array(self.n)
        for tube in range(0, self.n):
            joint = self.single_trig2joint(trig_state[tube:tube + 2])
            beta[tube] = joint[0]
            alpha[tube] = joint[1]
        full_joint = np.concatenate((beta, alpha))
        return full_joint

    def _sample_goal(self):
        """
        Sample a goal from forward kinematics with valid q_pos that fits all constraints
        """
        # while loop to get constrained points, maybe switch this for a workspace later on
        while True:
            joint_sample = self.trig_joint_space.sample()
            joint_sample = [joint_sample[i:i + int(len(joint_sample) / self.n)] for i in
                            range(0, len(joint_sample), int(len(joint_sample) / self.n))]
            # Apply constraints
            valid_joint = []
            for i in range(1, self.n):
                valid_joint.append((joint_sample[i - 1][2] <= joint_sample[i][2]) and (
                        joint_sample[i - 1][2] + self.tube_parameters[i - 1].L >= self.tube_parameters[i].L +
                        joint_sample[i][2]))
            if all(valid_joint):
                break
        goal = self.ctr_model.fk((self.full_trig2joint(joint_sample)))
        return goal.flatten(), joint_sample

