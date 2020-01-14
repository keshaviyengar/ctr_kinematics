import gym
import numpy as np

from CtrModel import CTRModel
from CtrModel import TubeParameters


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

        # tube1 = TubeParameters(length=0.150, length_curved=103e-3, inner_diameter=2 * 0.35e-3,
        #                        outer_diameter=2 * 0.55e-3,
        #                        stiffness=6.4359738368e+10, torsional_stiffness=2.5091302912e+10, x_curvature=21.3,
        #                        y_curvature=0)
        # tube2 = TubeParameters(length=0.100, length_curved=113e-3, inner_diameter=2 * 0.7e-3,
        #                        outer_diameter=2 * 0.9e-3,
        #                        stiffness=5.2548578304e+10, torsional_stiffness=2.1467424256e+10, x_curvature=13.108,
        #                        y_curvature=0)
        # tube3 = TubeParameters(length=0.070, length_curved=134e-3, inner_diameter=2e-3, outer_diameter=2 * 1.1e-3,
        #                        stiffness=4.7163091968e+10, torsional_stiffness=2.9788923392e+10, x_curvature=3.5,
        #                        y_curvature=0)

        # Initialize CTRModel
        self.ctr_model = CTRModel(tube1, tube2, tube3)
        self.tube_lengths = [tube1.L, tube2.L, tube3.L]

        # Initialize trig joint space
        trig_joint_low = np.array([])
        trig_joint_high = np.array([])
        for tube_length in [tube1.L, tube2.L, tube3.L]:
            trig_joint_low = np.append(trig_joint_low, [-1, -1, -tube_length])
            trig_joint_high = np.append(trig_joint_high, [1, 1, 0])
        self.trig_joint_space = gym.spaces.Box(low=trig_joint_low, high=trig_joint_high, dtype=np.float32)

        # Initialize joint space
        joint_space_low = np.array([])
        joint_space_high = np.array([])
        for tube_length in [tube1.L, tube2.L, tube3.L]:
            joint_space_low = np.append(joint_space_low, [-tube_length])

        joint_space_low = np.concatenate((joint_space_low, np.full(int(self.n), -np.inf)))
        joint_space_high = np.concatenate((np.zeros(int(self.n)), np.full(int(self.n), np.inf)))

        self.joint_space = gym.spaces.Box(low=joint_space_low, high=joint_space_high)

        # Seperately store joint values normally and also have cylindrical output for observation
        # joint values is [beta_1, beta_2, ..., alpha_1, alpha_2, alpha_3]
        self.joint_state = np.empty(2 * self.n)
        # trig representation is [sin(alpha_1), cos(alpha_2), beta_1, sin(alpha_2]
        self.trig_joint_state = np.empty([3, self.n])

        # Set initial joint and trig state
        self.joint_state = np.array([0, 0, 0, 0, 0, 0])
        self.trig_joint_state = self.full_joint2trig(self.joint_state)

        # Observation space (desired goal, achieved goal, observation)
        self.observation_space = gym.spaces.Dict(dict(
            desired_goal=gym.spaces.Box(low=np.array([0.01, 0.01, 0.01]),
                                        high=np.array([0.05, 0.05, 0.05]), dtype="float32"),
            achieved_goal=gym.spaces.Box(low=np.array([0.01, 0.01, 0.01]),
                                         high=np.array([0.05, 0.05, 0.05]), dtype="float32"),
            observation=gym.spaces.Box(
                low=np.concatenate((self.trig_joint_space.low, np.array([-np.inf, -np.inf, -np.inf]))),
                high=np.concatenate((self.trig_joint_space.high, np.array([np.inf, np.inf, np.inf]))),
                dtype="float32")
        ))

        # Action space
        action_length_limit = 0.001
        action_orientation_limit = np.deg2rad(5)
        action_low = np.array([])
        action_high = np.array([])
        for i in range(0, self.n):
            action_low = np.append(action_low, [-action_orientation_limit, -action_length_limit])
            action_high = np.append(action_high, [action_orientation_limit, action_length_limit])

        self.action_space = gym.spaces.Box(low=action_low, high=action_high, dtype="float32")

        # substeps: Number of times to repeat selection action
        self.substeps = 10

        self.desired_goal = np.empty(3)
        self.achieved_goal = np.empty(3)
        self.desired_trig_joint = np.empty(3 * self.n)
        self.goal_tolerance = 0.001

        # backbone shape
        self.r = []

    # Reset environment for new episode: Resample a new valid cartesian goal and return observation object.
    def reset(self):
        self.t = 0
        # Resample goal
        self.desired_goal, self.desired_trig_joint = self.sample_goal()

        r, u_z_end, tip_pos = self.ctr_model.fk(self.joint_state)

        # Update global variables
        self.r = r
        self.achieved_goal = r[-1]

        # Initialize observation object
        dg, _ = self.sample_goal()
        self.desired_goal = dg

        obs = {
            'desired_goal': self.desired_goal,
            'achieved_goal': r[-1],
            'observation': np.concatenate((self.trig_joint_state, self.desired_goal - self.achieved_goal))
        }
        return obs

    def seed(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

    def step(self, action):
        # Add action to current joint state, include substeps here
        for _ in range(self.substeps):
            self.set_action(action)

        # Compute FK
        r, u_z_end, tip_pos = self.ctr_model.fk(self.joint_state)

        # Update global variables
        self.r = r
        self.achieved_goal = r[-1]
        # Compute reward
        reward = self.compute_reward(self.achieved_goal, self.desired_goal, dict())
        state = np.concatenate((self.trig_joint_state, self.desired_goal - self.achieved_goal))

        info = {}

        return {'observation': state, 'achieved_goal': tip_pos, 'desired_goal': self.desired_goal}, reward, r == 0, info

    def compute_reward(self, achieved_goal, desired_goal, info):
        assert achieved_goal.shape == desired_goal.shape
        d = np.linalg.norm(achieved_goal - desired_goal, axis=-1)
        return -(d > self.goal_tolerance).astype(np.float32)

    # Grab the backbone transforms and publish in ROS for rviz visualization
    def render(self, mode='human'):
        self.ctr_model.publish_transforms(self.r)

    def close(self):
        # No closing calls yet
        pass

    def set_action(self, action):
        """
        Take an action value in joint space, add it to current robot joints, limit and convert to
        trig space. Sets new trig and joint space values
        """
        self.joint_state = self.joint_state + np.clip(action, self.action_space.low, self.action_space.high)
        self.joint_state = np.clip(self.joint_state, self.joint_space.low, self.joint_space.high)

        # Convert to cylindrical/trig representation
        self.trig_joint_state = self.full_joint2trig(self.joint_state)

        separate_trig_state = [self.trig_joint_state[i:i + int(len(self.trig_joint_state) / self.n)] for i in
                               range(0, len(self.trig_joint_state), int(len(self.trig_joint_state) / self.n))]

        for i in range(1, self.n):
            # Bi-1 <= Bi
            separate_trig_state[i - 1][2] = min(separate_trig_state[i - 1][2], separate_trig_state[i][2])
            # Bi-1 >= Bi - Li-1 + Li
            # if i == 1:
            #     print('beta_0: ', q_pos[i-1][2])
            #     print('beta_1: ', q_pos[i][2])
            #     print('L_1: ', self.tube_length[i])
            #     print('L_0: ', self.tube_length[0])
            #     print('B_1 + L_1 - L_0: ', self.tube_length[i] - self.tube_length[i-1] + q_pos[i][2])
            separate_trig_state[i - 1][2] = max(separate_trig_state[i - 1][2],
                                                self.tube_lengths[i] - self.tube_lengths[i - 1] +
                                                separate_trig_state[i][2])

        self.trig_joint_state = np.concatenate(separate_trig_state)
        self.trig_joint_state = np.clip(self.trig_joint_state, self.trig_joint_space.low, self.trig_joint_space.high)

    # For a single joint_state convert to trig representation
    # Define joint state and trig_joint_state as the following.
    # Joint state: [beta_1, beta_2, ..., alpha_1, alpha_2]
    # Trig joint state: [sin(alpha_1), cos(alpha_1), beta_1, sin(alpha_2), cos(alpha_2), beta_2]
    @staticmethod
    def single_joint2trig(joint_state):
        return np.array([np.sin(joint_state[1]),
                         np.cos(joint_state[1]),
                         joint_state[0]])

    def full_joint2trig(self, joint_state):
        full_trig = np.empty(3 * self.n)
        for tube in range(0, self.n):
            beta = joint_state[tube]
            alpha = joint_state[int(len(joint_state) / 2) + tube]
            trig = self.single_joint2trig(np.array([alpha, beta]))
            full_trig[tube:tube + 3] = trig
        return full_trig

    @staticmethod
    def single_trig2joint(trig_state):
        return np.array([np.arctan2(trig_state[0], trig_state[1]), trig_state[2]])

    def full_trig2joint(self, trig_state):
        beta = np.empty(self.n)
        alpha = np.empty(self.n)
        for tube in range(0, self.n):
            joint = self.single_trig2joint(trig_state[tube])
            alpha[tube] = joint[0]
            beta[tube] = joint[1]
        full_joint = np.concatenate((beta, alpha))
        return full_joint

    def sample_goal(self):
        """
        Sample a goal from forward kinematics with valid q_pos that fits all constraints
        """
        # while loop to get constrained points, maybe switch this for a workspace later on
        while True:
            q_sample = self.trig_joint_space.sample()
            q_sample = [q_sample[i:i + int(len(q_sample) / self.n)] for i in
                        range(0, len(q_sample), int(len(q_sample) / self.n))]
            # Apply constraints
            valid_joint = []
            for i in range(1, self.n):
                valid_joint.append((q_sample[i - 1][2] <= q_sample[i][2]) and (
                        q_sample[i - 1][2] + self.tube_lengths[i - 1] >= self.tube_lengths[i] + q_sample[i][2]))
            if all(valid_joint):
                break
        joint_sample = self.full_trig2joint(np.array(q_sample))
        r, u_z_end, goal = self.ctr_model.fk(joint_sample)
        end_pos = r[-1]
        self.r = r
        return end_pos, q_sample
