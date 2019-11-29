import gym
import rospy
import numpy as np
from ctr_kinematics.srv import ComputeForwardKinematics
from sensor_msgs.msg import JointState


# ROS environment to be used with RVim CTR kinematics library for variable curvature / stability modelling
class CtrEnv(gym.GoalEnv):
    def __init__(self):
        # Initialize rospy node and service
        rospy.init_node("ctr_gym_environment")
        self.fk_service = rospy.ServiceProxy('fk_service', ComputeForwardKinematics)
        # Initialize number of tubes(maybe from params server)
        self.num_tubes = 2
        # Seperately store joint values normally and also have cylindrical output for observation
        self.joint_state = np.empty([2, self.num_tubes])
        self.trig_joint_values = np.empty([3, self.num_tubes])
        # Initialize action space limits

        self.goal_tolerance = 0.001

    def reset(self):
        pass

    def seed(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

    def step(self, action):
        # Add action to joint values, convert to cylindrical representation
        # Call ROS service to compute FK
        joint_state_srv = JointState()
        joint_state_srv.position = self.joint_state
        response = self.fk_service(joint_state_srv)
        ee_pos = response.ee_pos
        pass

    def compute_reward(self, achieved_goal, desired_goal, info):
        assert achieved_goal.shape == desired_goal.shape
        d = np.linalg.norm(achieved_goal - desired_goal, axis=-1)
        return -(d > self.goal_tolerance).astype(np.float32)

    def render(self, mode='human'):
        pass

    def close(self):
        pass
