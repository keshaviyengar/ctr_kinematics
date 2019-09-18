from pynput.keyboard import Key, Listener
import time
import numpy as np

import rospy
from sensor_msgs.msg import JointState


class TeleopAgent:
    def __init__(self):
        # Initialize ros
        rospy.init_node("joint_space_teleop")
        self.joint_state_pub_ = rospy.Publisher("joint_state", JointState, queue_size=10)

        self.key_listener = Listener(on_press=self.on_press_callback)
        self.key_listener.start()

        self.joint_states = JointState()
        self.joint_states.position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.del_phi = 0.0001
        self.del_alpha = 5 * np.pi / 180

        self.exit = False

    def on_press_callback(self, key):
        # Tube 1 (inner most tube) is w s a d
        # Tube 2 (outer most tube) is up down left right
        # up is action to extend tube
        # down is action to de-extend tube
        # left is action to rotate tube left
        # right is action to rotate tube right
        del_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        try:
            if key.char in ['w', 's', 'a', 'd', 't', 'g', 'f', 'h', 'i', 'k', 'j', 'l']:
                if key.char == 'w':
                    del_joints[0] = self.del_phi
                if key.char == 's':
                    del_joints[0] = -self.del_phi
                if key.char == 'a':
                    del_joints[1] = -self.del_alpha
                if key.char == 'd':
                    del_joints[1] = self.del_alpha
                if key.char == 't':
                    del_joints[2] = self.del_phi
                if key.char == 'g':
                    del_joints[2] = -self.del_phi
                if key.char == 'f':
                    del_joints[3] = -self.del_alpha
                if key.char == 'h':
                    del_joints[3] = self.del_alpha
                if key.char == 'i':
                    del_joints[4] = self.del_phi
                if key.char == 'k':
                    del_joints[4] = -self.del_phi
                if key.char == 'j':
                    del_joints[5] = -self.del_alpha
                if key.char == 'l':
                    del_joints[5] = self.del_alpha

                self.joint_states.position += del_joints
                self.joint_state_pub_.publish(self.joint_states)

        except AttributeError:
            if key == Key.esc:
                print("escaped pressed")
                self.exit = True
                exit()


if __name__ == '__main__':
    teleop_agent = TeleopAgent()
    while not rospy.is_shutdown():
        rospy.spin()
