from pynput.keyboard import Key, Listener
import gym
import ctr_envs

import time

import numpy as np


class TeleopAgent(object):
    def __init__(self):
        self.env = gym.make('Exact-Ctr-v0')
        self.key_listener = Listener(on_press=self.on_press_callback)
        self.key_listener.start()

        self.action = np.zeros_like(self.env.action_space.low)
        self.exit = False

    def on_press_callback(self, key):
        # Tube 1 (inner most tube) is w s a d
        # Tube 2 (outer most tube) is t g f h
        # Tube 3 (outer most tube) is i k j l
        try:
            if key.char in ['w', 's', 'a', 'd']:
                print "tube 1 control"
                if key.char == 'w':
                    self.action[0] = self.env.action_space.high[0] / 2
                elif key.char == 's':
                    self.action[0] = self.env.action_space.low[0] / 2
                elif key.char == 'a':
                    self.action[3] = self.env.action_space.low[3] / 2
                elif key.char == 'd':
                    self.action[3] = self.env.action_space.high[3] / 2
            if key.char in ['t', 'g', 'f', 'h']:
                print "tube 3 control"
                if key.char == 't':
                    self.action[1] = self.env.action_space.high[1] / 2
                elif key.char == 'g':
                    self.action[1] = self.env.action_space.low[1] / 2
                elif key.char == 'f':
                    self.action[4] = self.env.action_space.low[4] / 2
                elif key.char == 'h':
                    self.action[4] = self.env.action_space.high[4] / 2
            if key.char in ['i', 'k', 'j', 'l']:
                print "tube 2 control"
                if key.char == 'i':
                    self.action[2] = self.env.action_space.high[2] / 2
                elif key.char == 'k':
                    self.action[2] = self.env.action_space.low[2] / 2
                elif key.char == 'j':
                    self.action[5] = self.env.action_space.low[5] / 2
                elif key.char == 'l':
                    self.action[5] = self.env.action_space.high[5] / 2
        except AttributeError:
            if key == Key.esc:
                self.exit = True
                exit()
            else:
                self.action = np.zeros_like(self.env.action_space.low)

    def run(self):
        obs = self.env.reset()
        while not self.exit:
            print self.action
            observation, reward, done, info = self.env.step(self.action)
            self.env.render()
            if info['is_success']:
                obs = self.env.reset()
            self.action = np.zeros_like(self.env.action_space.low)
            time.sleep(0.1)
        self.env.close()


if __name__ == '__main__':
    teleop_agent = TeleopAgent()
    teleop_agent.run()
