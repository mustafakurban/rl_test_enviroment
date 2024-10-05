#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
# import gym
import gymnasium as gym
import numpy as np
import time
from gymnasium import spaces
import gym
# from gym import spaces
from geometry_msgs.msg import Pose
from gymnasium.utils import seeding
import random
from robot_node import RobotNode
import rclpy
from geometry_msgs.msg import Twist


class TurtleBotMazeEnv(gym.Env):
    def __init__(self):
        
        # We assume that a ROS node has already been created
        # before initialising the environment

        # Define action and observation space

        self.action_space = spaces.Discrete(3)  # 3 possible actions: left, right, forward
        
        self.observation_space = spaces.Box(low=np.array([-np.inf,-np.inf]), high=np.array([np.inf,np.inf]),  dtype=np.float32)
        
        # Initialize state
        self.step_ = 0
        self.position = Pose()
        self.done = False  
        self.desired_pose = Pose()   
        self.running_step = 1.0
        self.reward = 0.0
        self.reward_range = (-np.inf, np.inf)
        self.robot_node_ = RobotNode()
        self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
        
    # Resets the state of the environment and returns an initial observation.
    def reset(self,seed=None,options=None):

        super().reset(seed=seed)
        print("Resetting Simulation")

        #create rando positin range low = 0.1 high = 
        self.desired_pose.position.x = 5.0
        self.desired_pose.position.y = 5.0
        print("Desired Pose: ",self.desired_pose)

        self.robot_node_.reset_simulation()

        self.done = False
        rclpy.spin_once(self.robot_node_, timeout_sec=1)

        state = self.robot_node_.get_observation()

        # time.sleep(self.running_step)
        info = {}

        self.encoder = [
            [],  # Left
            [],  # Right
            [],  # Forward
            [],  # Distance 0.1
            [],  # Distance 0.2
            [],  # Distance 0.3
            [],  # Distance 0.4
            [],  # Distance 0.5
            [],  # Distance 0.6
            [],  # Distance 0.7
            [],  # Distance 0.8
            [],  # Distance 0.9
            [],  # Distance 1.0
            [],  # Distance 1.1
            [],  # Distance 1.2
            [],  # Distance 1.3
            [],  # Distance 1.4
            [],  # Distance 1.5
            [],  # Distance 1.6
            [],  # Distance 1.7
            [],  # Distance 1.8
            [],  # Distance 1.9
            [],  # Distance 2.0
            [],  # Distance 2.1
            [],  # Distance 2.2
            [],  # Distance 2.3
            [],  # Distance 2.4
            [],  # Distance 2.5
            [],  # Distance 2.6
            [],  # Distance 2.7
            [],  # Distance 2.8
            [],  # Distance 2.9
            [],  # Distance 3.0
            [],  # Distance 3.1
            [],  # Distance 3.2
            [],  # Distance 3.3
            [],  # Distance 3.4
            [],  # Distance 3.5
            [],  # Distance 3.6
            [],  # Distance 3.7
            [],  # Distance 3.8
            [],  # Distance 3.9
            [],  # Distance 4.0
            [],  # Distance 4.1
            [],  # Distance 4.2
            [],  # Distance 4.3
            [],  # Distance 4.4
            [],  # Distance 4.5
            [],  # Distance 4.6
            [],  # Distance 4.7
            [],  # Distance 4.8
            [],  # Distance 4.9
            [],  # Distance 5.0
            [],  # Distance 5.1
            [],  # Distance 5.2
            [],  # Distance 5.3
            [],  # Distance 5.4
            [],  # Distance 5.5
            [],  # Distance 5.6
            [],  # Distance 5.7
            [],  # Distance 5.8
            [],  # Distance 5.9
            [],  # Distance 6.0
            [],  # Distance 6.1
            [],  # Distance 6.2
            [],  # Distance 6.3
            [],  # Distance 6.4
            [],  # Distance 6.5
            [],  # Distance 6.6
            [],  # Distance 6.7
            [],  # Distance 6.8
            [],  # Distance 6.9
            [],  # Distance 7.0
            [],  # Distance 7.1
            [],  # Distance 7.2
            [],  # Distance 7.3
            [],  # Distance 7.4
            [],  # Distance 7.5
            [],  # Distance 7.6
            [],  # Distance 7.7
            [],  # Distance 7.8
            [],  # Distance 7.9
            [],  # Distance 8.0
            [],  # Distance 8.1
            [],  # Distance 8.2
            [],  # Distance 8.3
            [],  # Distance 8.4
            [],  # Distance 8.5
            [],  # Distance 8.6
            [],  # Distance 8.7
            [],  # Distance 8.8
            [],  # Distance 8.9
            [],  # Distance 9.0
            [],  # Distance 9.1
            [],  # Distance 9.2
            [],  # Distance 9.3
            [],  # Distance 9.4
            [],  # Distance 9.5
            [],  # Distance 9.6
            [],  # Distance 9.7
            [],  # Distance 9.8
            [],  # Distance 9.9
            [],  # Collision
            []   # Reward
        ]


        print("Resetting Simulation Done")
        return state,info

    def _render(self, mode='human', close=True):
        pass

    def step(self, action):
        # Execute action
        move_cmd = Twist()
        empty_twist = Twist()
        if action == 0:  # Turn left
            move_cmd.angular.z = 0.5
            print("Turning Left")
        elif action == 1:  # Turn right
            move_cmd.angular.z = -0.5
            print("Turning Right")
        elif action == 2:  # Move forward
            move_cmd.linear.x = 0.3
            print("Moving Forward")
        self.robot_node_.vel_pub.publish(move_cmd)
        time.sleep(self.running_step)
        self.robot_node_.vel_pub.publish(empty_twist)
        rclpy.spin_once(self.robot_node_, timeout_sec=1)

        # Observe new state
        state = self.robot_node_.get_observation()
        self.position = self.robot_node_.odom.pose.pose
        scan_data = self.robot_node_.scan_data
        distance = np.linalg.norm(np.array([self.position.position.x, self.position.position.y]) -
                                  np.array([self.desired_pose.position.x, self.desired_pose.position.y]))

        # Check for collision
        if min(scan_data.ranges) < 0.2:
            self.done = True
            reward = -1.0
        else:
            self.done = False
            reward = 1.0

        # --------------------------------------------------------------------------
        # Encoder design
        # Update encoder based on the action and observations

        # Fill all lists first index to zero
        for i in self.encoder:
            i.append(0)
            i.append(0)

        if action == 0:  # Turn left
            self.encoder[0][self.step_] = 1  # Left
        elif action == 1:  # Turn right
            self.encoder[1][self.step_] = 1  # Right
        elif action == 2:  # Move forward
            self.encoder[2][self.step_] = 1  # Forward

        # Update distance encoder
        distance_index = min(int(distance * 10), 99)  # Ensure the index is within range (0-99)
        if len(self.encoder[3 + distance_index]) == 0:
            self.encoder[3 + distance_index][self.step_+1] = 1

        # Update collision and reward encoders
        self.encoder[-2][self.step_+1] = (1 if self.done else 0)  # Collision
        self.encoder[-1][self.step_+1] = reward  # Reward
        # --------------------------------------------------------------------------

        print("Step: ", self.step_)
        print("Distance: ", distance)
        print("Reward: ", reward)
        self.step_ += 2

        return state, reward, self.done, False, {}
