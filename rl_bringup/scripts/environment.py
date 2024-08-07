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
        self.step_ += 1
        distance = np.linalg.norm(np.array([self.position.position.x, self.position.position.y]) - 
        np.array([self.desired_pose.position.x, self.desired_pose.position.y]))
      

        # Check for collision
        if min(scan_data.ranges) < 0.2:
            self.done = True
            reward = -100
        else:
            self.done = False
            reward = 100 - distance
        print("Step: ",self.step_)
        print("Distance: ",distance)
        print("Reward: ",reward)
        return state, reward ,self.done, False,{}
