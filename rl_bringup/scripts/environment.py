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

        self.observation_space = spaces.Box(low=np.array([-np.inf, -np.inf]), high=np.array([np.inf, np.inf]),
                                            dtype=np.float32)

        # Initialize state
        self.step_ = 0
        self.position = Pose()
        self.done = False
        self.desired_pose = Pose()
        self.running_step = 1.0
        self.reward = 0.0
        self.reward_range = (-np.inf, np.inf)
        self.robot_node_ = RobotNode()

        self.txt_file = open("data.txt", "w")

        # Initialize encoder
        self.encoder_names = ['Left', 'Right', 'Forward', 'Collision', 'Reward']
        self.encoder = [[] for _ in self.encoder_names]

        self._seed()

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def reset(self, seed=None, options=None):

        super().reset(seed=seed)
        print("Resetting Simulation")

        # Set desired position
        self.desired_pose.position.x = 5.0
        self.desired_pose.position.y = 5.0
        print("Desired Pose: ", self.desired_pose)

        self.robot_node_.reset_simulation()

        self.done = False
        rclpy.spin_once(self.robot_node_, timeout_sec=1)

        state = self.robot_node_.get_observation()

        # Get initial position and compute initial distance
        self.position = self.robot_node_.odom.pose.pose
        self.initial_distance = np.linalg.norm(np.array([self.position.position.x, self.position.position.y]) -
                                               np.array([self.desired_pose.position.x, self.desired_pose.position.y]))

        # Reset step counter
        self.step_ = 0

        # Reset encoder
        self.encoder = [[] for _ in self.encoder_names]

        info = {}

        print("Resetting Simulation Done")
        return state, info

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

        # Compute PPO reward (previously 1 - distance)
        ppo_reward = 1.0 - (distance / self.initial_distance)
        print(f"PPO Reward: {ppo_reward}")

        # Check for collision
        if min(scan_data.ranges) < 0.2:
            self.done = True
            reward = -1.0
        elif distance <= 0.5:  # Goal reached threshold
            self.done = True
            reward = 1.0
        else:
            self.done = False
            reward = 0.0

        # --------------------------------------------------------------------------
        # Encoder design
        # Update encoder based on the action and observations

        # Ensure all neurons are extended to current step
        for neuron in self.encoder:
            while len(neuron) <= self.step_:
                neuron.append(0)

        # Update action neurons at time t
        if action == 0:  # Turn left
            self.encoder[self.encoder_names.index('Left')][self.step_] = 1  # Left
        elif action == 1:  # Turn right
            self.encoder[self.encoder_names.index('Right')][self.step_] = 1  # Right
        elif action == 2:  # Move forward
            self.encoder[self.encoder_names.index('Forward')][self.step_] = 1  # Forward

        # Round distance to one decimal place
        rounded_distance = round(distance, 1)
        distance_neuron_name = f'Distance {rounded_distance}'

        # Check if distance neuron exists
        if distance_neuron_name in self.encoder_names:
            distance_neuron_index = self.encoder_names.index(distance_neuron_name)
        else:
            # Add new distance neuron
            self.encoder_names.append(distance_neuron_name)
            self.encoder.append([0] * (self.step_ + 1))
            distance_neuron_index = len(self.encoder) - 1

        # Ensure all neurons are extended to step_ + 2
        for neuron in self.encoder:
            while len(neuron) <= self.step_ + 1:
                neuron.append(0)

        # Update distance neuron at time t+1
        self.encoder[distance_neuron_index][self.step_ + 1] = 1

        # Update collision neuron at time t+1
        collision_index = self.encoder_names.index('Collision')
        if min(scan_data.ranges) < 0.2:
            self.encoder[collision_index][self.step_ + 1] = 1
        else:
            self.encoder[collision_index][self.step_ + 1] = 0

        # Update reward neuron at time t+1
        reward_index = self.encoder_names.index('Reward')
        if reward == 1.0:
            self.encoder[reward_index][self.step_ + 1] = 1
        else:
            self.encoder[reward_index][self.step_ + 1] = 0

        # Print encoder data for this step
        print(f"Step: {self.step_}")
        print(f"Distance: {distance}")
        print(f"Reward: {reward}")
        action_str = 'Left' if action == 0 else 'Right' if action == 1 else 'Forward'
        print(f"Turning {action_str}")
        print("Encoder at current step:")
        for idx, neuron_name in enumerate(self.encoder_names):
            neuron_values = self.encoder[idx][self.step_:self.step_ + 2]
            print(f"{neuron_name}: {neuron_values}")
        print("---------------------------------")

        self.step_ += 1

        # --------------------------------------------------------------------------

        return state, reward, self.done, False, {}

    # Function to print the entire encoder data after simulation ends
    def print_encoder_data(self):
        print("Final Encoder Data:")
        max_length = max(len(neuron) for neuron in self.encoder)
        for idx in range(max_length):
            print(f"Time Step {idx}:")
            for neuron_idx, neuron_name in enumerate(self.encoder_names):
                if idx < len(self.encoder[neuron_idx]):
                    value = self.encoder[neuron_idx][idx]
                else:
                    value = 0
                print(f"  {neuron_name}: {value}")
            print("---------------------------------")
