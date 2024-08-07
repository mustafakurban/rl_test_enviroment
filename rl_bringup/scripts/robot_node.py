#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import gymnasium as gym
from std_srvs.srv import Empty
from gymnasium import spaces
# import gym
# from gym import spaces

class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')
        self.get_logger().info('Robot Node started')

        # Define publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        self.observation_ = spaces.Box(low=np.array([-np.inf,-np.inf]), high=np.array([np.inf,np.inf]),  dtype=np.float32)
        self.odom = Odometry()
        self.scan_data = LaserScan()

    def scan_callback(self, data):
        self.scan_data = data

    def odom_callback(self, data):
        self.odom = data

    def get_observation(self):
        return np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y])

    def reset_simulation(self):
        # Reset the simulation
        reset_simulation = self.create_client(Empty, '/reset_simulation')
        while not reset_simulation.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for /reset_simulation service...')
        reset_request = Empty.Request()
        reset_simulation.call_async(reset_request)