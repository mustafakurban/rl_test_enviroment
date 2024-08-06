import gym
from gym import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class TurtleBotMazeEnv(gym.Env, Node):
    def __init__(self):
        super().__init__('turtlebot_maze_rl')

        # Define action and observation space
        self.action_space = spaces.Discrete(3)  # 3 possible actions: left, right, forward
        self.observation_space = spaces.Box(low=0, high=10, shape=(24,), dtype=np.float32)

        # Initialize ROS node
        rclpy.init(args=None)

        # Define publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        # Initialize state
        self.scan_data = np.zeros(24)
        self.position = None
        self.done = False

    def scan_callback(self, data):
        self.scan_data = np.array(data.ranges)

    def odom_callback(self, data):
        self.position = data.pose.pose.position

    def step(self, action):
        # Execute action
        move_cmd = Twist()
        if action == 0:  # Turn left
            move_cmd.angular.z = 0.5
        elif action == 1:  # Turn right
            move_cmd.angular.z = -0.5
        elif action == 2:  # Move forward
            move_cmd.linear.x = 0.5
        self.vel_pub.publish(move_cmd)
        rclpy.spin_once(self, timeout_sec=1)

        # Observe new state
        state = self.scan_data

        # Check for collision
        if min(state) < 0.2:
            self.done = True
            reward = -100
        else:
            self.done = False
            reward = 1

        return np.array(state), reward, self.done, {}

    def reset(self):
        # Reset the simulation
        reset_simulation = self.create_client(Empty, '/reset_simulation')
        while not reset_simulation.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /reset_simulation service...')
        reset_request = Empty.Request()
        reset_simulation.call(reset_request)

        # Reset state
        self.done = False
        rclpy.spin_once(self)
        return np.array(self.scan_data)

    def render(self, mode='human'):
        pass

    def close(self):
        rclpy.shutdown()
