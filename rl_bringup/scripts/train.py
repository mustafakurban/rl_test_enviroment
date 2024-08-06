from turtlebot_maze_env import TurtleBotMazeEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

def main():
    env = TurtleBotMazeEnv()
    check_env(env)

    model = PPO('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=10000)
    model.save("ppo_turtlebot_maze")

if __name__ == "__main__":
    main()
