from turtlebot_maze_rl.turtlebot_maze_env import TurtleBotMazeEnv
from stable_baselines3 import PPO

def main():
    env = TurtleBotMazeEnv()
    model = PPO.load("ppo_turtlebot_maze")

    obs = env.reset()
    for _ in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, rewards, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()
    env.close()

if __name__ == "__main__":
    main()
