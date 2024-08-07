#!/usr/bin/env python3
import environment
import os
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv,VecNormalize,VecMonitor
from stable_baselines3.common.callbacks  import EvalCallback
import rclpy
def main():

    reg = gym.register(
        id='Robot-v0',
        entry_point='environment:TurtleBotMazeEnv',
        max_episode_steps=100
        )
        
    total_timesteps = 1000000000
    training_step = 5000

    environment_name = 'Robot-v0'
    #Eval parameters
    train_name = "robot_PPO"
    eval_freq = 5000
    eval_episodes = 20
    eval_log_path = os.path.join("training_results",environment_name,train_name,"eval_logs")
    deterministic = True
    train_file_name = "robot_PPO"
    save_path = os.path.join("training_results",environment_name,train_name)
    log_path = os.path.join("training_results",environment_name,train_name,"logs")
    log_name = train_name

    if not os.path.exists(save_path):
        os.makedirs(save_path)
    if not os.path.exists(eval_log_path):
        os.makedirs(eval_log_path)
    if not os.path.exists(log_path):
        os.makedirs(log_path)


    env = gym.make(environment_name)
    env = DummyVecEnv([lambda: env])
    # env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)
    # env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.)


    vec_env = VecMonitor(env)

    # PPO parameters
    n_steps = 50
    n_epochs = 1
    gamma = 0.99
    gae_lambda = 0.95
    clip_range = 0.2
    clip_range_vf = 0.2
    ent_coef = 0.001
    vf_coef = 0.5
    max_grad_norm = 0.5


    model = PPO('MlpPolicy', vec_env, verbose=2, tensorboard_log=log_path, n_steps=n_steps,
                n_epochs=n_epochs,gamma=gamma,ent_coef=ent_coef,vf_coef=vf_coef,
                max_grad_norm=max_grad_norm,clip_range=clip_range,
                clip_range_vf=clip_range_vf,gae_lambda=gae_lambda,batch_size=n_steps)
    
    print("Training with PPO")    
  
    eval_callback = EvalCallback(vec_env, best_model_save_path=save_path, log_path=eval_log_path, 
                                 eval_freq=eval_freq, n_eval_episodes=eval_episodes, deterministic=deterministic, render=False)

    print("Learning")
    for i in range(int(total_timesteps/training_step)):

        step_name = train_name + "_" + str((i+1)*training_step)
        model.set_env(vec_env)

        # model.learn(total_timesteps=10000) 
        model.learn(total_timesteps=training_step, log_interval=1, callback=eval_callback,progress_bar=True,reset_num_timesteps=False)
        env.close()
        model.save(save_path+"/"+step_name )

        # if i < training_step:
        #     model.load(save_path+"/"+step_name,env=vec_env, tensorboard_log=log_path+"/"+log_name )


if __name__ == '__main__':
    rclpy.init()
    main()