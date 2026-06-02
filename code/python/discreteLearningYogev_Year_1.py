# from stable_baselines3 import SAC
# import os
# import csv
# import visdom
# # from Visdom import VisdomCallback
# import numpy as np
# from stable_baselines3 import SAC
# # from stable_baselines3 import DDPG
# from stable_baselines3.common.callbacks import CallbackList, BaseCallback
# from stable_baselines3.common.callbacks import CheckpointCallback
# from stable_baselines3.common.monitor import Monitor
# from stable_baselines3.common.results_plotter import load_results, ts2xy
# from gymnasium.spaces.box import Box as GymnasiumBox
# from EnvronmentRLyogev import RobotModelEnv
import os
import numpy as np
from stable_baselines3 import PPO  # Import PPO
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
from EnvronmentRLyogev_Year_1 import RobotModelEnv
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'


class SaveOnBestTrainingRewardCallback(BaseCallback):
    """
    Callback for saving a model (the check is done every ``check_freq`` steps)
    based on the training reward (in practice, we recommend using ``EvalCallback``).

    :param check_freq: (int)
    :param log_dir: (str) Path to the folder where the model will be saved.
      It must contains the file created by the ``Monitor`` wrapper.
    :param verbose: (int)
    """

    def __init__(self, check_freq: int, log_dir: str, verbose=1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, "best_model")
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        # Create folder if needed
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % 100 == 0:
            self.save_path = os.path.join(log_dir, "best_model"+str(self.n_calls))
            self.model.save(self.save_path)
            # # Retrieve training reward
            # x, y = ts2xy(load_results(self.log_dir), "timesteps")
            # if len(x) > 0:
            #     # Mean training reward over the last 100 episodes
            #     mean_reward = np.mean(y[-100:])
            #     if self.verbose > 0:
            #         print(f"Num timesteps: {self.num_timesteps}")
            #         print(
            #             f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}"
            #         )
            #
            #     # New best model, you could save the agent here
            #     if mean_reward > self.best_mean_reward:
            #         self.best_mean_reward = mean_reward
            #         # Example for saving best model
            #         if self.verbose > 0:
            #             print(f"Saving new best model to {self.save_path}.zip")


        return True


log_dir = r"C:\Users\yogev\Documents\yogev\Robodog\saved_models\random"

# env = RobotModelEnv(action_type='continuous')
# env = RobotModelEnv(action_type='discrete')
env = RobotModelEnv(action_type='discrete')
env = Monitor(env, log_dir)
print("Action space:", env.action_space)
print("Type:", type(env.action_space))
env = Monitor(env, log_dir)
print("create a new model")
policy_kwargs = dict(net_arch=[512, 512, 512, 512])
model = PPO(policy="MlpPolicy", env=env, learning_rate=0.001, verbose=True, batch_size=512,
            policy_kwargs=policy_kwargs)
checkpoint_callback = CheckpointCallback(save_freq=500, save_path=log_dir)
callback_best_reward = SaveOnBestTrainingRewardCallback(check_freq=250, log_dir=log_dir)
callback = CallbackList([checkpoint_callback, callback_best_reward])
model.learn(total_timesteps=1e6, callback=callback, progress_bar=True)
checkpoint_callback = CheckpointCallback(save_freq=500, save_path=log_dir)
callback_best_reward = SaveOnBestTrainingRewardCallback(check_freq=250, log_dir=log_dir)
callback = CallbackList([checkpoint_callback, callback_best_reward])
model.learn(total_timesteps=1e6, callback=callback, progress_bar=True)
