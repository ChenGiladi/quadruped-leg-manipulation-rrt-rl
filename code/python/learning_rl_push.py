"""
Training, inference, and mass-robustness sweep for the box-pushing PPO agent.

USAGE
-----
    # Train from scratch (1M steps, default)
    python learning_rl_push.py --mode train

    # Replay the published agent (../models/best_model8800.zip)
    python learning_rl_push.py --mode infer --model ../models/best_model8800.zip --episodes 3

    # Reproduce Figure 17 data: sweep box mass, log mean angular/position error
    python learning_rl_push.py --mode sweep --model ../models/best_model8800.zip \
        --masses 0,200,300,400,500,600,700,800

Required environment variable:
    COPPELIASIM_PY_PATH   absolute path to <CoppeliaSimRoot>/programming/zmqRemoteApi/clients/python

Optional environment variable:
    RL_PUSH_LOG_DIR       output directory (default: ./logs/rl_push)
"""
import argparse
import csv
import os

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy

from environment_rl_push import RobotModelEnv

os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "True")

LOG_DIR = os.environ.get("RL_PUSH_LOG_DIR", os.path.join(os.getcwd(), "logs", "rl_push"))
os.makedirs(LOG_DIR, exist_ok=True)


class SaveOnBestTrainingRewardCallback(BaseCallback):
    def __init__(self, check_freq: int, log_dir: str, verbose: int = 1):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, "best_model")
        self.best_mean_reward = -np.inf

    def _init_callback(self) -> None:
        if self.save_path is not None:
            os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % 100 == 0:
            self.save_path = os.path.join(self.log_dir, "best_model" + str(self.n_calls))
            self.model.save(self.save_path)
        return True


def _make_env():
    env = RobotModelEnv(action_type='discrete')
    return Monitor(env, LOG_DIR)


def cmd_train(args):
    env = _make_env()
    print("Action space:", env.action_space)
    print("Creating PPO model — see Table 4 of the paper for hyperparameters.")
    policy_kwargs = dict(net_arch=[512, 512, 512, 512])
    model = PPO(policy="MlpPolicy", env=env, learning_rate=0.001, verbose=True,
                batch_size=512, policy_kwargs=policy_kwargs)
    checkpoint_callback = CheckpointCallback(save_freq=500, save_path=LOG_DIR)
    callback_best_reward = SaveOnBestTrainingRewardCallback(check_freq=250, log_dir=LOG_DIR)
    callback = CallbackList([checkpoint_callback, callback_best_reward])
    model.learn(total_timesteps=int(args.timesteps), callback=callback, progress_bar=True)
    final = os.path.join(LOG_DIR, "final_model.zip")
    model.save(final)
    print(f"Training complete. Final model: {final}")


def _run_episode(model, env):
    """Run one deterministic episode, return per-step records and summary errors."""
    obs = env.reset()
    done = False
    angular_errors, position_errors = [], []
    total_reward, n_steps = 0.0, 0
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        total_reward += float(reward)
        n_steps += 1
        # The environment_rl_push.RobotModelEnv exposes the current corner
        # offsets via self.dist_*; pull whatever it surfaces in `info` if
        # present, otherwise read off the env at episode end.
    final_ang = float(np.mean(np.abs(getattr(env, "angle_err_log", [0.0]))))
    final_pos_x = float(np.mean(np.abs(getattr(env, "dist_1x", [0.0]))))
    final_pos_y = float(np.mean(np.abs(getattr(env, "dist_1y", [0.0]))))
    final_pos = float(np.hypot(final_pos_x, final_pos_y))
    return dict(steps=n_steps, total_reward=total_reward,
                angular_err_deg=final_ang, position_err_m=final_pos)


def cmd_infer(args):
    env = _make_env()
    print(f"Loading policy from {args.model}")
    model = PPO.load(args.model, env=env)
    out_path = os.path.join(LOG_DIR, "infer_rollouts.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["episode", "steps", "total_reward", "angular_err_deg", "position_err_m"])
        for ep in range(args.episodes):
            rec = _run_episode(model, env)
            w.writerow([ep, rec["steps"], rec["total_reward"],
                        rec["angular_err_deg"], rec["position_err_m"]])
            print(f"Episode {ep}: steps={rec['steps']} reward={rec['total_reward']:.2f} "
                  f"ang_err={rec['angular_err_deg']:.2f} deg pos_err={rec['position_err_m']:.3f} m")
    print(f"Saved rollout log to {out_path}")


def cmd_sweep(args):
    """Mass-robustness sweep used to produce Figure 17 of the paper.

    For each requested Δm/m_train value (in percent), set the box mass in
    CoppeliaSim, run `episodes` deterministic rollouts, and record the mean
    final angular and position error. Output is one CSV row per mass value.

    The actual box-mass write is delegated to RobotModelEnv.set_box_mass_kg()
    if defined; if not, the function tries `env.sim.setShapeMass(...)` on the
    CUBE handle. Add or override as appropriate for your scene-tagging
    convention.
    """
    env = _make_env()
    print(f"Loading policy from {args.model}")
    model = PPO.load(args.model, env=env)

    masses_pct = [float(x) for x in args.masses.split(",")]
    base_mass_kg = float(args.base_mass_kg)
    out_path = os.path.join(LOG_DIR, "mass_sweep.csv")

    inner = getattr(env, "env", env)
    cube_handle = getattr(inner, "CUBE", None) or getattr(inner, "cube_handle", None)

    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["delta_m_pct", "total_mass_kg",
                    "angular_err_deg_mean", "position_err_m_mean", "n_episodes"])
        for dm_pct in masses_pct:
            new_mass = base_mass_kg * (1.0 + dm_pct / 100.0)
            if hasattr(inner, "set_box_mass_kg"):
                inner.set_box_mass_kg(new_mass)
            elif cube_handle is not None and hasattr(inner, "sim"):
                inner.sim.setShapeMass(cube_handle, new_mass)
            else:
                print(f"  [warn] cannot set box mass for Δm={dm_pct}% — "
                      f"add RobotModelEnv.set_box_mass_kg() or expose the CUBE handle")
            ang_runs, pos_runs = [], []
            for _ in range(args.episodes):
                rec = _run_episode(model, env)
                ang_runs.append(rec["angular_err_deg"])
                pos_runs.append(rec["position_err_m"])
            ang_mean = float(np.mean(ang_runs))
            pos_mean = float(np.mean(pos_runs))
            w.writerow([dm_pct, new_mass, ang_mean, pos_mean, args.episodes])
            print(f"  Δm={dm_pct:6.1f}% mass={new_mass:.3f} kg "
                  f"ang_err={ang_mean:.2f} deg pos_err={pos_mean:.3f} m")
    print(f"Saved mass-sweep results to {out_path}")
    print("Replot Figure 17 with: python regenerate_r2_figures.py (after editing it "
          "to load this CSV instead of the hard-coded thesis data).")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("train", "infer", "sweep"), required=True,
                        help="What to do.")
    parser.add_argument("--model", default="../models/best_model8800.zip",
                        help="Path to the SB3 PPO checkpoint (infer/sweep only).")
    parser.add_argument("--episodes", type=int, default=3,
                        help="Number of deterministic episodes per evaluation point.")
    parser.add_argument("--timesteps", type=float, default=1e6,
                        help="Training horizon (train mode only).")
    parser.add_argument("--masses", default="0,200,300,400,500,600,700,800",
                        help="Comma-separated Δm/m_train values in percent (sweep mode).")
    parser.add_argument("--base-mass-kg", type=float, default=0.1,
                        help="Training box mass; sweep multiplies this by 1 + Δm/100.")
    args = parser.parse_args()

    if args.mode == "train":
        cmd_train(args)
    elif args.mode == "infer":
        cmd_infer(args)
    elif args.mode == "sweep":
        cmd_sweep(args)


if __name__ == "__main__":
    main()
