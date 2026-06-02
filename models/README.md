# Trained PPO Policy — `best_model8800.zip`

This is the trained Proximal Policy Optimization (PPO) agent that produces the
straight-line and circular-arc box-pushing behaviors reported in the paper.

| | |
|---|---|
| **File** | `best_model8800.zip` |
| **Size** | 19.4 MB |
| **Trained** | 2024-01-31 |
| **SHA256** | `cae30d88acc5c20fa4383e0f7bc7eb27312042d8a2dfbe311fee5e669f240d66` |

## Exact training environment (embedded in the checkpoint)

| Package | Version |
|---|---|
| Stable-Baselines3 | **1.8.0** |
| Gym | **0.21.0** *(not Gymnasium — SB3 1.8 is the final Gym-based release)* |
| PyTorch | 2.0.1 (CPU) |
| NumPy | 1.23.5 |
| Python | 3.10.11 |
| OS | Windows 10 |
| GPU | disabled (CPU training only) |

These versions are baked into the model bundle (`system_info.txt` inside the
zip); SB3 will warn at load time if the loading environment does not match.

## Loading and running inference

```bash
pip install stable-baselines3==1.8.0 gym==0.21.0 torch==2.0.1 numpy==1.23.5
```

```python
from stable_baselines3 import PPO
from EnvronmentRLyogev_Year_1 import RobotModelEnv  # see ../code/python/

env = RobotModelEnv()
model = PPO.load("best_model8800.zip", env=env)

obs = env.reset()
done = False
while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
```

A running CoppeliaSim instance with `RoboDog_Learning_Push.ttt` open (see
`../code/scenes/`) is required because the env communicates with the simulator
via the ZeroMQ Remote API.

## Provenance

Saved by Yogev Attias on 2024-01-31 from a successful PPO training run on the
quadruped box-pushing task. The numeric suffix `8800` is the training-step
index from which the best-mean-reward checkpoint was emitted by the
`SaveOnBestTrainingRewardCallback` defined in
`../code/python/Learning_RL_Push.py`.
