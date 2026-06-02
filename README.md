# A Hybrid RRT–PPO Framework for Leg-Based Object Manipulation of Quadruped Robots

Reproducibility package for the MDPI *Robotics* manuscript by Yogev Attias and
Chen Giladi (corresponding author). The framework couples an RRT global path
planner (MATLAB, Robotics System Toolbox) with a Proximal Policy Optimization
(PPO) controller for leg-based pushing motions (Python, Stable-Baselines3),
both driving a Unitree Go1 quadruped in a CoppeliaSim/MuJoCo physics
simulator. This repository contains the code, training data, trained policy
and supplementary video assets needed to reproduce the experiments reported
in the paper.

The manuscript itself is **not** in this repository — it is hosted by the
journal.

## Repository layout

```
code/
  matlab/      RRT global planner, smoothing, footprint adaptation, occupancy map
  python/      Gym environment, PPO training/inference, ZMQ bridge to CoppeliaSim,
               figure-regeneration script
  lua/         CoppeliaSim per-object child scripts (cube, Spot/Go1)
  scenes/      CoppeliaSim .ttt scene files (RoboDog_RRT, RoboDog_Learning_Push)
data/
  Cube.csv             Logged box-corner trajectories (84 MB; warns on push, no LFS needed)
  RRT_Data.xlsx        Planned RRT path waypoints used by the simulator side
  data.mat             MATLAB workspace snapshot from a representative run
  sine_wave/           Down-sampled measured box-coordinate trajectories
                       (sine_curve_l.npz, sine_curve_r.npz) — Figure 21 source
models/
  best_model8800.zip   Trained PPO policy (SB3 1.8 / Gym 0.21 / Torch 2.0.1)
  README.md            Versions, SHA-256, load instructions
videos/
  01..07_*.mp4         Six supplementary clips referenced from the paper
```

## Requirements

| Component | Version | Source |
|---|---|---|
| Python | 3.10.11 | python.org |
| CoppeliaSim EDU | 4.5.x (MuJoCo engine) | coppeliarobotics.com |
| MATLAB | R2023a or newer with **Robotics System Toolbox** and **Navigation Toolbox** | mathworks.com |
| ZeroMQ Remote API | bundled with CoppeliaSim 4.5 | — |

Python packages are pinned in `code/python/requirements.txt`. Install with:

```bash
cd code/python
pip install -r requirements.txt
```

`stable-baselines3==1.8.0`, `gym==0.21.0`, `torch==2.0.1`, `numpy==1.23.5` are
required exactly — they are the versions baked into the saved policy.

## Reproducing the main results

1. **Plan an RRT path (MATLAB).** Open `code/matlab/RoboDog_RRT_Planner.m`,
   adjust `start` / `goal` if desired, run. The script writes
   `data/RRT_Data.xlsx`, which the CoppeliaSim side reads.

2. **Launch the simulator.** Open `code/scenes/RoboDog_RRT.ttt` (for RRT
   path-following) or `code/scenes/RoboDog_Learning_Push.ttt` (for the PPO
   pushing task) in CoppeliaSim and start the simulation.

3. **Run the controller (Python).**
   * Inference with the trained policy:
     `python code/python/Run_RRT_In_Coppeliasim.py` after loading
     `models/best_model8800.zip` (see `models/README.md` for the exact load
     snippet).
   * Retrain from scratch: `python code/python/Learning_RL_Push.py` — uses
     `EnvronmentRLyogev_Year_1.py` as the Gym environment.

4. **Regenerate the data-driven figures.**
   ```
   cd code/python
   python regenerate_r2_figures.py --figure all          # Figs 16, 21
   python regenerate_r2_figures.py --figure straight_gallery
   python regenerate_r2_figures.py --figure bezier_panelb
   ```

## Figure ↔ source map

| Figure | Source script / asset |
|---|---|
| Fig. 10 — System architecture | TikZ in manuscript (no external script) |
| Fig. 16 — Mass-robustness scatter | `code/python/regenerate_r2_figures.py --figure r2` |
| Fig. 17a — Straight-line push gallery | `regenerate_r2_figures.py --figure straight_gallery` ← `videos/03_Straight_Line_Box_Pushing.mp4` |
| Fig. 17b — Bézier push gallery | `regenerate_r2_figures.py --figure bezier_panelb` ← `videos/02_Bezier_Trajectory_Box_Pushing.mp4` |
| Fig. 18b — Box-corner XY trajectory | Authoritative PNG in manuscript (data: `data/Cube.csv`) |
| Fig. 21 — Sinusoidal fit | `regenerate_r2_figures.py --figure sine` ← `data/sine_wave/*.npz` |

## Citation

```
@article{AttiasGiladi2026Go1,
  title   = {A Hybrid {RRT--PPO} Framework for Leg-Based Object Manipulation of Quadruped Robots},
  author  = {Attias, Yogev and Giladi, Chen},
  journal = {Robotics},
  year    = {2026},
  note    = {Submitted}
}
```

## License

* Code — MIT License (see `LICENSE`).
* Data and trained-model weights (`data/`, `models/`) — Creative Commons
  Attribution 4.0 International (CC-BY-4.0); see `DATA_LICENSE`.
* Supplementary videos (`videos/`) — CC-BY-4.0.

## Contact

Chen Giladi — Department of Mechanical Engineering, Sami Shamoon College of
Engineering, Ashdod, Israel — `chengi1@sce.ac.il`.
