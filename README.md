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
journal. The canonical repository URL referenced from the paper's Data
Availability Statement is
<https://github.com/ChenGiladi/quadruped-leg-manipulation-rrt-rl>.

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

## Installation

The framework spans three components — CoppeliaSim (simulator), MATLAB
(planner) and Python (RL controller) — and each must be installed
separately. The steps below were validated on Windows 10 and reproduce on
Ubuntu 22.04 as well.

| Component | Version | Source |
|---|---|---|
| Python | 3.10.11 | <https://www.python.org/downloads/release/python-31011/> |
| CoppeliaSim EDU | 4.5.x (MuJoCo engine) | <https://www.coppeliarobotics.com/downloads> |
| MATLAB | R2023a or newer | <https://www.mathworks.com/downloads/> |

### 1. Clone the repository

```bash
git clone https://github.com/ChenGiladi/quadruped-leg-manipulation-rrt-rl.git
cd quadruped-leg-manipulation-rrt-rl
```

### 2. Python environment

The trained policy in `models/best_model8800.zip` is bound to specific
package versions; using a virtual environment is strongly recommended so
those pinned versions do not collide with other projects.

```bash
# Create and activate a Python 3.10 virtual environment
python3.10 -m venv .venv
source .venv/bin/activate          # Linux / macOS
# .venv\Scripts\activate           # Windows PowerShell

# Install the pinned dependencies
pip install --upgrade pip
pip install -r code/python/requirements.txt
```

`stable-baselines3==1.8.0`, `gym==0.21.0`, `torch==2.0.1` (CPU) and
`numpy==1.23.5` are the **exact** versions baked into the saved policy and
must not be upgraded if you want to load `best_model8800.zip` without
warnings. See `models/README.md` for the full version manifest and SHA-256.

Smoke test the install:

```bash
python -c "import stable_baselines3, gym, torch; \
print('SB3', stable_baselines3.__version__, '| Gym', gym.__version__, \
'| Torch', torch.__version__)"
# Expected:  SB3 1.8.0 | Gym 0.21.0 | Torch 2.0.1+cpu
```

### 3. CoppeliaSim

1. Download and install **CoppeliaSim EDU 4.5.x** from
   <https://www.coppeliarobotics.com/downloads>.
2. The Python ZeroMQ Remote API client is bundled with CoppeliaSim. Expose
   it to your Python environment by setting `COPPELIASIM_PY_PATH` to the
   bundled clients directory before launching any of the Python scripts:

   ```bash
   # Linux / macOS
   export COPPELIASIM_PY_PATH="/path/to/CoppeliaSim/programming/zmqRemoteApi/clients/python"

   # Windows PowerShell
   $env:COPPELIASIM_PY_PATH = "C:\Program Files\CoppeliaSim\programming\zmqRemoteApi\clients\python"
   ```
3. Open one of the scene files in `code/scenes/` (`RoboDog_RRT.ttt` for the
   RRT path-following demo or `RoboDog_Learning_Push.ttt` for the PPO
   pushing task) and press **Start** to launch the simulation.

### 4. MATLAB

Open MATLAB R2023a or newer. The RRT planner requires two add-on toolboxes:

* **Robotics System Toolbox** — used by `plannerRRT`, `validatorOccupancyMap`,
  `occupancyMap` in `code/matlab/RoboDog_RRT_Planner.m`.
* **Navigation Toolbox** — used by `optimizePath`, `stateSpaceSE2`.

Verify both are installed by running `ver` at the MATLAB prompt and looking
for the two toolboxes in the output. Then add the `code/matlab/` folder to
the MATLAB path (right-click → Add to Path → Selected Folders and Subfolders)
and you are ready to run the planner scripts.

## Reproducing the main results

1. **Plan an RRT path (MATLAB).** Open `code/matlab/RoboDog_RRT_Planner.m`,
   adjust `start` / `goal` if desired, run. The script writes
   `data/RRT_Data.xlsx`, which the CoppeliaSim side reads.

2. **Launch the simulator.** Open `code/scenes/RoboDog_RRT.ttt` (for RRT
   path-following) or `code/scenes/RoboDog_Learning_Push.ttt` (for the PPO
   pushing task) in CoppeliaSim and start the simulation.

3. **Run the controller (Python).**
   * Inference with the trained policy:
     `python code/python/run_rrt_in_coppeliasim.py` after loading
     `models/best_model8800.zip` (see `models/README.md` for the exact load
     snippet).
   * Retrain from scratch: `python code/python/learning_rl_push.py` — uses
     `environment_rl_push.py` as the Gym environment.

4. **Regenerate the data-driven figures.**
   ```
   cd code/python
   python regenerate_r2_figures.py --figure all          # Figs 16, 21
   python regenerate_r2_figures.py --figure straight_gallery
   python regenerate_r2_figures.py --figure bezier_panelb
   ```

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
