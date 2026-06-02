"""
Drive a CoppeliaSim Spot/Go1 scene along the smoothed RRT path produced by
either the MATLAB planner (RoboDog_RRT_Planner.m) or the Python planner
(rrt_planner.py). The two writers produce the same xlsx column layout
(leg_K_x_diff / leg_K_y_diff / path_angle / leg_{x,y}_{FL,FR,BL,BR}).

Prerequisites:
  - A CoppeliaSim scene `code/scenes/RoboDog_RRT.ttt` is open and the
    simulation is started.
  - The CoppeliaSim ZeroMQ Remote API Python client is on the path.
    Either install it into the Python environment, or export
    COPPELIASIM_PY_PATH to the bundled clients/python directory:
      Linux/macOS: export COPPELIASIM_PY_PATH=/opt/CoppeliaSim/programming/zmqRemoteApi/clients/python
      Windows   : $env:COPPELIASIM_PY_PATH = "C:\\Program Files\\CoppeliaSim\\programming\\zmqRemoteApi\\clients\\python"

Usage:
  python run_rrt_in_coppeliasim.py                          # reads ../../data/RRT_Data.xlsx
  python run_rrt_in_coppeliasim.py --xlsx /path/to/file.xlsx
"""

import argparse
import os
import sys
import time
from pathlib import Path

import pandas as pd

os.environ.setdefault('KMP_DUPLICATE_LIB_OK', 'True')

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XLSX = REPO_ROOT / "data" / "RRT_Data.xlsx"


def _bootstrap_coppeliasim_client():
    """Ensure `zmqRemoteApi` is importable. Honour COPPELIASIM_PY_PATH if set."""
    coppelia_py = os.environ.get("COPPELIASIM_PY_PATH")
    if coppelia_py:
        sys.path.append(os.path.abspath(coppelia_py))
    try:
        from zmqRemoteApi import RemoteAPIClient  # noqa: F401
    except ImportError as e:
        raise SystemExit(
            "Cannot import 'zmqRemoteApi'. Either install the CoppeliaSim ZMQ "
            "Remote API client into this Python environment, or set the "
            "COPPELIASIM_PY_PATH environment variable to the bundled "
            "<coppeliasim>/programming/zmqRemoteApi/clients/python folder."
        ) from e


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xlsx", type=Path, default=DEFAULT_XLSX,
                    help="Path to the planner output (default: ../../data/RRT_Data.xlsx)")
    ap.add_argument("--step-pause", type=float, default=2.0,
                    help="Seconds to wait between leg actuations (default: 2.0)")
    args = ap.parse_args()

    if not args.xlsx.exists():
        raise SystemExit(
            f"Cannot find RRT path file: {args.xlsx}\n"
            "Run rrt_planner.py first (Python) or RoboDog_RRT_Planner.m (MATLAB) "
            "to generate it."
        )

    _bootstrap_coppeliasim_client()
    from zmqRemoteApi import RemoteAPIClient

    client = RemoteAPIClient()
    sim = client.getObject('sim')
    spot = sim.getObject('/spot')
    spot_script = sim.getScript(1, spot, '/spot')
    Center_Spot = sim.getObject('./Center_Spot')

    df = pd.read_excel(args.xlsx, engine='openpyxl')
    v = {name: df[name].values for name in df.columns}

    leg_1_x_diff = v['leg_1_x_diff']
    leg_1_y_diff = v['leg_1_y_diff']
    leg_2_x_diff = v['leg_2_x_diff']
    leg_2_y_diff = v['leg_2_y_diff']
    leg_3_x_diff = v['leg_3_x_diff']
    leg_3_y_diff = v['leg_3_y_diff']
    leg_4_x_diff = v['leg_4_x_diff']
    leg_4_y_diff = v['leg_4_y_diff']
    path_angle   = v['path_angle']
    leg_y_FL, leg_x_FL = v['leg_y_FL'], v['leg_x_FL']
    leg_y_FR, leg_x_FR = v['leg_y_FR'], v['leg_x_FR']
    leg_y_BL, leg_x_BL = v['leg_y_BL'], v['leg_x_BL']
    leg_y_BR, leg_x_BR = v['leg_y_BR'], v['leg_x_BR']

    sim.startSimulation()
    Center_Spot_pos = sim.getObjectPosition(Center_Spot, -1)
    sim.stopSimulation()
    time.sleep(5)
    sim.startSimulation()
    stop_time = args.step_pause
    sim.getInt32Signal("execDone")

    for i in range(len(leg_1_x_diff)):
        sim.callScriptFunction('step', spot_script, leg_4_y_diff[i], leg_4_x_diff[i],
                               0.2, -path_angle[i], 1, leg_y_FL[i], leg_x_FL[i])
        time.sleep(stop_time)
        sim.callScriptFunction('step', spot_script, leg_1_y_diff[i], leg_1_x_diff[i],
                               0.2, -path_angle[i], 2, leg_y_FR[i], leg_x_FR[i])
        time.sleep(stop_time)
        if i < 1000:
            sim.callScriptFunction('step', spot_script, leg_3_y_diff[i], leg_3_x_diff[i],
                                   0.2, -path_angle[i], 3, leg_y_BL[i], leg_x_BL[i])
            time.sleep(stop_time)
            sim.callScriptFunction('step', spot_script, leg_2_y_diff[i], leg_2_x_diff[i],
                                   0.2, -path_angle[i], 4, leg_y_BR[i], leg_x_BR[i])
            time.sleep(stop_time)
        else:
            sim.callScriptFunction('step', spot_script, leg_2_y_diff[i], leg_2_x_diff[i],
                                   0.2, -path_angle[i], 3)
            time.sleep(stop_time)
            sim.callScriptFunction('step', spot_script, leg_3_y_diff[i], leg_3_x_diff[i],
                                   0.2, -path_angle[i], 4)
            time.sleep(stop_time)
        print(f"------------------------ step {i} ------------------------")


if __name__ == "__main__":
    main()
