"""
Python alternative to the MATLAB RRT planner (RoboDog_RRT_Planner.m).

With this script, the path-planning side of the pipeline can be reproduced
without MATLAB or the Robotics System / Navigation Toolbox: it builds the
same 60x60 occupancy map as Coppelia_map.m, runs an RRT* on SE(2), smooths
the result, and writes data/RRT_Data.xlsx in the column layout that
run_rrt_in_coppeliasim.py consumes.

The RRT*/shortcut-smoothing structure is inspired by Atsushi Sakai's
PythonRobotics library (https://github.com/AtsushiSakai/PythonRobotics,
BSD-3); the implementation here is self-contained and tailored to this
project's occupancy map and footprint conventions.

Usage:
    python rrt_planner.py                            # defaults match the MATLAB script
    python rrt_planner.py --plot                     # also render the path
    python rrt_planner.py --start 4.2 23.2 1.5708 \\
                          --goal  10  12  1.5708 \\
                          --seed 100
"""

import argparse
import math
from pathlib import Path

import numpy as np
import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "data"

# Body footprint used by DrawRectangle.m (Go1 chassis bounding rectangle, metres)
BODY_W = 0.5963
BODY_H = 0.3346

# Initial planar foot positions, matching RoboDog_RRT_Planner.m lines 83-86
LEG_INIT = {
    "FL": (4.4922, 1.9581),
    "FR": (4.5209, 1.6238),
    "BL": (3.8967, 1.9275),
    "BR": (3.9137, 1.5931),
}

# DrawRectangle corner ordering: corner 1 = back-left, 2 = front-left,
# 3 = front-right, 4 = back-right. The runner pairs corner 4 -> FL,
# corner 1 -> FR, corner 3 -> BL, corner 2 -> BR (see run_rrt_in_coppeliasim.py).
CORNER_FOR_LEG = {"FL": 3, "FR": 0, "BL": 2, "BR": 1}

DEFAULT_START = (4.2030, 23.2024, math.pi / 2)
DEFAULT_GOAL = (10.0, 12.0, math.pi / 2)


def build_occupancy_map():
    """Port of Coppelia_map.m: 60x60 grid with outer walls + three obstacles."""
    H, W = 60, 60
    grid = np.zeros((H, W), dtype=np.uint8)
    grid[0, :] = grid[-1, :] = 1
    grid[:, 0] = grid[:, -1] = 1
    for j in range(15):
        grid[9, j] = 1
        grid[10, j] = 1
    for j in range(30):
        grid[49, j] = 1
        grid[50, j] = 1
    for j in range(19, 60):
        grid[29, j] = 1
        grid[30, j] = 1
    return grid


def is_point_free(grid, x, y, inflate=0.75):
    H, W = grid.shape
    if not (0 <= x < W and 0 <= y < H):
        return False
    r = int(math.ceil(inflate))
    cx, cy = int(x), int(y)
    r2 = inflate * inflate
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            xx, yy = cx + dx, cy + dy
            if 0 <= xx < W and 0 <= yy < H and grid[yy, xx] == 1:
                if dx * dx + dy * dy <= r2 + 1e-9:
                    return False
    return True


def segment_free(grid, p1, p2, inflate=0.75, step=0.05):
    d = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    n = max(int(d / step), 1)
    for k in range(n + 1):
        t = k / n
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        if not is_point_free(grid, x, y, inflate):
            return False
    return True


def rrt_star(grid, start, goal, max_iter=5000, step_size=1.5,
             goal_bias=0.05, rewire_radius=3.0, inflate=0.75, seed=100):
    rng = np.random.default_rng(seed)
    H, W = grid.shape

    nodes = [(start[0], start[1])]
    parents = [-1]
    costs = [0.0]

    def nearest(pt):
        d2 = [(n[0] - pt[0]) ** 2 + (n[1] - pt[1]) ** 2 for n in nodes]
        return int(np.argmin(d2))

    def steer(p_from, p_to, step):
        dx, dy = p_to[0] - p_from[0], p_to[1] - p_from[1]
        d = math.hypot(dx, dy)
        if d <= step:
            return (p_to[0], p_to[1])
        return (p_from[0] + step * dx / d, p_from[1] + step * dy / d)

    goal_idx = None
    for _ in range(max_iter):
        if rng.random() < goal_bias:
            sample = (goal[0], goal[1])
        else:
            sample = (float(rng.uniform(0, W)), float(rng.uniform(0, H)))

        i_near = nearest(sample)
        p_new = steer(nodes[i_near], sample, step_size)

        if not is_point_free(grid, *p_new, inflate=inflate):
            continue
        if not segment_free(grid, nodes[i_near], p_new, inflate=inflate):
            continue

        new_cost = costs[i_near] + math.hypot(p_new[0] - nodes[i_near][0],
                                              p_new[1] - nodes[i_near][1])
        best_parent = i_near
        for j, n in enumerate(nodes):
            d = math.hypot(p_new[0] - n[0], p_new[1] - n[1])
            if d < rewire_radius and segment_free(grid, n, p_new, inflate=inflate):
                c = costs[j] + d
                if c < new_cost:
                    new_cost = c
                    best_parent = j

        nodes.append(p_new)
        parents.append(best_parent)
        costs.append(new_cost)
        new_idx = len(nodes) - 1

        for j, n in enumerate(nodes[:-1]):
            d = math.hypot(n[0] - p_new[0], n[1] - p_new[1])
            if d < rewire_radius and segment_free(grid, p_new, n, inflate=inflate):
                if new_cost + d < costs[j]:
                    parents[j] = new_idx
                    costs[j] = new_cost + d

        if math.hypot(p_new[0] - goal[0], p_new[1] - goal[1]) < step_size:
            if segment_free(grid, p_new, (goal[0], goal[1]), inflate=inflate):
                nodes.append((goal[0], goal[1]))
                parents.append(new_idx)
                costs.append(new_cost + math.hypot(p_new[0] - goal[0],
                                                   p_new[1] - goal[1]))
                goal_idx = len(nodes) - 1
                break

    if goal_idx is None:
        d2 = [(n[0] - goal[0]) ** 2 + (n[1] - goal[1]) ** 2 for n in nodes]
        goal_idx = int(np.argmin(d2))

    path = []
    i = goal_idx
    while i != -1:
        path.append(nodes[i])
        i = parents[i]
    path.reverse()
    return np.array(path)


def shortcut_smooth(grid, path, inflate=0.75, iters=300, seed=100):
    rng = np.random.default_rng(seed)
    pts = [tuple(p) for p in path.tolist()]
    for _ in range(iters):
        if len(pts) < 3:
            break
        i = int(rng.integers(0, len(pts) - 2))
        j = int(rng.integers(i + 2, len(pts)))
        if segment_free(grid, pts[i], pts[j], inflate=inflate):
            pts = pts[: i + 1] + pts[j:]
    return np.array(pts)


def densify_spline(path_xy, factor=3, min_states=30):
    """Cubic-spline resample of (x, y) to roughly `factor x` the input length."""
    from scipy.interpolate import CubicSpline
    n_in = path_xy.shape[0]
    n_out = max(n_in * factor, min_states)
    cum = np.zeros(n_in)
    for k in range(1, n_in):
        cum[k] = cum[k - 1] + math.hypot(path_xy[k, 0] - path_xy[k - 1, 0],
                                         path_xy[k, 1] - path_xy[k - 1, 1])
    cs_x = CubicSpline(cum, path_xy[:, 0])
    cs_y = CubicSpline(cum, path_xy[:, 1])
    s = np.linspace(0.0, float(cum[-1]), n_out)
    return np.column_stack([cs_x(s), cs_y(s)])


def headings_from_path(path_xy, start_yaw, goal_yaw):
    th = np.empty(path_xy.shape[0])
    th[0] = start_yaw
    th[-1] = goal_yaw
    for i in range(1, path_xy.shape[0] - 1):
        dx = path_xy[i + 1, 0] - path_xy[i - 1, 0]
        dy = path_xy[i + 1, 1] - path_xy[i - 1, 1]
        th[i] = math.atan2(dy, dx)
    return th


def footprint_corners(x, y, theta):
    """DrawRectangle.m port: 4 corners of the rotated body rectangle."""
    w, h = BODY_W, BODY_H
    P = np.array([[-w/2, w/2, w/2, -w/2],
                  [h/2,  h/2, -h/2, -h/2]])
    ct, st = math.cos(theta), math.sin(theta)
    R = np.array([[ct, -st], [st, ct]])
    Pr = R @ P
    Pr[0, :] += x
    Pr[1, :] += y
    return Pr  # shape (2, 4)


def build_xlsx_dataframe(states):
    n = states.shape[0]
    leg_x = np.zeros((n, 4))
    leg_y = np.zeros((n, 4))
    for i in range(n):
        Pr = footprint_corners(states[i, 0], states[i, 1], states[i, 2])
        leg_x[i] = Pr[0]
        leg_y[i] = Pr[1]

    # MATLAB writes: leg_K_x_t_t = diff(leg_K_x); leg_K_y_t_t = diff(25 - leg_K_y)
    dx = np.diff(leg_x, axis=0)
    dy = np.diff(25.0 - leg_y, axis=0)

    # path_angle: per-step heading delta in radians, with 0 at the first row
    rel = states[:, 2] - math.pi / 2
    pa_diff = np.diff(rel)
    pa = np.concatenate([[0.0], pa_diff[1:]])  # length n-2
    L = min(dx.shape[0], len(pa))
    dx, dy, pa = dx[:L], dy[:L], pa[:L]

    cum = {k: np.zeros((L, 2)) for k in CORNER_FOR_LEG}
    for leg, ci in CORNER_FOR_LEG.items():
        p = list(LEG_INIT[leg])
        for i in range(L):
            p[0] += dy[i, ci]
            p[1] += dx[i, ci]
            cum[leg][i, 0] = p[0]
            cum[leg][i, 1] = p[1]

    return pd.DataFrame({
        "leg_1_x_diff": dx[:, 0], "leg_1_y_diff": dy[:, 0],
        "leg_2_x_diff": dx[:, 1], "leg_2_y_diff": dy[:, 1],
        "leg_3_x_diff": dx[:, 2], "leg_3_y_diff": dy[:, 2],
        "leg_4_x_diff": dx[:, 3], "leg_4_y_diff": dy[:, 3],
        "path_angle":  pa,
        "leg_y_FL": cum["FL"][:, 1], "leg_x_FL": cum["FL"][:, 0],
        "leg_y_FR": cum["FR"][:, 1], "leg_x_FR": cum["FR"][:, 0],
        "leg_y_BL": cum["BL"][:, 1], "leg_x_BL": cum["BL"][:, 0],
        "leg_y_BR": cum["BR"][:, 1], "leg_x_BR": cum["BR"][:, 0],
    })


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--start", nargs=3, type=float, default=list(DEFAULT_START),
                    metavar=("X", "Y", "YAW"))
    ap.add_argument("--goal", nargs=3, type=float, default=list(DEFAULT_GOAL),
                    metavar=("X", "Y", "YAW"))
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--max-iter", type=int, default=5000)
    ap.add_argument("--inflate", type=float, default=0.75,
                    help="Obstacle safety margin in occupancy cells (matches "
                         "MATLAB's ObstacleSafetyMargin = 0.75)")
    ap.add_argument("--out", type=Path, default=DATA_DIR / "RRT_Data.xlsx")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    start = tuple(args.start)
    goal = tuple(args.goal)

    grid = build_occupancy_map()
    print(f"[plan] map  : 60x60 occupancy with 3 internal obstacles")
    print(f"[plan] start: {start}    goal: {goal}    seed: {args.seed}")

    raw = rrt_star(grid, start=start, goal=goal,
                   max_iter=args.max_iter, inflate=args.inflate, seed=args.seed)
    print(f"[plan] raw RRT* path     : {raw.shape[0]} states")

    short = shortcut_smooth(grid, raw, inflate=args.inflate, seed=args.seed)
    print(f"[plan] after shortcutting: {short.shape[0]} states")

    dense = densify_spline(short, factor=3)
    print(f"[plan] after spline      : {dense.shape[0]} states")

    yaws = headings_from_path(dense, start[2], goal[2])
    states = np.column_stack([dense, yaws])

    df = build_xlsx_dataframe(states)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    df.to_excel(args.out, index=False, engine="openpyxl")
    print(f"[plan] wrote {args.out}  ({len(df)} rows, {len(df.columns)} cols)")

    if args.plot:
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(grid, origin="lower", cmap="Greys", alpha=0.6)
        ax.plot(raw[:, 0], raw[:, 1], "b.-", lw=0.6, ms=3, label="raw RRT*")
        ax.plot(dense[:, 0], dense[:, 1], "r-", lw=1.8, label="smoothed")
        ax.plot(start[0], start[1], "go", ms=10, label="start")
        ax.plot(goal[0], goal[1], "rx", ms=12, mew=2, label="goal")
        ax.set_xlim(0, 60)
        ax.set_ylim(0, 60)
        ax.set_aspect("equal")
        ax.legend(loc="upper right")
        ax.set_title("Python RRT* planner (MATLAB-free alternative)")
        plt.show()


if __name__ == "__main__":
    main()
