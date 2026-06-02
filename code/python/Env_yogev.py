from stable_baselines3 import SAC
import os
import csv
import visdom
# from Visdom import VisdomCallback
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
import os
import sys
import time
import csv
import gym
import math
import numpy as np
from gym import spaces
from gym.utils import seeding
import visdom
from timeit import default_timer
from RobotModelEnv_spot import RobotModelEnv
import os
import sys
import time
import csv
import gym
import math
import numpy as np
from gym import spaces
from gym.utils import seeding
import visdom
from timeit import default_timer
import os
import csv
import visdom
# from Visdom import VisdomCallback
import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CallbackList, BaseCallback
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.patches import Rectangle
from RobotModelEnv_spot import RobotModelEnv
import time
import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.patches import Rectangle
from scipy.interpolate import splprep, splev
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'


# class SaveOnBestTrainingRewardCallback(BaseCallback):
#     """
#     Callback for saving a model (the check is done every ``check_freq`` steps)
#     based on the training reward (in practice, we recommend using ``EvalCallback``).
#
#     :param check_freq: (int)
#     :param log_dir: (str) Path to the folder where the model will be saved.
#       It must contains the file created by the ``Monitor`` wrapper.
#     :param verbose: (int)
#     """
#
#     def __init__(self, check_freq: int, log_dir: str, verbose=1):
#         super().__init__(verbose)
#         self.check_freq = check_freq
#         self.log_dir = log_dir
#         self.save_path = os.path.join(log_dir, "best_model")
#         self.best_mean_reward = -np.inf
#
#     def _init_callback(self) -> None:
#         # Create folder if needed
#         if self.save_path is not None:
#             os.makedirs(self.save_path, exist_ok=True)
#
#     def _on_step(self) -> bool:
#         if self.n_calls % self.check_freq == 0:
#
#             # Retrieve training reward
#             x, y = ts2xy(load_results(self.log_dir), "timesteps")
#             if len(x) > 0:
#                 # Mean training reward over the last 100 episodes
#                 mean_reward = np.mean(y[-100:])
#                 if self.verbose > 0:
#                     print(f"Num timesteps: {self.num_timesteps}")
#                     print(
#                         f"Best mean reward: {self.best_mean_reward:.2f} - Last mean reward per episode: {mean_reward:.2f}"
#                     )
#
#                 # New best model, you could save the agent here
#                 if mean_reward > self.best_mean_reward:
#                     self.best_mean_reward = mean_reward
#                     # Example for saving best model
#                     if self.verbose > 0:
#                         print(f"Saving new best model to {self.save_path}.zip")
#                     self.model.save(self.save_path)
#
#         return True


sys.path.append(os.path.abspath("C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python"))
from zmqRemoteApi import RemoteAPIClient

global client, sim, spot, legBase, base, target, spot_script, force_sensors, episode

client = RemoteAPIClient()
# Connect to VREP (CoppeliaSim)
print('Connect to CoppeliaSim')

client = RemoteAPIClient()
sim = client.getObject('sim')
spot = sim.getObject('/spot')
target = sim.getObject('/target')
legBase = sim.getObject('/legBase')
base = sim.getObject('/base')
tips = np.array([sim.getObject('./tip_FL'),
                 sim.getObject('./tip_FR'),
                 sim.getObject('./tip_BL'),
                 sim.getObject('./tip_BR')])

force_sensors = np.array([sim.getObject('/spot_front_left_lower_leg_force_sensor'),
                          sim.getObject('/spot_front_right_lower_leg_force_sensor'),
                          sim.getObject('/spot_rear_left_lower_leg_force_sensor'),
                          sim.getObject('/spot_rear_right_lower_leg_force_sensor')])
spot_script = sim.getScript(1, spot, '/spot')
# tip_FL_target = sim.getObject('/tip_FL_target')
# tip_BR_target = sim.getObject('/tip_BR_target')
# tip_BL_target = sim.getObject('/tip_BL_target')
# tip_FR_target = sim.getObject('/tip_FR_target')

################
# need to add the cube positions #
################
# get the cube position
# Cube = sim.getObject('/CUBE')
# Center_cube_plane = sim.getObject('./Center_cube_plane')
# Rigth_Up_cube_plane = sim.getObject('./Rigth_Up_cube_plane')
# Left_Up_cube_plane = sim.getObject('./Left_Up_cube_plane')
# Left_Down_cube_plane = sim.getObject('./Left_Down_cube_plane')
# Rigth_Down_cube_plane = sim.getObject('./Rigth_Down_cube_plane')
# # Corners
# Corner_1 = sim.getObject('/Corner_1')
# Corner_2 = sim.getObject('/Corner_2')
# Corner_3 = sim.getObject('/Corner_3')
# Corner_4 = sim.getObject('/Corner_4')
# Point_to_be_infront = sim.getObject('/Point_to_be_infront')

Center_Spot = sim.getObject('./Center_Spot')

log_dir = r"C:\Users\yogev\Documents\yogev\Robodog\saved_models\random"

dx = 0.5
dy = 0.5
h = 0.01
leg = 1
print("dx = {}, dy = {}, h = {}, leg id = {}".format(dx, dy, h, leg))
sim.startSimulation()
# res = sim.callScriptFunction('step', spot_script, 1.1, 2.2,3 , 3) # Move
# res = sim.callScriptFunction('step', spot_script, dx, dy, h, leg)  # Move
# signal = sim.getInt32Signal("execDone")
# fall_signal = sim.getStringSignal("fall")
# get Positions
Center_Spot_pos = sim.getObjectPosition(Center_Spot, -1)
# Point_to_be_infront_pos = sim.getObjectPosition(Point_to_be_infront, -1)
# Define the start and goal states
target_angle = np.pi/2
start = (Center_Spot_pos[0], Center_Spot_pos[1], 0) # x, y, theta
goal = (12, 12, target_angle) # x, y, theta
sim.stopSimulation()


# Define the world boundaries
bounds = [(0, 50), (0, 50)]

# Define the step size
step_size = 0.3

# Define the tree data structure as a dictionary
tree = {start: None}
def draw_rectangle(x, y, theta):
    w = 0.32
    h = 0.64
    X = np.array([-w/2, w/2, w/2, -w/2, -w/2])
    Y = np.array([h/2, h/2, -h/2, -h/2, h/2])
    P = np.array([X, Y])

    ct = np.cos(theta)
    st = np.sin(theta)
    R = np.array([[ct, -st], [st, ct]])
    Pr = np.matmul(R, P)
    Pr[0, :] += x
    Pr[1, :] += y
    # plt.figure()
    # plt.plot(Pr[0, :], Pr[1, :])
    # plt.axis('equal')
    # plt.show()
    return Pr
# Define a function to generate random states within the world boundaries
def generate_random_state(bounds):
    x = random.uniform(bounds[0][0], bounds[0][1])
    y = random.uniform(bounds[1][0], bounds[1][1])
    theta = random.uniform(-np.pi, np.pi)
    return (x, y, theta)

# Define a function to find the nearest neighbor
def find_nearest_neighbor(state, tree):
    neighbors = list(tree.keys())
    distances = [np.sqrt((s[0]-state[0])**2 + (s[1]-state[1])**2) for s in neighbors]
    nearest_neighbor = neighbors[np.argmin(distances)]
    return nearest_neighbor

# Define a function to steer towards the random state with a fixed step size
def steer_towards(state1, state2, step_size):
    dist = np.sqrt((state2[0]-state1[0])**2 + (state2[1]-state1[1])**2)
    if dist <= step_size:
        return state2
    else:
        dx = (state2[0]-state1[0])*step_size/dist
        dy = (state2[1]-state1[1])*step_size/dist
        theta = np.arctan2(dy, dx)
        return (state1[0]+dx, state1[1]+dy, theta)

def smooth_path_b_spline(path):
    x = [p[0] for p in path]
    y = [p[1] for p in path]
    tck, _ = splprep([x, y], s=0.5, k=2)  # s is smoothness factor, k is degree of the spline
    u_fine = np.linspace(0, 1, 500)  # Define a fine grid along the spline
    x_fine, y_fine = splev(u_fine, tck)
    return list(zip(x_fine, y_fine))

# Main loop of the RRT algorithm
while True:
    random_state = generate_random_state(bounds)
    nearest_neighbor = find_nearest_neighbor(random_state, tree)
    new_state = steer_towards(nearest_neighbor, random_state, step_size)
    tree[new_state] = nearest_neighbor
    if np.sqrt((new_state[0]-goal[0])**2 + (new_state[1]-goal[1])**2) < step_size:
        tree[goal] = new_state
        break

# Backtrack from the goal to the start to get the path
path = []
current_state = goal
while current_state is not None:
    path.append(current_state)
    current_state = tree[current_state]
path = path[::-1]


# Smooth the path
smoothed_path = smooth_path_b_spline(path)
initial_p = smoothed_path[1]
angle_Vec = []
for i in range(len(smoothed_path)-1):
    delta = np.array(smoothed_path[i+1]) - np.array(smoothed_path[i])
    angle = np.arctan2(delta[1],delta[0])
    angle_Vec = np.append(angle_Vec, angle)
    initial_p = smoothed_path[i+1]
# Define the size of the rectangle
angle_Vec = np.append(angle_Vec, target_angle)
rect_width = 0.9
rect_height = 0.5

# # Create a figure and an axes
# fig, ax = plt.subplots()
#
# # Plot the result
# ax.grid(True)
# ax.set_xlim(bounds[0])
# ax.set_ylim(bounds[1])
# for state, parent in tree.items():
#     if parent is not None:
#         ax.plot([state[0], parent[0]], [state[1], parent[1]], 'b-')
# for i in range(len(path)-1):
#     ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]], 'r-')
# ax.plot([start[0], start[0]], [start[1], start[1]], 'go')
# ax.plot([goal[0], goal[0]], [goal[1], goal[1]], 'ro')
leg_1_p = sim.getObjectPosition(sim.getObject('./tip_FL'), -1)
leg_2_p = sim.getObjectPosition(sim.getObject('./tip_FR'), -1)
leg_3_p = sim.getObjectPosition(sim.getObject('./tip_BL'), -1)
leg_4_p = sim.getObjectPosition(sim.getObject('./tip_BR'), -1)

# create the path for each leg
leg_1_x = [leg_1_p[0]]
leg_1_y = [leg_1_p[1]]
leg_2_x = [leg_2_p[0]]
leg_2_y = [leg_2_p[1]]
leg_3_x = [leg_3_p[0]]
leg_3_y = [leg_3_p[1]]
leg_4_x = [leg_4_p[0]]
leg_4_y = [leg_4_p[1]]
path_optim_ract = []
print(path[1][1])

for vec, j in zip(smoothed_path, range(len(smoothed_path))):
    path_optim_ract = draw_rectangle(vec[0], vec[1], angle_Vec[j])
    # print(path_optim_ract[1][1])
    # print(path_optim_ract[1][2])
    # print(path_optim_ract[1][3])
    # print(path_optim_ract[1][4])
    # print(path_optim_ract[2][1])
    # x_val = vec[0]
    # y_val = vec[1]
    leg_1_x = np.append(leg_1_x, path_optim_ract[0][1])
    leg_1_y = np.append(leg_1_y, path_optim_ract[1][1])
    leg_2_x = np.append(leg_2_x, path_optim_ract[0][2])
    leg_2_y = np.append(leg_2_y, path_optim_ract[1][2])
    leg_3_x = np.append(leg_3_x, path_optim_ract[0][0])
    leg_3_y = np.append(leg_3_y, path_optim_ract[1][0])
    leg_4_x = np.append(leg_4_x, path_optim_ract[0][3])
    leg_4_y = np.append(leg_4_y, path_optim_ract[1][3])
    # leg_1_x = np.append(leg_1_x, x_val + 0.32)
    # leg_1_y = np.append(leg_1_y, y_val + 0.16)
    # leg_2_x = np.append(leg_2_x, x_val + 0.32)
    # leg_2_y = np.append(leg_2_y, y_val - 0.16)
    # leg_3_x = np.append(leg_3_x, x_val - 0.2784)
    # leg_3_y = np.append(leg_3_y, y_val + 0.16)
    # leg_4_x = np.append(leg_4_x, x_val - 0.2784)
    # leg_4_y = np.append(leg_4_y, y_val - 0.16)

# Create a figure and an axes


plt.figure(1)
plt.plot(leg_1_x, leg_1_y, color='red')
plt.plot(leg_2_x, leg_2_y, color='blue')
plt.plot(leg_3_x, leg_3_y, color='green')
plt.plot(leg_4_x, leg_4_y, color='black')
plt.show()



leg_1_x_diff = np.diff(leg_1_x)
leg_1_y_diff = np.diff(leg_1_y)
leg_2_x_diff = np.diff(leg_2_x)
leg_2_y_diff = np.diff(leg_2_y)
leg_3_x_diff = np.diff(leg_3_x)
leg_3_y_diff = np.diff(leg_3_y)
leg_4_x_diff = np.diff(leg_4_x)
leg_4_y_diff = np.diff(leg_4_y)
sim.startSimulation()
stop_time = 2
# move the legs
for i in range(len(leg_1_x_diff)):
    # move leg 1
    res = sim.callScriptFunction('step', spot_script, leg_1_x_diff[i], leg_1_y_diff[i], 0.2, 1)  # Move
    time.sleep(stop_time)
    res = sim.callScriptFunction('step', spot_script, leg_2_x_diff[i], leg_2_y_diff[i], 0.2, 2)  # Move
    time.sleep(stop_time)
    res = sim.callScriptFunction('step', spot_script, leg_4_x_diff[i], leg_4_y_diff[i], 0.2, 4)  # Move
    time.sleep(stop_time)
    res = sim.callScriptFunction('step', spot_script, leg_3_x_diff[i], leg_3_y_diff[i], 0.2, 3)  # Move
    time.sleep(stop_time)

# Create a rectangle at the start state
#rect = Rectangle((start[0]-rect_width/2, start[1]-rect_height/2), rect_width, rect_height,
#                 angle=np.degrees(start[2]), fill=True, color='green')
#ax.add_patch(rect)
rect = Rectangle((start[0]-rect_width/2, start[1]-rect_height/2), rect_width, rect_height,
                 angle=np.degrees(start[2]), fill=True, color='green')
ax.add_patch(rect)
# Define a function to update the rectangle's location and orientation
#def update_rect(rect, state):
#    rect.set_xy((state[0]-rect_width/2, state[1]-rect_height/2))
#    rect.angle = np.degrees(state[2])
#    return rect

# Iterate through the path to move the rectangle
#for state in path:
#    rect = update_rect(rect, state)
#    plt.draw()
#    plt.pause(0.1) # Wait for a while

#plt.show()


# Define a function to update the rectangle's location and orientation
def update_rect(rect, state):
    rect.set_xy((state[0]-rect_width/2, state[1]-rect_height/2))  # rectangle's center at state (x, y)
    rect.angle = np.degrees(state[2])
    return rect

# Iterate through the path to move the rectangle
for state in path:
    rect = update_rect(rect, state)
    plt.draw()
    plt.pause(0.1)  # Wait for a while

plt.show()
# if not os.path.exists(log_dir):
#     os.makedirs(log_dir, exist_ok=True)
#
# csv_file = r'C:\Users\yogev\Documents\yogev\Robodog\saved_models\random\train_data.csv'
# csv_fields = ['step', 'reward', 'fall', 'passed_stair', 'target reach']
# gamma = 0.99
# vis = visdom.Visdom()
# episode = 0
# if not os.path.isfile(csv_file):
#     with open(csv_file, 'w', newline='') as f:
#         writer = csv.writer(f)
#         writer.writerow(csv_fields)
#
#
# class RobotModelEnv(gym.Env):
#     # gym.Env.max_episode_steps = 200
#     """Custom Environment that follows gym interface"""
#
#     # def __init__(self, action_type='discrete'):
#     #     super(RobotModelEnv, self).__init__()
#     #     self.csv_row = 0
#     #     # create CSV file with header row
#     #     self.passed_stair_flag = False
#     #     self.reach_target_flag = False
#     #
#     #     self.body_position = np.array(sim.getObjectPosition(spot, -1))
#     #     tip_FL_pos = sim.getObjectPosition(sim.getObject('./tip_FL'), -1)
#     #     tip_FR_pos = sim.getObjectPosition(sim.getObject('./tip_FR'), -1)
#     #     tip_BL_pos = sim.getObjectPosition(sim.getObject('./tip_BL'), -1)
#     #     tip_BR_pos = sim.getObjectPosition(sim.getObject('./tip_BR'), -1)
#     #     self.leg_tip_positions_world_frame = np.concatenate((tip_FL_pos, tip_FR_pos, tip_BL_pos, tip_BR_pos))
#     #     self.back_leg_position = min(self.leg_tip_positions_world_frame[0], self.leg_tip_positions_world_frame[3],
#     #                                  self.leg_tip_positions_world_frame[6], self.leg_tip_positions_world_frame[9])
#     #     self.average_tips_x_position = (self.leg_tip_positions_world_frame[0] +
#     #                                     self.leg_tip_positions_world_frame[3] +
#     #                                     self.leg_tip_positions_world_frame[6] +
#     #                                     self.leg_tip_positions_world_frame[9]) / 4
#     #     res, force_FL, torque_FL = sim.readForceSensor(sim.getObject('/spot_front_left_lower_leg_force_sensor'))
#     #     res, force_FR, torque_FR = sim.readForceSensor(sim.getObject('/spot_front_right_lower_leg_force_sensor'))
#     #     res, force_BL, torque_BL = sim.readForceSensor(sim.getObject('/spot_rear_left_lower_leg_force_sensor'))
#     #     res, force_BR, torque_BR = sim.readForceSensor(sim.getObject('/spot_rear_right_lower_leg_force_sensor'))
#     #
#     #     self.leg_force_sensors = np.concatenate(
#     #         (force_FL, force_FR, force_BL, force_BR), axis=None, dtype=np.float32)
#     #
#     #     self.target = np.array(sim.getObjectPosition(target, -1))
#     #     self.data = np.concatenate((self.average_tips_x_position, self.body_position, self.leg_force_sensors,
#     #                                 self.leg_tip_positions_world_frame, self.target), axis=None, dtype=np.float32)
#     #
#     #     high_position = np.array([2, 1, 0.7], dtype=np.float32)
#     #     low_position = np.array([-2, -1, -0.01], dtype=np.float32)
#     #
#     #     high_x_position = np.array([2.0], dtype=np.float32)
#     #     low_x_position = np.array([-2.0], dtype=np.float32)
#     #
#     #     high_force_sensors = np.array([500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500],
#     #                                   dtype=np.float32)
#     #     low_force_sensors = np.array(
#     #         [-500, -500, -500, -500, -500, -500, -500, -500, -500, -500, -500, -500], dtype=np.float32)
#     #     low_tip_positions = low_force_sensors
#     #     high_tip_positions = high_force_sensors
#     #
#     #     observation_space_min = np.concatenate(
#     #         (low_x_position, low_position, low_force_sensors, low_tip_positions,
#     #          low_position), axis=None,
#     #         dtype=np.float32)
#     #     observation_space_max = np.concatenate(
#     #         (high_x_position, high_position, high_force_sensors,
#     #          high_tip_positions, high_position), axis=None,
#     #         dtype=np.float32)
#     #
#     #     # action space - dx, dh, leg
#     #     high_action = np.array([0.2, 0.05, 0.2, 4.999], dtype=np.float32)
#     #     low_action = np.array([0.05, -0.05, 0.1, 1], dtype=np.float32)
#     #
#     #     self.action_space = spaces.Box(low=low_action, high=high_action, shape=(4,), dtype=np.float32)
#     #     self.observation_space = spaces.Box(low=observation_space_min, high=observation_space_max, shape=(31,),
#     #                                         dtype=np.float32)
#     #     self.cum_reward = np.array([0])
#     #     self.seed()
#     #     self.counts = 0
#     #     self.steps_beyond_done = None
#     #
#     # def seed(self, seed=None):
#     #     self.np_random, seed = seeding.np_random(seed)
#     #     return [seed]
#     #
#     # def step(self, action):
#     #     global episode
#     #     dx = float(action[0])
#     #     dy = float(action[1])
#     #     h = float(action[2])
#     #     leg = int(action[3])
#     #     print("dx = {}, dy = {}, h = {}, leg id = {}".format(dx, dy, h, leg))
#     #
#     #     # res = sim.callScriptFunction('step', spot_script, 1.1, 2.2,3 , 3) # Move
#     #     res = sim.callScriptFunction('step', spot_script, dx, dy, h, leg)  # Move
#     #     signal = sim.getInt32Signal("execDone")
#     #     fall_signal = sim.getStringSignal("fall")
#     #
#     #     start = default_timer()
#     #
#     #     while signal is None:
#     #         duration = default_timer() - start
#     #         signal = sim.getInt32Signal("execDone")
#     #         fall_signal = sim.getStringSignal("fall")
#     #         if duration > 15:
#     #             fall_signal = True
#     #         if fall_signal:
#     #             break
#     #
#     #     sim.clearInt32Signal("execDone")
#     #     res, force_FL, torque_FL = sim.readForceSensor(sim.getObject('/spot_front_left_lower_leg_force_sensor'))
#     #     res, force_FR, torque_FR = sim.readForceSensor(sim.getObject('/spot_front_right_lower_leg_force_sensor'))
#     #     res, force_BL, torque_BL = sim.readForceSensor(sim.getObject('/spot_rear_left_lower_leg_force_sensor'))
#     #     res, force_BR, torque_BR = sim.readForceSensor(sim.getObject('/spot_rear_right_lower_leg_force_sensor'))
#     #     self.leg_force_sensors = np.concatenate(
#     #         (force_FL, force_FR, force_BL, force_BR), axis=None, dtype=np.float32)
#     #     normal_co = np.max(np.abs(self.leg_force_sensors))
#     #     self.leg_force_sensors = self.leg_force_sensors / normal_co
#     #
#     #     self.body_position = sim.getObjectPosition(spot, -1)
#     #
#     #     tip_FL_pos = sim.getObjectPosition(sim.getObject('./tip_FL'), -1)
#     #     tip_FR_pos = sim.getObjectPosition(sim.getObject('./tip_FR'), -1)
#     #     tip_BL_pos = sim.getObjectPosition(sim.getObject('./tip_BL'), -1)
#     #     tip_BR_pos = sim.getObjectPosition(sim.getObject('./tip_BR'), -1)
#     #     self.leg_tip_positions_world_frame = np.concatenate((tip_FL_pos, tip_FR_pos, tip_BL_pos, tip_BR_pos))
#     #     average_tips_x_position_old = self.average_tips_x_position
#     #     self.average_tips_x_position = (self.leg_tip_positions_world_frame[0] +
#     #                                     self.leg_tip_positions_world_frame[3] +
#     #                                     self.leg_tip_positions_world_frame[6] +
#     #                                     self.leg_tip_positions_world_frame[9]) / 4
#     #
#     #     self.data = np.concatenate(
#     #         (self.average_tips_x_position, self.body_position, self.leg_force_sensors,
#     #          self.leg_tip_positions_world_frame, self.target),
#     #         axis=None, dtype=np.float32)
#     #
#     #     self.counts += 1
#     #     self.csv_row += 1
#     #     fall = bool(fall_signal)
#     #     # limit to 50 actions
#     #     if self.counts > 50:
#     #         fall = True
#     #
#     #     # if all legs passed the stair
#     #     self.back_leg_position = min(self.leg_tip_positions_world_frame[0], self.leg_tip_positions_world_frame[3],
#     #                                  self.leg_tip_positions_world_frame[6], self.leg_tip_positions_world_frame[9])
#     #
#     #     reward = self.reward_function_18_05_try_2(leg, fall, average_tips_x_position_old)
#     #
#     #     # plotting and saving data
#     #     self.cum_reward = np.append(self.cum_reward, self.cum_reward[-1] + reward * gamma)
#     #     if fall or self.reach_target_flag or self.passed_stair_flag:
#     #         done = True
#     #         # '''plot the cum reward as function of step once in 250 episodes'''
#     #         # x = list(range(len(self.cum_reward)))
#     #         # y = self.cum_reward
#     #         # opts = {'title': 'Cumulative Rewards', 'xlabel': 'Step', 'ylabel': 'Cumulative Reward'}
#     #         #
#     #         # if episode % 50 == 0:
#     #         #     vis.line(X=x, Y=y, opts=opts, win='cum_rewards_19.05.23_try_1', update='append', name='%s' % episode)
#     #
#     #         '''save data in csv file'''
#     #         with open(csv_file, 'a', newline='') as f:
#     #             writer = csv.writer(f)
#     #             writer.writerow(
#     #                 [self.csv_row, self.cum_reward[-1], fall, self.passed_stair_flag, self.reach_target_flag])
#     #             # if episode % 25 == 0:
#     #             #     vis.scatter(X=[episode], Y=[self.cum_reward[-1]], name='%s' % episode, update='append',
#     #             #                 win='reward_per_episode_19.05.23_try_1', opts=dict(markercolor=np.array([[0, 0, 1]]),
#     #             #                                                                    markersize=5, xlabel='Episode',
#     #             #                                                                    ylabel='Reward',
#     #             #                                                                    title='Reward per episode'))
#     #         print(self.cum_reward)
#     #         episode += 1
#     #     else:
#     #         done = False
#     #
#     #     print("end of step")
#     #     print("reward = " + str(reward))
#     #     return self.data, reward, done, {}
#
#     def reset(self):
#         self.counts = 0
#         sim.stopSimulation()
#         time.sleep(5)
#         sim.startSimulation()
#         time.sleep(5)
#         self.__init__()
#         return self.data
#
#     def render(self):
#         return None
#
#     def close(self):
#         sim.stopSimulation()  # stop the simulation
#         print('Close the environment')
#         return None
#
#     # # reward על התקדמות במקום על מיקום
#     # def reward_function_19_05_23_try_1(self, leg, fall, average_tips_x_position_old):
#     #     delta_x = self.average_tips_x_position - average_tips_x_position_old
#     #     print("delta_x = {}".format(delta_x))
#     #     delta_x_reward = np.exp(2*delta_x) - 1
#     #     reward = delta_x_reward
#     #     print("reward = {}".format(reward))
#     #     y_reward = 0
#     #     if np.abs((self.body_position[1] - self.target[1])) > 0.25:
#     #         y_reward = - 0.3
#     #         print("received y_reward = {}".format(y_reward))
#     #         reward += y_reward
#     #     # robot passed the stair
#     #     passed_stair_reward = 0
#     #     if self.back_leg_position > 0.540 and self.passed_stair_flag == False:
#     #         self.passed_stair_flag = True
#     #         passed_stair_reward = 100 / self.counts
#     #         reward += passed_stair_reward
#     #         print("passed_stair_reward is received = {}".format(passed_stair_reward))
#     #         print("self.counts = {}".format(self.counts))
#     #     # reach target
#     #     reach_target_reward = 0
#     #     if self.back_leg_position > self.target[0] and self.reach_target_flag == False:
#     #         self.reach_target_flag = True
#     #         reach_target_reward = 400 / self.counts
#     #         reward += reach_target_reward
#     #         print("reach_target_reward is received = {}".format(reach_target_reward))
#     #         print("self.counts = {}".format(self.counts))
#     #     falling_reward = 0
#     #     if fall:
#     #         falling_reward = - 1
#     #         print("falling reward = {}".format(falling_reward))
#     #         reward = falling_reward
#     #     print("reach_target_reward = {}, passed_stair_reward = {}, falling_reward = {}, x_reward = {}, y_reward = {}"
#     #           .format(reach_target_reward, passed_stair_reward, falling_reward, delta_x_reward, y_reward))
#     #     return reward
#
#     # def reward_function_18_05_try_2(self, leg, fall, average_tips_x_position_old):
#     #     x_reward = (10 * np.exp(5 * (self.average_tips_x_position / 1.5) - 5))
#     #     reward = x_reward
#     #     # if the robot stay in place he doesn't get the x reward
#     #     not_moving_forward_reward = 0
#     #     if average_tips_x_position_old - self.average_tips_x_position > - 0.015:
#     #         not_moving_forward_reward = - 0.1
#     #         reward = not_moving_forward_reward
#     #     y_reward = 0
#     #     if np.abs((self.body_position[1] - self.target[1])) > 0.25:
#     #         y_reward = - 0.3
#     #         reward += y_reward
#     #
#     #     # robot passed the stair
#     #     passed_stair_reward = 0
#     #     if self.back_leg_position > 0.540 and self.passed_stair_flag == False:
#     #         self.passed_stair_flag = True
#     #         passed_stair_reward = 200 / self.counts
#     #         reward = passed_stair_reward
#     #     # reach target
#     #     reach_target_reward = 0
#     #     if self.back_leg_position > self.target[0] and self.reach_target_flag == False:
#     #         self.reach_target_flag = True
#     #         reach_target_reward = 400 / self.counts
#     #         reward = reach_target_reward
#     #     falling_reward = 0
#     #     if fall:
#     #         falling_reward = -0.3
#     #         reward += falling_reward
#     #
#     #     print("reach_target_reward = {}, passed_stair_reward = {}, falling_reward = {}, x_reward = {}, y_reward = {}, "
#     #           "not_moving_forward_reward = {}".format(reach_target_reward, passed_stair_reward, falling_reward,
#     #                                                   x_reward, y_reward, not_moving_forward_reward))
#     #     return reward
#
# # log_dir = r"C:\Users\yogev\Documents\yogev\Robodog\saved_models\random"
# #
# # if not os.path.exists(log_dir):
# #     os.makedirs(log_dir, exist_ok=True)
# # ---------------- Create environment
# # env = RobotModelEnv(action_type='continuous')
# # env = Monitor(env, log_dir)
# #
# # # '''new model + train'''
# # print("create a new model")
# # policy_kwargs = dict(net_arch=[512, 512, 512, 512])
# # model = SAC(policy="MlpPolicy", env=env, learning_rate=0.0003, verbose=True, gamma=0.99, batch_size=512,
# #             policy_kwargs=policy_kwargs)
# #
# #
# # checkpoint_callback = CheckpointCallback(save_freq=500, save_path=log_dir)
# # callback_best_reward = SaveOnBestTrainingRewardCallback(check_freq=250, log_dir=log_dir)
# #
# # callback = CallbackList([checkpoint_callback, callback_best_reward])
# #
# # model.learn(total_timesteps=1e6, callback=callback, progress_bar=True)
#
# # print('---------------------------------------------------Finished--------------------------------------------------')
#
# '''load the model and predict'''
# # Option 2: load the model from files (note that the loaded model can be learned again)
# # print("load the model from files")
# # path = r"C:\Users\Owner\Documents\shirelle\RobotDog\saved_models\08.05.23_try_1\rl_model_23500_steps.zip"
# # model = SAC.load(path=path, env=env)
# # model.learn(total_timesteps=1e6, callback=callback, progress_bar=True)
#
# '''plot the reward per episode as function of time'''
# # plot_results([log_dir], 1e5, results_plotter.X_TIMESTEPS, "SAC")
# # plt.show()
#
# # print('Prediction')
# #
# # for _ in range(100):
# #     observation, done = env.reset(), False
# #     episode_reward = 0.0
# #
# #     while not done:
# #         action, _state = model.predict(observation, deterministic=True)
# #         observation, reward, done, info = env.step(action)
# #         if done:
# #             env.reset()
# #         episode_reward += reward
# #
# #     print([episode_reward, env.counts])
# #
# # env.close()
#
#
# # action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(5), sigma=float(0.7) * np.ones(12))
# # model = DDPG(policy='MlpPolicy', env=env, learning_rate=1e-3, verbose=True, action_noise=action_noise, gamma=0.95)
#
# # vis.line(Y=model.predict(np.array([[0, 0, 0]])), X=np.array([1]), opts={'title': 'Predicted Value'})
#
# # del model # delete the model and load the best model to predict
# # model = A2C.load("../CartPole/saved_models/tmp/best_model", env=env)
