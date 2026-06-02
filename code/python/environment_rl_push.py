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

# --------------------------------------------------------------------------
# Config: override any of these with an environment variable.
# Defaults make the script importable and produce a working layout under ./logs/.
#   COPPELIASIM_PY_PATH  -> CoppeliaSim's programming/zmqRemoteApi/clients/python
#   RL_PUSH_LOG_DIR      -> directory for training logs, callbacks, and CSV traces
# --------------------------------------------------------------------------
_DEFAULT_COPPELIA_PATHS = [
    r"C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python",
    "/opt/coppeliasim/programming/zmqRemoteApi/clients/python",
    os.path.expanduser("~/CoppeliaSim/programming/zmqRemoteApi/clients/python"),
]

def _resolve_coppeliasim_python_dir():
    env_path = os.environ.get("COPPELIASIM_PY_PATH")
    if env_path and os.path.isdir(env_path):
        return env_path
    for candidate in _DEFAULT_COPPELIA_PATHS:
        if os.path.isdir(candidate):
            return candidate
    return None

log_dir = os.environ.get("RL_PUSH_LOG_DIR", os.path.join(os.getcwd(), "logs", "rl_push"))
csv_file = os.path.join(log_dir, "train_data.csv")
path_to_the_train_data = os.path.join(log_dir, "train_data_")
path_to_the_dist_data  = os.path.join(log_dir, "X_Y_TRAJ_")

_coppeliasim_python_dir = _resolve_coppeliasim_python_dir()
if _coppeliasim_python_dir is None:
    raise RuntimeError(
        "CoppeliaSim Python client directory not found. Set the COPPELIASIM_PY_PATH "
        "environment variable to <CoppeliaSimRoot>/programming/zmqRemoteApi/clients/python "
        "before importing this module."
    )
sys.path.append(os.path.abspath(_coppeliasim_python_dir))
from zmqRemoteApi import RemoteAPIClient

global client, sim, spot, legBase, base, target, spot_script, force_sensors, episode
global action_vec, reward_vec, dist_vec, episode_vec
action_vec = []
reward_vec = []
dist_vec = []
episode_vec = []


client = RemoteAPIClient()
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
CUBE = sim.getObject('/CUBE')
Point_tobe = sim.getObject('./CUBE/Point_to_be_infront')
First_point = sim.getObject('./Center_Spot/First_point')
corner_1 = sim.getObject('./CUBE/Corner_1')
corner_2 = sim.getObject('./CUBE/Corner_2')
corner_3 = sim.getObject('./CUBE/Center_cube_Floor')
R_toReward = sim.getObject('./CUBE/Corner_4')
L_toReward = sim.getObject('./CUBE/Corner_3')
Target_1 = sim.getObject('./Target_1')
Target_2 = sim.getObject('./Target_1/Target_2')


force_sensors = np.array([sim.getObject('/spot_front_left_lower_leg_force_sensor'),
                          sim.getObject('/spot_front_right_lower_leg_force_sensor'),
                          sim.getObject('/spot_rear_left_lower_leg_force_sensor'),
                          sim.getObject('/spot_rear_right_lower_leg_force_sensor')])
spot_script = sim.getScript(1, spot, '/spot')


if not os.path.exists(log_dir):
    os.makedirs(log_dir, exist_ok=True)

gamma = 0.99
vis = visdom.Visdom()
episode = 0

class RobotModelEnv(gym.Env):

    def set_box_mass_kg(self, kg):
        """Set the mass (kg) of the box (CUBE) in the running CoppeliaSim scene; used by the Figure-17 mass-robustness sweep."""
        sim.setShapeMass(CUBE, float(kg))

    def __init__(self, action_type='discrete'):
        super(RobotModelEnv, self).__init__()
        self.dist_1x = []
        self.dist_1y = []
        self.dist_2x = []
        self.dist_2y = []
        self.csv_row = 0
        self.Point_tobe_P = sim.getObjectPosition(Point_tobe, -1)
        self.First_point_P = sim.getObjectPosition(First_point, -1)
        self.corner_1_P = sim.getObjectPosition(corner_1, -1)
        self.corner_2_P = sim.getObjectPosition(corner_2, -1)
        self.corner_3_P = sim.getObjectPosition(corner_3, -1)
        self.R_toReward_P = sim.getObjectPosition(R_toReward, -1)
        self.L_toReward_P = sim.getObjectPosition(L_toReward, -1)
        Target_1_P = sim.getObjectPosition(Target_1, -1)
        self.Target_1_P = [Target_1_P[0], Target_1_P[1]]
        Target_2_P = sim.getObjectPosition(Target_2, -1)
        self.Target_2_P = [Target_2_P[0], Target_2_P[1]]
        self.body_position = np.array(sim.getObjectPosition(spot, -1))
        res, force_FL, torque_FL = sim.readForceSensor(sim.getObject('/spot_front_left_lower_leg_force_sensor'))
        res, force_FR, torque_FR = sim.readForceSensor(sim.getObject('/spot_front_right_lower_leg_force_sensor'))
        res, force_BL, torque_BL = sim.readForceSensor(sim.getObject('/spot_rear_left_lower_leg_force_sensor'))
        res, force_BR, torque_BR = sim.readForceSensor(sim.getObject('/spot_rear_right_lower_leg_force_sensor'))
        self.leg_force_sensors = np.concatenate((force_FL, force_FR, force_BL, force_BR), axis=None, dtype=np.float32)
        self.target = np.array(sim.getObjectPosition(target, -1))
        self.data = np.concatenate((self.body_position, self.leg_force_sensors, self.R_toReward_P, self.L_toReward_P,
                                    self.Target_1_P, self.Target_2_P), axis=None, dtype=np.float32)#25
        high_x_position = np.array([12.5, 12.5, 2], dtype=np.float32)
        low_x_position = np.array([-12.5, -12.5, -0.1], dtype=np.float32)
        high_force_sensors = np.array([500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500], dtype=np.float32)
        low_force_sensors = np.array([-500, -500, -500, -500, -500, -500, -500, -500, -500, -500, -500, -500], dtype=np.float32)
        targer_lowPos = np.array([-12.5, -12.5, -0.1], dtype=np.float32)
        targer_highPos = np.array([12.5, 12.5, 2], dtype=np.float32)
        target_1_pos_min = np.array([-12.5, -12.5], dtype=np.float32)
        target_1_pos_max = np.array([12.5, 12.5], dtype=np.float32)
        target_2_pos_min = np.array([-12.5, -12.5], dtype=np.float32)
        target_2_pos_max = np.array([12.5, 12.5], dtype=np.float32)
        observation_space_min = np.concatenate((low_x_position, low_force_sensors, targer_lowPos, targer_lowPos,
             target_1_pos_min, target_2_pos_min), axis=None, dtype=np.float32)
        observation_space_max = np.concatenate((high_x_position, high_force_sensors, targer_highPos, targer_highPos,
             target_1_pos_max, target_2_pos_max), axis=None, dtype=np.float32)
        self.action_space = spaces.MultiDiscrete([8, 8, 2])
        self.observation_space = spaces.Box(low=observation_space_min, high=observation_space_max, shape=(25,),
                                            dtype=np.float32)
        self.cum_reward = np.array([0])
        self.seed()
        self.counts = 0

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        global episode
        global action_vec, reward_vec, dist_vec, episode_vec
        Point_tobe_pos = np.array(sim.getObjectPosition(Point_tobe, -1))
        First_point_pos = np.array(sim.getObjectPosition(First_point, -1))
        First_point_posxy = np.array([First_point_pos[0], First_point_pos[1]])
        Point_tobe_posxy = np.array([Point_tobe_pos[0], Point_tobe_pos[1]])
        dist_pos_to_rrt = np.linalg.norm(First_point_posxy - Point_tobe_posxy)
        if dist_pos_to_rrt >=0.45:
            print("RRT ")
            print("distance = {}".format(dist_pos_to_rrt))
        X_Bez_1 = (action[0]+1)/10
        X_Bez_2 = (action[1]+1)/10
        leg_num = float(action[2]+1)
        print("X_Bez_1 = {}, X_Bez_2 = {}, leg_num = {}".format(X_Bez_1, X_Bez_2, leg_num))
        res = sim.callScriptFunction('step_Up', spot_script, X_Bez_1, X_Bez_2, leg_num)
        signal = sim.getInt32Signal("execDone1")
        fall_signal = sim.getStringSignal("fall")
        start = default_timer()
        while signal is None:
            duration = default_timer() - start
            signal = sim.getInt32Signal("execDone1")
            fall_signal = sim.getStringSignal("fall")
            if fall_signal:
                break

        sim.clearInt32Signal("execDone1")
        res, force_FL, torque_FL = sim.readForceSensor(sim.getObject('/spot_front_left_lower_leg_force_sensor'))
        res, force_FR, torque_FR = sim.readForceSensor(sim.getObject('/spot_front_right_lower_leg_force_sensor'))
        res, force_BL, torque_BL = sim.readForceSensor(sim.getObject('/spot_rear_left_lower_leg_force_sensor'))
        res, force_BR, torque_BR = sim.readForceSensor(sim.getObject('/spot_rear_right_lower_leg_force_sensor'))
        self.leg_force_sensors = np.concatenate((force_FL, force_FR, force_BL, force_BR), axis=None, dtype=np.float32)
        normal_co = np.max(np.abs(self.leg_force_sensors))
        self.leg_force_sensors = self.leg_force_sensors / normal_co
        self.body_position = sim.getObjectPosition(spot, -1)
        self.Target_1_P = sim.getObjectPosition(Target_1, -1)
        self.Target_2_P = sim.getObjectPosition(Target_2, -1)
        targt_1_xy = np.array([self.Target_1_P[0], self.Target_1_P[1]])
        targt_2_xy = np.array([self.Target_2_P[0], self.Target_2_P[1]])
        self.data = np.concatenate((self.body_position, self.leg_force_sensors, self.R_toReward_P, self.L_toReward_P,
             targt_1_xy, targt_2_xy), axis=None, dtype=np.float32)

        self.counts += 1
        self.csv_row += 1
        fall = bool(fall_signal)

        R_R = sim.getObjectPosition(R_toReward, -1)
        TR_R = sim.getObjectPosition(Target_1, -1)
        dist_1 = np.array([R_R[0], R_R[1]]) - np.array([TR_R[0], TR_R[1]])
        self.dist_1x = np.append(self.dist_1x, np.abs(dist_1[0]))
        self.dist_1y = np.append(self.dist_1y, np.abs(dist_1[1]))
        L_R = sim.getObjectPosition(L_toReward, -1)
        TL_R = sim.getObjectPosition(Target_2, -1)
        dist_2 = np.array([L_R[0], L_R[1]]) - np.array([TL_R[0], TL_R[1]])
        self.dist_2x = np.append(self.dist_2x, np.abs(dist_2[0]))
        self.dist_2y = np.append(self.dist_2y, np.abs(dist_2[1]))
        dist_1_NROM = np.linalg.norm(dist_1)
        dist_2_NROM = np.linalg.norm(dist_2)
        reward = self.Cube_Reward(dist_1_NROM, dist_2_NROM, fall)

        self.cum_reward = np.append(self.cum_reward, self.cum_reward[-1] + reward * gamma)
        if fall or dist_1_NROM<0.1 or dist_2_NROM<0.1 or np.abs(dist_1[0])<0.05 or np.abs(dist_2[0])<0.05:
            done = True
            if np.abs(dist_1[0]) < 0.1:
                if np.abs(dist_1[0]) < 0.1:
                    reward += 100
                    done = True

            episode += 1
            action_vec = np.append(action_vec, self.counts)
            reward_vec = np.append(reward_vec, self.cum_reward[-1])
            episode_vec = np.append(episode_vec, episode)
            if episode % 10 == 0 or episode == 1:
                rows = zip(episode_vec, action_vec, reward_vec)
                file_name = path_to_the_train_data + str(episode) + '_.csv'
                with open(file_name, 'w', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(["Epoch", "Action", "Reward"])  # Header row
                    for row in rows:
                        writer.writerow(row)
                rows1 = zip(self.dist_1x, self.dist_1y, self.dist_2x, self.dist_2y)
                file_name2 = path_to_the_dist_data + str(episode) + '_.csv'
                with open(file_name2, 'w', newline='') as file1:
                    writer2 = csv.writer(file1)
                    writer2.writerow(["dist_1x", "dist_1y", "dist_2x", "dist_2y"])  # Header row
                    for row1 in rows1:
                        writer2.writerow(row1)
                action_vec = action_vec[-1]
                reward_vec = reward_vec[-1]
                # dist_vec = np.append(dist_vec, )
                episode_vec = episode_vec[-1]

            self.dist_1x = 0
            self.dist_1y = 0
            self.dist_2x = 0
            self.dist_2y = 0
            print("-----------------")
            print("EPOCH = " + str(episode))
            print("-----------------")
        else:
            done = False


        print("end of step")
        print("reward = " + str(reward))
        return self.data, reward, done, {}

    def reset(self):
        self.counts = 0
        sim.stopSimulation()
        time.sleep(5)
        sim.startSimulation()
        time.sleep(5)
        self.__init__()
        return self.data

    def render(self):
        return None

    def close(self):
        sim.stopSimulation()  # stop the simulation
        print('Close the environment')
        return None

    def Cube_Reward(self, dist_1_NROM, dist_2_NROM, fall):
        reward = -10*np.abs(dist_1_NROM) - 10*np.abs(dist_2_NROM)
        falling_reward = 0
        if fall:
            falling_reward = -10000
            reward += falling_reward
        if dist_1_NROM<0.1:
            reward += 100
            if dist_2_NROM<0.1:
                reward +=10000

        return reward





