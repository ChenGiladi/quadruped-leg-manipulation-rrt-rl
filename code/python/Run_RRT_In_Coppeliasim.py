import os
import sys
import os
import time
import numpy as np
import random
import pandas as pd

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

sys.path.append(os.path.abspath("path\to\your\coppelisim\and\python\folder"))
from zmqRemoteApi import RemoteAPIClient
global client, sim, spot, base, spot_script, episode
client = RemoteAPIClient()
sim = client.getObject('sim')
spot = sim.getObject('/spot')
spot_script = sim.getScript(1, spot, '/spot')
Center_Spot = sim.getObject('./Center_Spot')
file_path = r'path\to\your\csv\file\RRT_YYY.xlsx'
df = pd.read_excel(file_path, engine='openpyxl')

vectors = {}
for col_name in df.columns:
    vectors[col_name] = df[col_name].values

leg_1_x_diff = vectors['leg_1_x_t_yogev']
leg_1_y_diff = vectors['leg_1_y_t_yogev']
leg_2_x_diff = vectors['leg_2_x_t_yogev']
leg_2_y_diff = vectors['leg_2_y_t_yogev']
leg_3_x_diff = vectors['leg_3_x_t_yogev']
leg_3_y_diff = vectors['leg_3_y_t_yogev']
leg_4_x_diff = vectors['leg_4_x_t_yogev']
leg_4_y_diff = vectors['leg_4_y_t_yogev']
path_angle   = vectors['path_angle']
leg_y_FL     = vectors['leg_y_FL']
leg_x_FL     = vectors['leg_x_FL']
leg_y_FR     = vectors['leg_y_FR']
leg_x_FR     = vectors['leg_x_FR']
leg_y_BL     = vectors['leg_y_BL']
leg_x_BL     = vectors['leg_x_BL']
leg_y_BR     = vectors['leg_y_BR']
leg_x_BR     = vectors['leg_x_BR']

sim.startSimulation()
Center_Spot_pos = sim.getObjectPosition(Center_Spot, -1)
sim.stopSimulation()
time.sleep(5)
sim.startSimulation()
stop_time = 2
signal = sim.getInt32Signal("execDone")
for i in range(len(leg_1_x_diff)):
    res = sim.callScriptFunction('step', spot_script, leg_4_y_diff[i], leg_4_x_diff[i], 0.2, -path_angle[i], 1, leg_y_FL[i], leg_x_FL[i])
    time.sleep(stop_time)
    res = sim.callScriptFunction('step', spot_script, leg_1_y_diff[i], leg_1_x_diff[i], 0.2, -path_angle[i], 2, leg_y_FR[i], leg_x_FR[i])
    time.sleep(stop_time)
    if i < 1000 :
        res = sim.callScriptFunction('step', spot_script, leg_3_y_diff[i], leg_3_x_diff[i], 0.2, -path_angle[i], 3, leg_y_BL[i], leg_x_BL[i])
        time.sleep(stop_time)
        res = sim.callScriptFunction('step', spot_script, leg_2_y_diff[i], leg_2_x_diff[i], 0.2, -path_angle[i], 4, leg_y_BR[i], leg_x_BR[i])
        time.sleep(stop_time)
    else:
        res = sim.callScriptFunction('step', spot_script, leg_2_y_diff[i], leg_2_x_diff[i], 0.2, -path_angle[i], 3)
        time.sleep(stop_time)
        res = sim.callScriptFunction('step', spot_script, leg_3_y_diff[i], leg_3_x_diff[i], 0.2, -path_angle[i], 4)
        time.sleep(stop_time)
    print('------------------------')
    print(i)
    print('------------------------')