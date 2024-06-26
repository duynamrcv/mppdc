import numpy as np
import time
import matplotlib.pyplot as plt

TIMESTEP = 0.1
ROBOT_RADIUS = 0.2
SENSING_RADIUS = 3.0
EPSILON = 0.1
NMPC_SCALE = 3
ALPHA = 12 # alpha >= 4
BETA = 50

SCENARIO = 1
TYPE = 1
if TYPE == 1:
    TOPOLOGY = np.array([[np.sin(2*np.pi/5*0), np.cos(2*np.pi/5*0), 0.0],
                         [np.sin(2*np.pi/5*1), np.cos(2*np.pi/5*1), 0.0],
                         [np.sin(2*np.pi/5*2), np.cos(2*np.pi/5*2), 0.0],
                         [np.sin(2*np.pi/5*3), np.cos(2*np.pi/5*3), 0.0],
                         [np.sin(2*np.pi/5*4), np.cos(2*np.pi/5*4), 0.0]])*1.2
    STARTS = np.array([
        [-8., 5., 5., 0, 0, 0],
        [-8., 3., 5., 0, 0, 0],
        [-8., 1., 5., 0, 0, 0],
        [-8., 2., 5., 0, 0, 0],
        [-8., 4., 5., 0, 0, 0],
    ])
elif TYPE == 2:
    TOPOLOGY = np.array([[ 2.0,-1.0, 0.0],
                         [ 1.0, 1.0, 0.0],
                         [-1.0, 1.0, 0.0],
                         [-2.0,-1.0, 0.0],
                         [ 0.0, 0.0, 0.0]])
    STARTS = np.array([
        [-8., 1., 5., 0, 0, 0],
        [-8., 2., 5., 0, 0, 0],
        [-8., 3., 5., 0, 0, 0],
        [-9., 2., 5., 0, 0, 0],
        [-9., 1., 5., 0, 0, 0],
    ])

NUM_UAV = TOPOLOGY.shape[0]

VREF = 1.0
UREF = np.array([1,0,0])
DREF = 3*ROBOT_RADIUS

W_sep = 2.0
W_dir = 1.0
W_nav = 5.0
W_u = 4e-1
W_obs = 5.0
W_col = 2.0

# Scenario
X_GOAL = 22.
OBSTACLES = np.load("../Environment/environment_{}.npy".format(SCENARIO))
SAVE_FILE = "data_{}_{}.txt".format(SCENARIO, TYPE)