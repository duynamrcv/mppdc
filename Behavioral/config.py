import numpy as np
import time
import matplotlib.pyplot as plt

TIMESTEP = 0.1
ROBOT_RADIUS = 0.3
SENSING_RADIUS = 3.0
EPSILON = 0.1
TOPOLOGY = np.array([[-np.sqrt(3)/2,-0.5, 0.0],
                     [ np.sqrt(3)/2,-0.5, 0.0],
                     [          0.0, 1.0, 0.0]])
NUM_UAV = TOPOLOGY.shape[0]

VREF = 1.0
UREF = np.array([1,0,0])
DREF = 3*ROBOT_RADIUS
VMAX = 1.0
UMAX = 1.0
BETA = 1.0

W_sep = 1.0
W_dir = 1.0
W_nav = 20.0
W_u = 4e-1
W_obs = 2.0
W_col = 1.0

# Scenario
STARTS = np.array([[-8., 3., 5., 0, 0, 0],
                   [-8., 2., 5., 0, 0, 0],
                   [-8., 4., 5., 0, 0, 0]])
X_GOAL = 22.
env_file = "../Environment/environment_1.npy"
OBSTACLES = np.load(env_file)