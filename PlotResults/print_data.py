from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle 

NUM_UAV = 5
SCENARIO = 2
SENSING_RADIUS = 3.0
ROBOT_RADIUS = 0.2
OBSTACLES = np.load("../Environment/environment_{}.npy".format(SCENARIO))

## Load comparison files
methods = ["Behavioral", "Soria2021", "Proposed"]
def observerObstacles(state):
    observed_obstacles = []
    for j in range(OBSTACLES.shape[0]):
        obs = OBSTACLES[j,:]
        if np.linalg.norm(state[:2]-obs[:2]) <= SENSING_RADIUS:
            observed_obstacles.append(obs)
    return np.array(observed_obstacles)

# Data to print: Travel time, min distance to obstacle, min distance to others, success rate

# Min distance to neighbors
for idx in range(len(methods)):
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    min_dis2other = float(np.inf)
    for i in range(NUM_UAV-1):
        pathi = data[i]['path']
        for j in range(i+1,NUM_UAV):
            pathj = data[j]['path']
            x = np.min(np.linalg.norm(pathi[:,1:4]-pathj[:,1:4], axis=1))
            if min_dis2other > x:
                min_dis2other = x
    print("Min distance to neighbors - {}: {:.4f} m".format(methods[idx], min_dis2other))
print("-----")

# Min distance to obstacles
for idx in range(len(methods)):
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    min_dis2obs = float(np.inf)
    for i in range(NUM_UAV):
        path = data[i]['path']
        for j in range(path.shape[0]):
            obstacles = observerObstacles(path[j,1:4])
            if obstacles.shape[0] == 0:
                continue
            dis = np.min(np.linalg.norm(path[j,1:3] - obstacles, axis=1))
            min_dis2obs = min(dis, min_dis2obs)

    print("Min distance to obstacles - {}: {:.4f} m".format(methods[idx], min_dis2obs-ROBOT_RADIUS))
print("-----")

# Travel time
for idx in range(len(methods)):
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    print("Traveling time - {}: {:.4f} s".format(methods[idx], data[0]['path'].shape[0]))
print("-----")


# Travel distance
for idx in range(len(methods)):
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    distance = 0
    for i in range(NUM_UAV):
        path = data[i]['path']
        for j in range(path.shape[0]-1):
            distance += np.linalg.norm(path[j+1,1:3]-path[j,1:3])
    distance /= NUM_UAV
    print("Traveling distance - {}: {:.4f} m".format(methods[idx], distance))
print("-----")
