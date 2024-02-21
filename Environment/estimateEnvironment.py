import time
import math
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN

## Parameters
UREF = np.array([1,0]) 
RS = 3.0
RA = 0.3

## Functions
def getObstacles(robot_pose, env):
    distance = np.linalg.norm(env - robot_pose[:2], axis=1)
    obstacles = env[np.where(distance<=RS),:][0]
    return obstacles

def cluster(obstacles):
    clusters = DBSCAN(eps = 2*RA, min_samples = 3).fit(obstacles)
    label = np.unique(clusters.labels_)
    x = []
    for i in label:
        xi = obstacles[np.where(clusters.labels_==i),:][0]
        x.append(xi)
    return x

def estimateEnvironmentWidth(robot_pose, obstacles):
    # Get obstacles in front of robot in motion direction
    vec = robot_pose[:2] - obstacles
    obstacles = obstacles[np.where(vec*UREF<0),:][0]

    # DBSCAN clustering
    clusters = DBSCAN(eps = 2*RA, min_samples = 3).fit(obstacles)
    label = np.unique(clusters.labels_)
    if label.shape[0] != 2:
        return None
    
    x = []
    for i in label:
        xi = obstacles[np.where(clusters.labels_==i),:][0]
        # Find the environment's width
        distance = abs((xi[:,0]-robot_pose[0])*UREF[1] +\
                       (xi[:,1]-robot_pose[1])*UREF[0])
        index = np.argmin(distance)
        x.append(xi[index,:])
    
    theta = math.atan2(UREF[1], UREF[0])
    width = abs((x[0][0]-x[1][0])*np.sin(theta) + (x[0][1]-x[1][1])*np.cos(theta))
    return x, width

## Main process
env_file = "environment_1.npy"
env = np.load(env_file)
robot = np.array([4.5,3.0,np.pi/6])

start = time.time()
obstacles = getObstacles(robot, env)
obstacles,width = estimateEnvironmentWidth(robot, obstacles)
print("time: {}s".format(time.time()-start))
print(obstacles)
print("width = {}".format(width))

plt.figure()
plt.scatter(env[:,0], env[:,1], label="Environment")
plt.scatter(robot[0], robot[1], label="Robot")
if obstacles is not None:
    for obstacle in obstacles:
        plt.scatter(obstacle[0], obstacle[1], label="Observed obstacles")
plt.legend()
plt.grid(True)
plt.axis('scaled')
plt.show()