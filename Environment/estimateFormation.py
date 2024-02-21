import time
import math
import numpy as np
import matplotlib.pyplot as plt

## Parameters
UREF = np.array([1,0]) 
RS = 3.0
RA = 0.3

## Formation topology
number = 3
if number == 1:
    topology = np.array([[0.5, 0],
                        [0, 0.5],
                        [0, -0.5],
                        [-0.5, -1],
                        [-0.5, 1]])
elif number == 2:
    topology = np.array([[(3**0.5)/2, 0.5],
                        [0,1],
                        [-(3**0.5)/2, 0.5],
                        [-(3**0.5)/2, -0.5],
                        [0,-1],
                        [(3**0.5)/2, -0.5]])
elif number == 3:
    topology = np.array([[ 1, .5],
                        [ 1,-.5],
                        [ 0,  1],
                        [ 0, -1],
                        [-1,  1],
                        [-1, -1]])
    
def estimateFormationWidth(topology):
    y_left = np.min(topology[:,1])
    y_right = np.max(topology[:,1])
    return y_right - y_left

def selectLeader(index, drones):
    drone = drones[index,:]
    vec = (drones[:,0]-drone[0])*UREF[0] + (drones[:,1]-drone[1])*UREF[1]
    vec[np.where(vec<=0)]=np.inf
    print(vec)
    leader_index = np.argmin(vec)
    if leader_index == index:   # is leader
        return -1
    return leader_index

rotation = np.pi/6
topology = topology@np.array([[np.cos(rotation),-np.sin(rotation)],
                              [np.sin(rotation), np.cos(rotation)]])
index = 5
lidx = selectLeader(index, topology)
# print(estimateFormationWidth(topology))
plt.figure()
plt.scatter(topology[:,0], topology[:,1], label="Formation")
plt.scatter(topology[index,0], topology[index,1], label="Drone")
if lidx != -1:
    plt.scatter(topology[lidx,0], topology[lidx,1], label="Drone Leader")
plt.legend()
plt.grid(True)
plt.axis('scaled')
plt.show()