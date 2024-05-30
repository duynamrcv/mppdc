from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import math
import pickle
from config import *

percent = 0.3 
width = 0.02
export = False
if export:
    import cv2
    image_array = []

# Solid environment
if SCENARIO == 1:
    coords = [
            [[-8,-2], [0,-2], [0,1], [ 5,2.5], [15,2.5], [20,-2]],
            [[-8,8], [0,8], [0,5], [10,3.5], [15,3.5], [20,8]]
        ]
else:
    coords = [
        [[-8,-2], [0,-2], [0,1], [5,1.5], [5,-2], [13,-2], [13,2.5], [18,2.5], [18,-2], [20,-2]],
        [[-8, 8], [0, 8], [0,5], [5,5.0], [5, 8], [13, 8], [13,3.5], [18,3.5], [18, 8], [20, 8]]
    ]

def getCircle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 50 )   
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

def observerObstacles(positions):
    observed_obstacles = []
    for position in positions:
        for j in range(OBSTACLES.shape[0]):
            obs = OBSTACLES[j,:]
            if np.linalg.norm(position[:2]-obs[:2]) <= SENSING_RADIUS:
                observed_obstacles.append(obs)
    return np.array(observed_obstacles)

def findCentroid(positions):
    centroid = 0
    for position in positions:
        centroid += position[:2]
    return centroid/NUM_UAV

with open(SAVE_FILE, 'rb') as file:
    data = pickle.load(file)

size = (6,6)
plt.figure(figsize=size)
ax = plt.axes()
for iter in range(data[0]['path'].shape[0]):
    ax.cla()

    # Plot solid environment
    kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
    for coord in coords:
        coord = np.array(coord)
        plt.plot(coord[:,0], coord[:,1], **kwargs)
    ax.plot([], [], label="Environment surface", **kwargs)

    # Plot observed obstacles
    positions = []
    for i in range(NUM_UAV):
        positions.append(data[i]['path'][iter,1:4])
    
    centroid = findCentroid(positions)
    
    obstacles = observerObstacles(positions)
    if obstacles.shape[0] != 0:
        plt.scatter(obstacles[:,0], obstacles[:,1], label="Observed obstacle points")

    # Plot current trjectory
    for i in range(NUM_UAV):
        path = data[i]['path']
        ax.plot(path[:iter,1], path[:iter,2], "-", label="Drone {}".format(i))

        # Plot current drone
        a, b = getCircle(path[iter,1], path[iter,2], ROBOT_RADIUS)
        plt.plot(a, b, '-b')
        plt.arrow(path[iter,1],  path[iter,2],
                    path[iter,4]*percent,  path[iter,5]*percent,
                    width=width, color='r')

    # Plot current formation
    positions.append(positions[0])
    formation = np.array(positions)
    ax.plot(formation[:,0], formation[:,1], '-k')

    

    plt.gcf().canvas.mpl_connect('key_release_event',
                                    lambda event:
                                    [exit(0) if event.key == 'escape' else None])

    
    plt.grid(True)
    ax.axis('scaled')
    plt.legend()
    plt.xlim([centroid[0]-5, centroid[0]+5])
    plt.ylim([centroid[1]-5, centroid[1]+5])
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.tight_layout()
    plt.title("Time: {:.3f}s".format(path[iter,0]))
    plt.pause(0.001)

    if export:
        file_name = "results/data.png"
        plt.savefig(file_name)
        img = cv2.imread(file_name)
        image_array.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

if export:
    import imageio
    imageio.mimsave('results/results.gif', image_array)

plt.show()