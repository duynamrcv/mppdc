from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle
from config import *

def getCircle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 150 )   
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

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

with open(SAVE_FILE, 'rb') as file:
    data = pickle.load(file)

## Plot
## time_stamp, x, y, z, vx, vy, vz, ax, ay, az, mode, scaling_factor, gx, gy, gz, gvx, gvy, gvz
# Plot motion path
plt.figure(figsize=(9,3.5))
# Plot solid environment
kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
for coord in coords:
    coord = np.array(coord)
    plt.plot(coord[:,0], coord[:,1], **kwargs)
plt.plot([], [], label="Environment surface", **kwargs)

for i in range(NUM_UAV):
    path = data[i]['path']
    plt.plot(path[:,1], path[:,2], label="Drone {}".format(i))
plt.legend()
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.grid(True)
plt.axis('scaled')
plt.tight_layout()

# Plot Speed
plt.figure(figsize=(6,3))
speeds = []
for i in range(NUM_UAV):
    path = data[i]['path']
    speeds.append(np.linalg.norm(path[:,4:7], axis=1))
speeds = np.array(speeds).T
plt.fill_between(path[:,0], np.min(speeds,axis=1), np.max(speeds,axis=1), color="#1f77b4", label="Max/Min", alpha=0.3)
plt.plot(path[:,0], np.mean(speeds,axis=1), 'b-', label="Average")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.xlim([0, path[-1,0]])
plt.legend()
plt.tight_layout()
plt.grid(True)

# Plot order
plt.figure(figsize=(6,3))
headings = []
for i in range(1,len(path)):
    heading = 0
    for j in range(NUM_UAV):
        heading += data[j]['path'][i,4:6]/np.linalg.norm(data[j]['path'][i,4:6])
    headings.append(np.linalg.norm(heading)/NUM_UAV)
plt.plot(path[1:,0], headings, 'b-')
plt.xlabel("Time (s)")
plt.ylabel("Order")
plt.xlim([0, path[-1,0]])
plt.ylim(0,1.1)
plt.tight_layout()
plt.grid(True)

# Plot number of correlation
plt.figure(figsize=(6,3))
size = 10
x = np.arange(0,path.shape[0],size)
num_state = 0
for i in range(NUM_UAV):
    num_state += data[i]['path'][:,10]
num_state = num_state[x]

plt.bar(path[:,0][x], num_state, label="Tailgating")
plt.bar(path[:,0][x], NUM_UAV-num_state, bottom=num_state, label="Formation")
plt.xlabel("Time (s)")
plt.ylabel("Number of Robot")
plt.xlim([-1, path[-1,0]+1])
plt.ylim([0, NUM_UAV])
plt.tight_layout()
plt.legend()

# Plot error
plt.figure(figsize=(6,3))
for i in range(NUM_UAV):
    plt.plot(data[i]['path'][1:,0], data[i]['error'])

# Plot distance
plt.figure(figsize=(6,3))
distances = []
for i in range(NUM_UAV-1):
    pathi = data[i]['path']
    for j in range(i+1,NUM_UAV):
        pathj = data[j]['path']
        distances.append(np.linalg.norm(pathi[:,1:4]-pathj[:,1:4], axis=1))
distances = np.array(distances).T
plt.plot(pathi[:,0], np.min(distances,axis=1), 'b-', label="")
plt.plot([pathi[0,0], pathi[-1,0]], [2*ROBOT_RADIUS, 2*ROBOT_RADIUS], 'k--', label="Safety radius")
plt.xlabel("Time (s)")
plt.ylabel("Inter-agent distance $\min(d_{ij})$(m)")
plt.xlim([0, pathi[-1,0]])
plt.legend()
plt.tight_layout()
plt.grid(True)

plt.show()