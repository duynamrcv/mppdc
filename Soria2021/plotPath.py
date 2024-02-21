from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from config import *

animation = True

def getCircle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 150 )   
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

path0 = np.load("path_0.npy")
path1 = np.load("path_1.npy")
path2 = np.load("path_2.npy")
ct = np.load("process_time.npy")
print(len(path0), len(path1), len(path2))

## Plot
## time_stamp, x, y, z, vx, vy, vz, ax, ay, az, mode, scaling_factor, gx, gy, gz, gvx, gvy, gvz
# Plot motion path
plt.figure()
plt.scatter(OBSTACLES[:,0], OBSTACLES[:,1], label="Environment")

plt.plot(path0[:,1], path0[:,2], label="Drone 0")
plt.plot(path1[:,1], path1[:,2], label="Drone 1")
plt.plot(path2[:,1], path2[:,2], label="Drone 2")
plt.legend()
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.grid(True)
plt.axis('scaled')

# Plot Speed
plt.figure()
speeds = np.array([np.linalg.norm(path0[:,4:7], axis=1),
                   np.linalg.norm(path1[:,4:7], axis=1),
                   np.linalg.norm(path2[:,4:7], axis=1)]).T
plt.fill_between(path0[:,0], np.min(speeds,axis=1), np.max(speeds,axis=1), color="#1f77b4", label="Max/Min", alpha=0.3)
plt.plot(path0[:,0], np.mean(speeds,axis=1), 'b-', label="Average")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.xlim([0, path0[-1,0]])
plt.legend()
plt.grid(True)

# Plot distance
plt.figure()
distances = np.array([np.linalg.norm(path0[:,1:4]-path1[:,1:4], axis=1),
                      np.linalg.norm(path1[:,1:4]-path2[:,1:4], axis=1),
                      np.linalg.norm(path2[:,1:4]-path0[:,1:4], axis=1)]).T
plt.fill_between(path0[:,0], np.min(distances,axis=1), np.max(distances,axis=1), color="#1f77b4", label="Max/Min", alpha=0.3)
plt.plot(path0[:,0], np.mean(distances,axis=1), 'b-', label="Average")
plt.plot([path0[0,0], path0[-1,0]], [2*ROBOT_RADIUS, 2*ROBOT_RADIUS], 'k--', label="Safety radius")
plt.xlabel("Time (s)")
plt.ylabel("Inter-agent distance (m)")
plt.xlim([0, path0[-1,0]])
plt.legend()
plt.grid(True)

# Plot order
plt.figure()
headings = []
for i in range(1,len(path0)):
    heading = path0[i,4:6]/np.linalg.norm(path0[i,4:6]) \
            + path1[i,4:6]/np.linalg.norm(path1[i,4:6]) \
            + path2[i,4:6]/np.linalg.norm(path2[i,4:6])
    headings.append(np.linalg.norm(heading)/NUM_UAV)
plt.plot(path0[1:,0], headings)
plt.xlabel("Time (s)")
plt.ylabel("Order")
plt.xlim([0, path0[-1,0]])
plt.grid(True)

# Plot errors
plt.figure()
plt.plot(path0[:,0], np.linalg.norm(path0[:,1:4]-path0[:,12:15],axis=1), label="Drone 0")
plt.plot(path0[:,0], np.linalg.norm(path1[:,1:4]-path1[:,12:15],axis=1), label="Drone 1")
plt.plot(path0[:,0], np.linalg.norm(path2[:,1:4]-path2[:,12:15],axis=1), label="Drone 2")
err = np.array([np.linalg.norm(path0[:,1:4]-path0[:,12:15], axis=1),
                np.linalg.norm(path1[:,1:4]-path1[:,12:15], axis=1),
                np.linalg.norm(path2[:,1:4]-path2[:,12:15], axis=1)]).T
plt.fill_between(path0[:,0], np.min(err,axis=1), np.max(err,axis=1), color="#1f77b4", label="Max/Min", alpha=0.3)
plt.plot(path0[:,0], np.mean(err,axis=1), 'b-', label="Average")
plt.xlabel("Time (s)")
plt.ylabel("Formation error (m)")
plt.xlim([0, path0[-1,0]])
plt.legend()
plt.grid(True)

# Plot number of correlation
plt.figure()
size = 9
x = np.arange(0,path0.shape[0],size)
num_state = path0[:,10] + path1[:,10] + path2[:,10]
num_state = num_state[x]

plt.bar(path0[:,0][x], num_state, label="Tailgating")
plt.bar(path0[:,0][x], NUM_UAV-num_state, bottom=num_state, label="Formation")
plt.xlabel("Time (s)")
plt.ylabel("Number of Robot")
# plt.xlim([0, path0[-1,0]])
plt.legend()

# Plot scaling factor
plt.figure()
plt.plot(path0[:,0], path0[:,11], '.', label="Drone 0")
plt.plot(path0[:,0], path1[:,11], '.', label="Drone 1")
plt.plot(path0[:,0], path2[:,11], '.', label="Drone 2")
plt.xlabel("Time (s)")
plt.ylabel("Scaling factor")

plt.show()

## Animation
if animation == True:
    length = len(path0)
    ax = plt.axes()
    for i in range(length):
        ax.cla()

        # Plot obstacles
        plt.scatter(OBSTACLES[:,0], OBSTACLES[:,1])

        # Plot current trjectory
        ax.plot(path0[:i,1], path0[:i,2], "-r", label="Drone 0")
        ax.plot(path1[:i,1], path1[:i,2], "-g", label="Drone 1")
        ax.plot(path2[:i,1], path2[:i,2], "-b", label="Drone 2")

        # Plot current formation
        formation = np.array([[path0[i-1,1], path1[i-1,1], path2[i-1,1], path0[i-1,1]],
                                [path0[i-1,2], path1[i-1,2], path2[i-1,2], path0[i-1,2]]]).T
        ax.plot(formation[:,0], formation[:,1])

        # Plot current drones
        a, b = getCircle(path0[i-1,1], path0[i-1,2], ROBOT_RADIUS)
        ax.plot(a, b, '-k')
        a, b = getCircle(path1[i-1,1], path1[i-1,2], ROBOT_RADIUS)
        ax.plot(a, b, '-k')
        a, b = getCircle(path2[i-1,1], path2[i-1,2], ROBOT_RADIUS)
        ax.plot(a, b, '-k')

        
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])

        plt.grid(True)
        ax.axis('equal')
        plt.legend()
        plt.xlim([-9, 25])
        plt.pause(0.001)

    plt.show()