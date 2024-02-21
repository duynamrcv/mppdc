from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import math
from config import *

# Solid environment
coords = [
        [[-8,-2], [0,-2], [0,1], [ 5,2.5], [15,2.5], [20,-2]],
        [[-8,8], [0,8], [0,5], [10,3.5], [15,3.5], [20,8]]
    ]

# For drone representation
p1 = np.array([3*ROBOT_RADIUS / 4, 0, 0, 1]).T
p2 = np.array([-3*ROBOT_RADIUS / 4, 0, 0, 1]).T
p3 = np.array([0, 3*ROBOT_RADIUS / 4, 0, 1]).T
p4 = np.array([0,-3*ROBOT_RADIUS / 4, 0, 1]).T

def transformation_matrix(data):
    x = data[0]; y = data[1]; z = data[2]; psi = data[3]
    return np.array([[np.cos(psi), -np.sin(psi), 0, x],
                     [np.sin(psi), np.cos(psi), 0, y],
                     [0, 0, 1, z]])

def getCircle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 150 )   
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

path0 = np.load("path_0.npy")
path1 = np.load("path_1.npy")
path2 = np.load("path_2.npy")
# ct = np.load("process_time.npy")
# print(len(path0), len(path1), len(path2))

## time_stamp, x, y, z, vx, vy, vz, ax, ay, az, mode, scaling_factor
length = len(path0)
fig = plt.figure()
plt.get_current_fig_manager().full_screen_toggle()
ax = plt.axes()
for i in range(length):
    ax.cla()

    # Plot solid environment
    kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
    for coord in coords:
        coord = np.array(coord)
        plt.plot(coord[:,0], coord[:,1], **kwargs)
    ax.plot([], [], label="Environment surface", **kwargs)

    # Plot observed obstacles
    obstacles = observerObstacles([path0[i,1:4], path1[i,1:4], path2[i,1:4]])
    if obstacles.shape[0] != 0:
        plt.scatter(obstacles[:,0], obstacles[:,1], label="Observed obstacle points")

    # Plot current trjectory
    ax.plot(path0[:i,1], path0[:i,2], "-r", label="Drone 0")
    ax.plot(path1[:i,1], path1[:i,2], "-g", label="Drone 1")
    ax.plot(path2[:i,1], path2[:i,2], "-b", label="Drone 2")

    # Plot current formation
    formation = np.array([[path0[i,1], path1[i,1], path2[i,1], path0[i,1]],
                            [path0[i,2], path1[i,2], path2[i,2], path0[i,2]]]).T
    ax.plot(formation[:,0], formation[:,1])

    # Plot current drones
    # Drone1 
    T = transformation_matrix([path0[i,1],path0[i,2],path0[i,3],
                               math.atan2(path0[i,5], path0[i,4])])
    p1_t = np.matmul(T, p1)
    p2_t = np.matmul(T, p2)
    p3_t = np.matmul(T, p3)
    p4_t = np.matmul(T, p4)

    ax.scatter( [p2_t[0], p3_t[0], p4_t[0]],
                [p2_t[1], p3_t[1], p4_t[1]], s=20, c='k')
    ax.scatter(p1_t[0], p1_t[1], s=20, c='r')

    ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]], 'k-')
    ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]], 'k-')

    a, b = getCircle(path0[i,1], path0[i,2], ROBOT_RADIUS)
    ax.plot(a, b, '--k')

    # Drone 2
    T = transformation_matrix([path1[i,1],path1[i,2],path1[i,3],
                               math.atan2(path1[i,5], path1[i,4])])
    p1_t = np.matmul(T, p1)
    p2_t = np.matmul(T, p2)
    p3_t = np.matmul(T, p3)
    p4_t = np.matmul(T, p4)

    ax.scatter( [p2_t[0], p3_t[0], p4_t[0]],
                [p2_t[1], p3_t[1], p4_t[1]], s=20, c='k')
    ax.scatter(p1_t[0], p1_t[1], s=20, c='r')

    ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]], 'k-')
    ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]], 'k-')
    a, b = getCircle(path1[i,1], path1[i,2], ROBOT_RADIUS)
    ax.plot(a, b, '--k')

    # Drone 3
    T = transformation_matrix([path2[i,1],path2[i,2],path2[i,3],
                               math.atan2(path2[i,5], path2[i,4])])
    p1_t = np.matmul(T, p1)
    p2_t = np.matmul(T, p2)
    p3_t = np.matmul(T, p3)
    p4_t = np.matmul(T, p4)

    ax.scatter( [p2_t[0], p3_t[0], p4_t[0]],
                [p2_t[1], p3_t[1], p4_t[1]], s=20, c='k')
    ax.scatter(p1_t[0], p1_t[1], s=20, c='r')

    ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]], 'k-')
    ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]], 'k-')
    a, b = getCircle(path2[i,1], path2[i,2], ROBOT_RADIUS)
    ax.plot(a, b, '--k')

    plt.gcf().canvas.mpl_connect('key_release_event',
                                    lambda event:
                                    [exit(0) if event.key == 'escape' else None])

    plt.grid(True)
    ax.axis('scaled')
    plt.legend()
    plt.xlim([-9, 25])
    fig.savefig("images/{:.6f}.png".format(time.time()))
    plt.pause(0.001)

plt.show()