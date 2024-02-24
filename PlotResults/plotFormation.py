## 1f77b4 ff7f0e d62728 2ca02c

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import math

ROBOT_RADIUS = 0.3

# Solid environment
coords = [
        [[-8,-2], [0,-2], [0,1], [ 5,2.5], [15,2.5], [20,-2]],
        [[-8,8], [0,8], [0,5], [10,3.5], [15,3.5], [20,8]]
    ]
num = 7

## Load comparison files
# Behavior
b_path = "/home/duynam/MPC_formation_changing/Behavioral/"
b_path0 = np.load(b_path+"path_0.npy")
b_path1 = np.load(b_path+"path_1.npy")
b_path2 = np.load(b_path+"path_2.npy")

# Proposed
p_path = "/home/duynam/MPC_formation_changing/Proposed/"
p_path0 = np.load(p_path+"path_0.npy")
p_path1 = np.load(p_path+"path_1.npy")
p_path2 = np.load(p_path+"path_2.npy")

# Soria2021
s_path = "/home/duynam/MPC_formation_changing/Soria2021/"
s_path0 = np.load(s_path+"path_0.npy")
s_path1 = np.load(s_path+"path_1.npy")
s_path2 = np.load(s_path+"path_2.npy")

## Proposed strategy
plt.figure(figsize=(10,4))
# Plot solid environment
kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
for coord in coords:
    coord = np.array(coord)
    plt.plot(coord[:,0], coord[:,1], **kwargs)
plt.plot([], [], label="Environment surface", **kwargs)

# Plot paths
plt.plot(p_path0[:,1], p_path0[:,2], label="Drone 0")
plt.plot(p_path1[:,1], p_path1[:,2], label="Drone 1")
plt.plot(p_path2[:,1], p_path2[:,2], label="Drone 2")

# Plot formation
step = math.ceil(p_path0.shape[0]/num)
for i in range(num+1):
    idx = i*step
    if idx >= p_path0.shape[0]:
        idx = -1
    formation = np.array([[p_path0[idx,1],p_path1[idx,1],p_path2[idx,1],p_path0[idx,1]],
                          [p_path0[idx,2],p_path1[idx,2],p_path2[idx,2],p_path0[idx,2]]]).T
    plt.plot(formation[:,0], formation[:,1], "cp-")


plt.legend()
plt.grid(True)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.axis('scaled')
plt.legend()
plt.xlim([-9, 25])
plt.tight_layout()
plt.savefig("/home/duynam/MPC_formation_changing/PlotResults/results/proposed_path.pdf", format="pdf", bbox_inches="tight")

## Behavior strategy
plt.figure(figsize=(10,4))
# Plot solid environment
kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
for coord in coords:
    coord = np.array(coord)
    plt.plot(coord[:,0], coord[:,1], **kwargs)
plt.plot([], [], label="Environment surface", **kwargs)

# Plot paths
plt.plot(b_path0[:,1], b_path0[:,2], label="Drone 0")
plt.plot(b_path1[:,1], b_path1[:,2], label="Drone 1")
plt.plot(b_path2[:,1], b_path2[:,2], label="Drone 2")

# Plot formation
step = math.ceil(b_path0.shape[0]/num)
for i in range(num+1):
    idx = i*step
    if idx >= b_path0.shape[0]:
        idx = -1
    formation = np.array([[b_path0[idx,1],b_path1[idx,1],b_path2[idx,1],b_path0[idx,1]],
                          [b_path0[idx,2],b_path1[idx,2],b_path2[idx,2],b_path0[idx,2]]]).T
    plt.plot(formation[:,0], formation[:,1], "cp-")


plt.legend()
plt.grid(True)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.axis('scaled')
plt.legend()
plt.xlim([-9, 25])
plt.tight_layout()
plt.savefig("/home/duynam/MPC_formation_changing/PlotResults/results/behavior_path.pdf", format="pdf", bbox_inches="tight")

## Soria2021 strategy
plt.figure(figsize=(10,4))
# Plot solid environment
kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
for coord in coords:
    coord = np.array(coord)
    plt.plot(coord[:,0], coord[:,1], **kwargs)
plt.plot([], [], label="Environment surface", **kwargs)

# Plot paths
plt.plot(s_path0[:,1], s_path0[:,2], label="Drone 0")
plt.plot(s_path1[:,1], s_path1[:,2], label="Drone 1")
plt.plot(s_path2[:,1], s_path2[:,2], label="Drone 2")

# Plot formation
step = math.ceil(s_path0.shape[0]/num)
for i in range(num+1):
    idx = i*step
    if idx >= s_path0.shape[0]:
        idx = -1
    formation = np.array([[s_path0[idx,1],s_path1[idx,1],s_path2[idx,1],s_path0[idx,1]],
                          [s_path0[idx,2],s_path1[idx,2],s_path2[idx,2],s_path0[idx,2]]]).T
    plt.plot(formation[:,0], formation[:,1], "cp-")


plt.legend()
plt.grid(True)
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.axis('scaled')
plt.legend()
plt.xlim([-9, 25])
plt.tight_layout()
plt.savefig("/home/duynam/MPC_formation_changing/PlotResults/results/soria_path.pdf", format="pdf", bbox_inches="tight")

plt.show()
