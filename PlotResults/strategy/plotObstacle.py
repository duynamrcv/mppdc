## 1f77b4 ff7f0e d62728 2ca02c

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

ROBOT_RADIUS = 0.3

## Load comparison files
obs = np.load("/home/duynam/MPC_formation_changing/Environment/environment_1.npy")
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

plt.figure(figsize=(5,3))
b_distances = np.array([np.min(np.hypot(b_path0[:,1]-obs[:,0,None], b_path0[:,2]-obs[:,1,None]),axis=0),
                        np.min(np.hypot(b_path1[:,1]-obs[:,0,None], b_path1[:,2]-obs[:,1,None]),axis=0),
                        np.min(np.hypot(b_path2[:,1]-obs[:,0,None], b_path2[:,2]-obs[:,1,None]),axis=0)]).T
p_distances = np.array([np.min(np.hypot(p_path0[:,1]-obs[:,0,None], p_path0[:,2]-obs[:,1,None]),axis=0),
                        np.min(np.hypot(p_path1[:,1]-obs[:,0,None], p_path1[:,2]-obs[:,1,None]),axis=0),
                        np.min(np.hypot(p_path2[:,1]-obs[:,0,None], p_path2[:,2]-obs[:,1,None]),axis=0)]).T

plt.plot(b_path0[:,0], np.min(b_distances,axis=1), 'b-', label="Rigid formation")
plt.plot(p_path0[:,0], np.min(p_distances,axis=1), 'r-', label="Proposed")
plt.plot([0, max(b_path0[-1,0],p_path0[-1,0])], [ROBOT_RADIUS, ROBOT_RADIUS], 'k--')
plt.xlabel("Time (s)")
plt.ylabel("Distance to obstacles $\min(d_{im})$ (m)")
plt.xlim([0, max(b_path0[-1,0],p_path0[-1,0])])
plt.legend()
plt.grid(True)
plt.savefig("/home/duynam/MPC_formation_changing/PlotResults/results/strategy_do.pdf", format="pdf", bbox_inches="tight")
plt.show()