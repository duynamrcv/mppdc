## 1f77b4 ff7f0e d62728 2ca02c

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

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

plt.figure(figsize=(5,3))
b_speeds = np.array([np.linalg.norm(b_path0[:,4:7], axis=1),
                     np.linalg.norm(b_path1[:,4:7], axis=1),
                     np.linalg.norm(b_path2[:,4:7], axis=1)]).T
p_speeds = np.array([np.linalg.norm(p_path0[:,4:7], axis=1),
                     np.linalg.norm(p_path1[:,4:7], axis=1),
                     np.linalg.norm(p_path2[:,4:7], axis=1)]).T

plt.fill_between(b_path0[:,0], np.min(b_speeds,axis=1), np.max(b_speeds,axis=1), color="#1f77b4", label="Behavior - Max/Min", alpha=0.3)
plt.fill_between(p_path0[:,0], np.min(p_speeds,axis=1), np.max(p_speeds,axis=1), color="#d62728", label="Proposed - Max/Min", alpha=0.3)
plt.plot(b_path0[:,0], np.mean(b_speeds,axis=1), 'b-', label="Behavior - Average")
plt.plot(p_path0[:,0], np.mean(p_speeds,axis=1), 'r-', label="Proposed - Average")

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.xlim([0, max(b_path0[-1,0],p_path0[-1,0])])
plt.ylim([0, 1.4])
plt.legend()
plt.tight_layout()
plt.grid(True)
plt.savefig("/home/duynam/MPC_formation_changing/PlotResults/results/behavior_velocity.pdf", format="pdf", bbox_inches="tight")
plt.show()