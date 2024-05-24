from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle 

NUM_UAV = 5
ROBOT_RADIUS = 0.2

## Load comparison files
methods = ["Behavioral", "Soria2021", "Proposed"]
colors = ["#1f77b4", "#2ca02c", "#d62728"]
names = ["Behavior", "Predictive", "Our"]

plt.figure(figsize=(6,3))

for idx in range(len(methods)):
    with open("../{}/data.txt".format(methods[idx]), 'rb') as file:
        data = pickle.load(file)
    distances = []
    for i in range(NUM_UAV-1):
        pathi = data[i]
        for j in range(i+1,NUM_UAV):
            pathj = data[j]
            distances.append(np.linalg.norm(pathi[:,1:4]-pathj[:,1:4], axis=1))
    distances = np.array(distances).T
    plt.plot(pathi[:,0], np.min(distances,axis=1), color=colors[idx], label=names[idx])
plt.plot([pathi[0,0], pathi[-1,0]], [2*ROBOT_RADIUS, 2*ROBOT_RADIUS], 'k--', label="Safety radius")

plt.xlabel("Time (s)")
plt.ylabel("Inter-agent distance $\min(d_{ij})$(m)")
plt.xlim([0, pathi[-1,0]])
plt.legend()
plt.tight_layout()
plt.grid(True)

plt.savefig("results/distance.pdf", format="pdf", bbox_inches="tight")
plt.show()