from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle 

NUM_UAV = 5

## Load comparison files
methods = ["Behavioral", "Soria2021", "Proposed"]
colors = ["#1f77b4", "#2ca02c", "#d62728"]
names = ["Behavior", "Predictive", "Our"]

plt.figure(figsize=(6,3))

for idx in range(len(methods)):
    with open("../{}/data.txt".format(methods[idx]), 'rb') as file:
        data = pickle.load(file)
    speeds = []
    for i in range(NUM_UAV):
        path = data[i]
        speeds.append(np.linalg.norm(path[:,4:7], axis=1))
    speeds = np.array(speeds).T
    plt.fill_between(path[:,0], np.min(speeds,axis=1), np.max(speeds,axis=1), color=colors[idx], label="{} - Max/Min".format(names[idx]), alpha=0.3)
    plt.plot(path[:,0], np.mean(speeds,axis=1), color=colors[idx], label="{} - Average".format(names[idx]))

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.xlim([0, path[-1,0]])
plt.legend()
plt.tight_layout()
plt.grid(True)

plt.savefig("results/velocity.pdf", format="pdf", bbox_inches="tight")
plt.show()