from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle 

NUM_UAV = 5
SCENARIO = 2

## Load comparison files
methods = ["Behavioral", "Soria2021", "Proposed"]
colors = ["#1f77b4", "#2ca02c", "#d62728"]
names = ["BDC", "MPFC", "MPPDC"]
styles = ["-", "--", "-."]

plt.figure(figsize=(5.5,2.5))

for idx in range(len(methods)):
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    speeds = []
    for i in range(NUM_UAV):
        path = data[i]['path']
        if methods[idx] == "Soria2021" and SCENARIO == 2:
            path = data[i]['path'][:210,:]
        speeds.append(np.linalg.norm(path[:,4:7], axis=1))
    speeds = np.array(speeds).T
    plt.fill_between(path[:,0], np.min(speeds,axis=1), np.max(speeds,axis=1), color=colors[idx], label="{} - Max/Min".format(names[idx]), alpha=0.3)
    plt.plot(path[:,0], np.mean(speeds,axis=1), styles[idx], color=colors[idx], label="{} - Average".format(names[idx]))

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.xlim([0, path[-1,0]])
plt.legend(ncol=2, fancybox=True)
plt.tight_layout()
plt.grid(True)

plt.savefig("results/velocity_scen{}.pdf".format(SCENARIO), format="pdf", bbox_inches="tight")
plt.show()