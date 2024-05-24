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
    headings = []
    for i in range(1,data[0].shape[0]):
        heading = 0
        for j in range(NUM_UAV):
            heading += data[j][i,4:6]/np.linalg.norm(data[j][i,4:6])
        headings.append(np.linalg.norm(heading)/NUM_UAV)
    plt.plot(data[0][1:,0], headings, color=colors[idx], label=names[idx])

plt.xlabel("Time (s)")
plt.ylabel("Order $\Phi$")
plt.xlim([0, data[0][-1,0]])
plt.legend()
plt.tight_layout()
plt.grid(True)

plt.savefig("results/order.pdf", format="pdf", bbox_inches="tight")
plt.show()