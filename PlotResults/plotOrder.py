from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle 

NUM_UAV = 5
ROBOT_RADIUS = 0.2
SCENARIO = 2

## Load comparison files
methods = ["Behavioral", "Soria2021", "Proposed"]
colors = ["#1f77b4", "#2ca02c", "#d62728"]
names = ["BC-Changing", "MPC-Rigid", "Our"]
styles = ["-", "--", "-."]

plt.figure(figsize=(5.5,2.5))

for idx in range(len(methods)):
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    headings = []
    if methods[idx] == "Soria2021" and SCENARIO == 2:
        length = 210
    else:
        length = data[0]['path'].shape[0]
    for i in range(1,length):
        heading = 0
        for j in range(NUM_UAV):
            if methods[idx] == "Soria2021" and SCENARIO == 2:
                path = data[j]['path'][:210,:]
            else:
                path = data[j]['path']
            heading += path[i,4:6]/np.linalg.norm(path[i,4:6])
        headings.append(np.linalg.norm(heading)/NUM_UAV)
    plt.plot(path[1:,0], headings, styles[idx], color=colors[idx], label=names[idx])

plt.xlabel("Time (s)")
plt.ylabel("Order $\Phi$")
plt.xlim([0, data[0]['path'][-1,0]])
plt.ylim([0.6, 1.1])
plt.legend()
plt.tight_layout()
plt.grid(True)

plt.savefig("results/order_scen{}.pdf".format(SCENARIO), format="pdf", bbox_inches="tight")
plt.show()