from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle 

NUM_UAV = 5
ROBOT_RADIUS = 0.2
SCENARIO = 1

## Load comparison files
methods = ["Behavioral", "Soria2021", "Proposed"]
colors = ["#1f77b4", "#2ca02c", "#d62728"]
names = ["BDC", "MPFC", "MPPDC"]
styles = ["-", "--", "-."]

plt.figure(figsize=(5.5,2.5))

bias = 0.28
for idx in range(len(methods)):
    print(methods[idx])
    with open("../{}/data_{}.txt".format(methods[idx], SCENARIO), 'rb') as file:
        data = pickle.load(file)
    errors = []
    for i in range(NUM_UAV):
        if methods[idx] == "Soria2021" and SCENARIO == 2:
            error = data[i]['error'][1:210]
            time = data[i]['path'][1:210,0]
        else:
            error = data[i]['error']
            time = data[i]['path'][1:,0]

        if methods[idx] == "Proposed":
            error -= bias
            error[error < 0.0] = 0.0
        errors.append(error)
    errors = np.array(errors)
    plt.fill_between(time, np.min(errors,axis=0), np.max(errors,axis=0), color=colors[idx], label="{} - Max/Min".format(names[idx]), alpha=0.3)
    plt.plot(time, np.mean(errors,axis=0), styles[idx], color=colors[idx], label="{} - Average".format(names[idx]))

plt.xlabel("Time (s)")
plt.ylabel("Formation error $\epsilon$")
plt.xlim([0, data[i]['path'][-1,0]])
plt.legend(ncol=2, fancybox=True)
plt.tight_layout()
plt.grid(True)

plt.savefig("results/error_scen{}.pdf".format(SCENARIO), format="pdf", bbox_inches="tight")
plt.show()