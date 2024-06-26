from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle

SCENARIO = 1
TYPE = 3
ROBOT_RADIUS = 0.2
SAVE_FILE = "../Proposed/data_{}_{}.txt".format(SCENARIO, TYPE)

def getCircle(x,y,r):
    theta = np.linspace( 0 , 2 * np.pi , 150 )   
    a = x + r * np.cos( theta )
    b = y + r * np.sin( theta )
    return a, b

if SCENARIO == 1:
    coords = [
            [[-8,-2], [0,-2], [0,1], [ 5,2.5], [15,2.5], [20,-2]],
            [[-8,8], [0,8], [0,5], [10,3.5], [15,3.5], [20,8]]
        ]
elif SCENARIO == 2:
    coords = [
        [[-8,-2], [0,-2], [0,1], [5,1.5], [5,-2], [13,-2], [13,2.5], [18,2.5], [18,-2], [20,-2]],
        [[-8, 8], [0, 8], [0,5], [5,5.0], [5, 8], [13, 8], [13,3.5], [18,3.5], [18, 8], [20, 8]]
    ]
elif SCENARIO == 3:
    coords = [
        [[-8,-2], [0,-2], [0,1], [18,1], [18,-2], [20,-2]],
        [[-8, 8], [0, 8], [0,6], [ 5,4], [10, 4], [15, 2], [18, 2], [18, 8], [20, 8]]
    ]

with open(SAVE_FILE, 'rb') as file:
    data = pickle.load(file)
NUM_UAV = len(data)

plt.figure(figsize=(6,2.8))
# Plot solid environment
kwargs = {'color': 'k', 'linewidth': 2, 'linestyle': '-'}
for coord in coords:
    coord = np.array(coord)
    plt.plot(coord[:,0], coord[:,1], **kwargs)
plt.plot([], [], label="Environment surface", **kwargs)

# Plot path
for i in range(NUM_UAV):
    path = data[i]['path']
    plt.plot(path[:,1], path[:,2], label="Drone {}".format(i))

# Plot
n_plot = 4
length = path.shape[0]
for i in range(n_plot+1):
    iter = int(round(length/n_plot)*i)
    if iter >= length:
        iter = length-1

    # Plot robots
    for i in range(NUM_UAV):
        a, b = getCircle(data[i]['path'][iter,1], data[i]['path'][iter,2], ROBOT_RADIUS)
        plt.plot(a, b, '-b')

    positions = []
    for i in range(NUM_UAV):
        positions.append(data[i]['path'][iter,1:4])

    # Plot current formation
    positions.append(positions[0])
    formation = np.array(positions)
    plt.plot(formation[:,0], formation[:,1], '-k')

plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.grid(True)
plt.axis('scaled')
plt.legend(loc='upper center', bbox_to_anchor=(0.5,1.4), ncol=3, fancybox=True, shadow=True)
plt.tight_layout()
plt.savefig("../PlotResults/results/path_scen{}.pdf".format(SCENARIO+TYPE), format="pdf", bbox_inches="tight")

# Plot number of correlation
plt.figure(figsize=(6,2))
size = 10
x = np.arange(0,path.shape[0],size)
num_state = 0
for i in range(NUM_UAV):
    num_state += data[i]['path'][:,10]
num_state = num_state[x]

plt.bar(path[:,0][x], num_state, label="Tailgating")
plt.bar(path[:,0][x], NUM_UAV-num_state, bottom=num_state, label="Formation")
plt.xlabel("Time (s)")
plt.ylabel("Number of Robots")
plt.xlim([-1, path[-1,0]+1])
plt.ylim([0, NUM_UAV])
plt.tight_layout()
plt.legend()
plt.savefig("../PlotResults/results/correlation_scen{}.pdf".format(SCENARIO+TYPE), format="pdf", bbox_inches="tight")

plt.show()