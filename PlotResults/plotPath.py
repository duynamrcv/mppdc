from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import pickle

SCENARIO = 2
NUM_UAV = 5
ROBOT_RADIUS = 0.2

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
else:
    coords = [
        [[-8,-2], [0,-2], [0,1], [5,1.5], [5,-2], [13,-2], [13,2.5], [18,2.5], [18,-2], [20,-2]],
        [[-8, 8], [0, 8], [0,5], [5,5.0], [5, 8], [13, 8], [13,3.5], [18,3.5], [18, 8], [20, 8]]
    ]

SAVE_FILE =  "../Proposed/data_{}.txt".format(SCENARIO)
with open(SAVE_FILE, 'rb') as file:
    data = pickle.load(file)

## Plot
## time_stamp, x, y, z, vx, vy, vz, ax, ay, az, mode, scaling_factor, gx, gy, gz, gvx, gvy, gvz
# Plot motion path
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
plt.savefig("results/path_scen{}.pdf".format(SCENARIO), format="pdf", bbox_inches="tight")
plt.show()