import matplotlib.pyplot as plt
import numpy as np
import math

def obstacle2point(coord, step = 0.2):
    size = len(coord)
    # coord.append(coord[0])

    points = []
    for i in range(size-1):
        s = coord[i]; e = coord[i+1]
        num = round(math.hypot(e[1]-s[1], e[0]-s[0])/step)
        for j in range(num):
            u = j/num 
            points.append([s[0]*u + e[0]*(1 - u),
                           s[1]*u + e[1]*(1 - u)])
    return points

index = 1
# Obstacles
if index == 1:
    coords = [
        [[-8,-2], [0,-2], [0,1], [ 5,2.5], [15,2.5], [20,-2]],
        [[-8,8], [0,8], [0,5], [10,3.5], [15,3.5], [20,8]]
    ]


all_points = []
for i in range(len(coords)):
    coord = coords[i]
    points = obstacle2point(coord)
    all_points.extend(points)
all_points = np.array(all_points)
np.save("environment_{}.npy".format(index), all_points)

# Plot obstacles
plt.figure()
for i in range(len(coords)):
    coord = coords[i]
    coord.append(coord[0])  # repeat the first point to create a 'closed loop'
    xs, ys = zip(*coord)    # create lists of x and y values
    plt.plot(xs,ys,'-k')

plt.scatter(all_points[:,0], all_points[:,1])

plt.xlabel('x (m)') 
plt.ylabel('y (m)')
plt.axis('scaled')
plt.show()