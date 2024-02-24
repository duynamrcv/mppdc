## 1f77b4 ff7f0e d62728 2ca02c

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes,mark_inset

NUM_UAV = 3

## Load comparison files
# Behavior
b_path = "/home/duynam/MPC_formation_changing/Soria2021/"
b_path0 = np.load(b_path+"path_0.npy")
b_path1 = np.load(b_path+"path_1.npy")
b_path2 = np.load(b_path+"path_2.npy")

# Proposed
p_path = "/home/duynam/MPC_formation_changing/Proposed/"
p_path0 = np.load(p_path+"path_0.npy")
p_path1 = np.load(p_path+"path_1.npy")
p_path2 = np.load(p_path+"path_2.npy")

b_headings = []
for i in range(1,len(b_path0)):
    heading = b_path0[i,4:6]/np.linalg.norm(b_path0[i,4:6]) \
            + b_path1[i,4:6]/np.linalg.norm(b_path1[i,4:6]) \
            + b_path2[i,4:6]/np.linalg.norm(b_path2[i,4:6])
    b_headings.append(np.linalg.norm(heading)/NUM_UAV)

p_headings = []
for i in range(1,len(p_path0)):
    heading = p_path0[i,4:6]/np.linalg.norm(p_path0[i,4:6]) \
            + p_path1[i,4:6]/np.linalg.norm(p_path1[i,4:6]) \
            + p_path2[i,4:6]/np.linalg.norm(p_path2[i,4:6])
    p_headings.append(np.linalg.norm(heading)/NUM_UAV)


plt.figure(figsize=(5,3))
ax = plt.subplot(111)
plt.plot(b_path0[1:,0], b_headings, 'b-', label="Rigid formation")
plt.plot(p_path0[1:,0], p_headings, 'r-', label="Proposed")
plt.xlabel("Time (s)")
plt.ylabel("Order")
plt.xlim([0, max(b_path0[-1,0],p_path0[-1,0])])
plt.legend()
plt.tight_layout()
plt.grid(True)

# # inset axes....
# axins = zoomed_inset_axes(ax,2,loc='lower right')
# axins.plot(b_path0[1:,0], b_headings, 'b-')
# axins.plot(p_path0[1:,0], p_headings, 'r-')
# x1,x2,y1,y2 = 9.9,20.1, 0.981,1.001
# axins.set_xlim(x1,x2)
# axins.set_ylim(y1,y2)

# pp,p1,p2 = mark_inset(ax,axins,loc1=1,loc2=3)
# pp.set_fill(True)
# pp.set_facecolor("white")
# pp.set_edgecolor("k")
plt.savefig("/home/duynam/MPC_formation_changing/PlotResults/results/strategy_order.pdf", format="pdf", bbox_inches="tight")
plt.show()