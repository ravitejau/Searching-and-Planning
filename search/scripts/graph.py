#!/usr/bin/env python
import matplotlib.pyplot as plt

gbfs_md = [0.13, 1.72, 2.88, 3.67]
astar_md = [0.39, 1.67, 6.53, 25.86]
gbfs = [0.31, 0.41, 1.00, 1.53]
astar = [0.47, 1.50, 4.11, 15.37]
plt.xlabel("Grid Dimension With Euclidean Distance Heuristic for GBFS and Astar")
plt.plot([2,4,8,16], gbfs_md, color='red', label='GBFS_Manhattan')
plt.plot([2,4,8,16], astar_md, color='blue', label='Astar_Manhattan')
plt.plot([2,4,8,16], gbfs, color='orange', label='GBFS_Euclidean')
plt.plot([2,4,8,16], astar, color='purple', label='Astar_Euclidean')
plt.legend(loc = 'upper left')
plt.show()
