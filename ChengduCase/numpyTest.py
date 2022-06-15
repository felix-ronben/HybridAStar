from matplotlib import projections
import numpy as np
import matplotlib.pyplot as plt

maps = np.load('maps.npz', allow_pickle=True)
obmap = maps['obmap']
ermap = maps['ermap']
costmap = maps['costmap']
(nx, ny) = maps['obmap'].shape
node = open('cost.txt', mode='w+')
couter = 0
node.write('ID,X,Y,Z\n')
for i in range(nx):
    for j in range(ny):
        if costmap[i][j] != -1:
            node.write('%d,%.2f,%.2f,%.2f\n' % (couter, i, j, costmap[i][j]))
            couter += 1
node.close()
