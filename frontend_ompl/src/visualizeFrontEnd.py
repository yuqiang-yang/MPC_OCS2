import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
fig = plt.figure()
ax = plt.axes(projection='3d')

res = np.loadtxt('/home/vision/yq_ws/my_mpc/src/frontend_ompl_ros/src/RRTstar.txt')
ax.plot3D(res[:,0],res[:,1],res[:,2])
plt.show()
