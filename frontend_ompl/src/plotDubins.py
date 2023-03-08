import numpy as np
from matplotlib import pyplot as plt
result = np.loadtxt("/home/vision/yq_ws/my_mpc/devel/lib/frontend_ompl/path.txt")
p = plt.figure()
plt.plot(result[:,0],result[:,1])
plt.show()
