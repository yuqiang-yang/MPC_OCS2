from matplotlib import pyplot as plt
from math import pow,log
import numpy as np
def value(h,delta_,mu_):
    if h > delta_:
        return -mu_ * log(h)
    else:
        return mu_ * (-log(delta_) + (0.5) * pow((h - 2.0 * delta_) / delta_, 2.0) - (0.5));
def gradient(h,delta_,mu_):
    if h > delta_:
        return -mu_/h
    else:
        return mu_ * ((h - 2.0 * delta_) / (delta_ * delta_))

t = np.linspace(-1,2,50)
v = np.zeros(50)
g = np.zeros(50)
for i in range(50):
    v[i] = value(t[i],1e-3,1e-3)
    g[i] = gradient(t[i],1e-3,1e-3)

plt.title('value')
plt.plot(t,v)
plt.figure()
plt.title('gradient')
plt.plot(t,g)
plt.show()
