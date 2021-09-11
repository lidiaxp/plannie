import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d

x = np.arange(10)
y = np.arange(10)
z = np.arange(10)

ax = plt.axes(projection = "3d")
ax.plot3D(x, y, z, "-b")
ax.set_xlim(0,10) 
ax.set_ylim(0,5) 
ax.set_zlim(0,6) 
plt.pause(0.01)
plt.show()