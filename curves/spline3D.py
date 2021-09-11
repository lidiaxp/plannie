import scipy as sp
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import splprep, splev, interp2d

def generate_curve(x, y, z):
    tck, _ = splprep([x,y,z], s=0)  # Generate function out of provided points, default k = 3
    unew = np.arange(0, 1.00, 0.005)
    newPoints = splev(unew, tck)          # Creating spline points

    return newPoints

if __name__ == '__main__':
    x = np.array([1,2,3,4,5])   
    y = np.array([1,2,3,4,5])   
    z = np.array([3,3,3,5,5])   

    newPoints = generate_curve(x, y, z)

    ax = plt.axes(projection = "3d")
    ax.plot3D(x, y, z, 'go')    
    ax.plot3D(newPoints[:][0], newPoints[:][1], newPoints[:][2], 'r-')   
    plt.show()