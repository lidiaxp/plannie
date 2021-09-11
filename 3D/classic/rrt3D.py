# https://github.com/motion-planning/rrt-algorithms
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
from helper.ambiente import Pontos

from rrt.rrt import RRT
from search_space.search_space import SearchSpace
from utilities.plotting import Plot

from curves.spline3D import generate_curve
from helper.utils import distancia_rota3D, diminuir_pontos3D, tirarRepertido3D
import time

def run(show=False, vmx=[None], vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    # print("RRT 3D")
    p = Pontos()
    start = time.time()

    x_min = 0
    x_max = p.limiar[0]
    y_min = 0
    y_max = p.limiar[1]
    z_min = 0
    z_max = 5

    X_dimensions = np.array([(x_min, x_max), (y_min, y_max), (z_min, z_max)])  # dimensions of Search Space

    # # obstacles
    if vmx[0] == None:
        Obstacles = []
        for x1, y1, z1 in zip(p.xobs, p.yobs, p.zobs):
            # if x1 - 1 >= 0 and y1 - 1 >= 0 and z1 - 1 >= 0 and x1 + 1 < x_max and y1 + 1 < y_max and z1 + 1 < z_max:
            Obstacles.append((x1-1, y1-1, z1-1, x1+1, y1+1, z1+1))
        Obstacles = np.array(Obstacles)
        x_init = (p.xs, p.ys, p.zs)  # starting location
    else:
        Obstacles = []
        for x1, y1, z1 in zip(vmx, vmy, vmz):
            Obstacles.append((x1-1, y1-1, z1-1, x1+1, y1+1, z1+1))
        Obstacles = np.array(Obstacles)
        x_init = (startx, starty, startz)

    x_goal = (p.xt, p.yt, p.zt)  # goal location
    
    Q = np.array([(8, 4)])  # length of tree edges
    r = 1  # length of smallest edge to check for intersection with obstacles
    max_samples = 1024  # max number of samples to take before timing out
    prc = 0.1  # probability of checking for a connection to goal

    # create Search Space
    X = SearchSpace(X_dimensions, Obstacles)

    # create rrt_search
    rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)    
    path = rrt.rrt_search()

    x,y,z = [],[],[]
    if path == None:
        x = pseudox[1:]
        y = pseudoy[1:]
        z = pseudoz[1:]
    else:
        for caminho in path:
            x.append(caminho[0])
            y.append(caminho[1])
            z.append(caminho[2])
    
    xxx1, yyy1, zzz1 = diminuir_pontos3D(x, y, z, p.xobs, p.yobs, p.zobs, value=0.8)
    xxx1, yyy1, zzz1 = tirarRepertido3D(xxx1, yyy1, zzz1)
    xx, yy, zz = generate_curve(xxx1, yyy1, zzz1)

    if show == 0:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(x,y,z, "-r")
        ax.plot3D(xxx1,yyy1,zzz1, "-y")
        ax.set_xlim(0,21)
        ax.set_ylim(0,21)
        ax.set_zlim(0,5)
        ax.plot3D(p.xobs,p.yobs,p.zobs, ".k")
        ax.plot3D(xx,yy,zz, "-b")
        plt.show()

    # plot
    # plot = Plot("rrt_3d")
    # plot.plot_tree(X, rrt.trees)
    # if path is not None:
    #     plot.plot_path(X, path)
    # plot.plot_obstacles(X, Obstacles)
    # plot.plot_start(X, x_init)
    # plot.plot_goal(X, x_goal)
    # plot.draw(auto_open=True)

    distance = distancia_rota3D(xx, yy, zz)

    return distance, time.time() - start, xx, yy, zz