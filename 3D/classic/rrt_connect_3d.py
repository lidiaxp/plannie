# https://github.com/motion-planning/rrt-algorithms
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
from helper.ambiente import Pontos

from rrt.rrt_connect import RRTConnect
from search_space.search_space import SearchSpace
from utilities.plotting import Plot

from curves.spline3D import generate_curve
from helper.utils import distancia_rota3D, diminuir_pontos3D, tirarRepertido3D
import time

def run(show=False, vmx=[None], vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    # print("RRTC 3D")
    p = Pontos()
    start = time.time()

    x_min = 0
    x_max = p.limiar[0]+1
    y_min = 0
    y_max = p.limiar[1]+1
    z_min = 0
    z_max = 6

    X_dimensions = np.array([(x_min, x_max), (y_min, y_max), (z_min, z_max)])  # dimensions of Search Space

    # obstacles
    if vmx[0] == None:
        Obstacles = []
        for x1, y1, z1 in zip(p.xobs, p.yobs, p.zobs):
            Obstacles.append((x1-0.5, y1-0.5, z1-0.5, x1+0.5, y1+0.5, z1+0.5))
        Obstacles = np.array(Obstacles)
        x_init = (p.xs, p.ys, p.zs)  # starting location
    else:
        Obstacles = []
        for x1, y1, z1 in zip(vmx, vmy, vmz):
            Obstacles.append((x1-0.5, y1-0.5, z1-0.5, x1+0.5, y1+0.5, z1+2))
        Obstacles = np.array(Obstacles)
        x_init = (startx, starty, startz)
        
    x_goal = (p.xt, p.yt, p.zt)  # goal location

    

    Q = np.array([2])  # length of tree edges
    r = 0.5  # length of smallest edge to check for intersection with obstacles
    max_samples = 1024  # max number of samples to take before timing out
    prc = 0.1  # probability of checking for a connection to goal

    # create search space
    X = SearchSpace(X_dimensions, Obstacles)


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0,30)
    ax.set_ylim(0,30)
    ax.set_zlim(0,5)
    ax.plot3D(p.xobs,p.yobs,p.zobs, ".k")
    plt.show()
    
    # # create rrt_search
    # print(x_init)
    # print(x_goal)
    rrt_connect = RRTConnect(X, Q, x_init, x_goal, max_samples, r, prc)

    path = []
    path = rrt_connect.rrt_connect()

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

    if show:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(x,y,z, "-r")
        ax.plot3D(xxx1,yyy1,zzz1, "-y")
        ax.set_xlim(0,20)
        ax.set_ylim(0,20)
        ax.set_zlim(0,5)
        ax.plot3D(p.xobs,p.yobs,p.zobs, ".k")
        ax.plot3D(xx,yy,zz, "-b")
        plt.show()

    distance = distancia_rota3D(xx, yy, zz)

    return distance, time.time() - start, xx, yy, zz
