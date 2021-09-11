import numpy as np
from classic.simpleAstar import Astar2D, Astar3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
from curves.spline3D import generate_curve
import matplotlib.image as mpimg
import os
import time
from helper.ambiente import Pontos
from helper.utils import *

def run(show=0, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    # test 2D A*

    # ignore zeros, use just a 2D points
    # grid2d = np.array([[5, 0], [5, 2], [5, 3], [5, 4], [5, 5], [5, 6], [5, 8], [5, 9],
    #                    [7, 0], [7, 1], [7, 2], [7, 3], [7, 4], [7, 5], [7, 6], [7, 7],
    #                    [3, 0], [3, 1], [3, 2], [3, 3], [3, 4], [3, 5], [3, 6], [3, 7],
    #                    [0, 0], [9, 9]])

    # start_point = (2, 2)
    # end_point = (8, 5)

    # asa2d = Astar2D()

    # # pass search space over shape argument, to optimize results
    # path2d = asa2d.generate_path(start_point=start_point, end_point=end_point, grid_map=grid2d, shape=(10, 10))

    # # display results for 2d search
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    
    # ax.plot(grid2d[:, 0], grid2d[:, 1], 'rx')
    # ax.plot(path2d[:, 0], path2d[:, 1], c='g')
    # ax.plot(start_point[0], start_point[1], 'gD')
    # ax.plot(end_point[0], end_point[1], c='g', marker='*')
    
    # plt.show()

    # test 3D A*
    tempoInicio = time.time()
    # p = ambiente.Pontos()
    p = Pontos()

    # ignore zeros, create a point a 3D cloud
    if startx != None:
        # print("5")
        x = np.asarray(p.xobs) # np.array([0, 10, 0, 10, 10, 0, 0, 10, 4, 5, 9, 4, 6, 3, 8, 2, 6, 8, 6, 4, 2, 8, 8, 5, 8])
        y = np.asarray(p.yobs) # np.array([0, 10, 0, 0, 10, 10, 10, 0, 4, 6, 8, 2, 7, 3, 4, 8, 8, 2, 1, 3, 1, 8, 8, 3, 7])
        z = np.asarray(p.zobs) # np.array([0, 10, 10, 0, 0, 0, 10, 10, 7, 3, 3, 1, 4, 6, 5, 3, 9, 2, 3, 3, 1, 2, 6, 1, 5])

        # x = np.array([0, 10, 0, 10, 10, 0, 0, 10, 4, 5, 9, 4, 6, 3, 8, 2, 6, 8, 6, 4, 2, 8, 8, 5, 8])
        # y = np.array([0, 10, 0, 0, 10, 10, 10, 0, 4, 6, 8, 2, 7, 3, 4, 8, 8, 2, 1, 3, 1, 8, 8, 3, 7])
        # z = np.array([0, 10, 10, 0, 0, 0, 10, 10, 7, 3, 3, 1, 4, 6, 5, 3, 9, 2, 3, 3, 1, 2, 6, 1, 5])

        point_cloud3d = np.c_[x.reshape(-1), y.reshape(-1), z.reshape(-1)]

        start_point = (int(p.xs), int(p.ys), int(p.zs))
    else:
        # print("6")
        x = np.asarray(vmx) # np.array([0, 10, 0, 10, 10, 0, 0, 10, 4, 5, 9, 4, 6, 3, 8, 2, 6, 8, 6, 4, 2, 8, 8, 5, 8])
        y = np.asarray(vmy) # np.array([0, 10, 0, 0, 10, 10, 10, 0, 4, 6, 8, 2, 7, 3, 4, 8, 8, 2, 1, 3, 1, 8, 8, 3, 7])
        z = np.asarray(vmz) # np.array([0, 10, 10, 0, 0, 0, 10, 10, 7, 3, 3, 1, 4, 6, 5, 3, 9, 2, 3, 3, 1, 2, 6, 1, 5])

        point_cloud3d = np.c_[x.reshape(-1), y.reshape(-1), z.reshape(-1)]
        
        start_point = (int(startx), int(starty), int(startz))

    
    end_point = (int(p.xt), int(p.yt), int(p.zt))

    asa3d = Astar3D()

    # pass search space over shape argument, to optimize results
    path3d = asa3d.generate_path(start_point=start_point, end_point=end_point, grid_map=point_cloud3d, shape=(121, 21, 26))
    print(path3d)
    print("Time: " + str(time.time() - tempoInicio))

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # ax.scatter(point_cloud3d[:, 0], point_cloud3d[:, 1], point_cloud3d[:, 2], c='r', marker='x', zorder=0)

    # # ax.scatter(path3d[:, 0], path3d[:, 1], path3d[:, 2], c='g', marker='.', zorder=0)
    # ax.scatter(start_point[0], start_point[1], start_point[2], c='g', marker='D', zorder=1)
    # ax.scatter(start_point[0], start_point[1], start_point[2], c='g', marker='*', zorder=1)
    # # ax.plot3D(path3d[:, 0], path3d[:, 1], path3d[:, 2], "-k")

    # # for i in range(len(path3d) - 1):
    # #     xl = [path3d[i][0], path3d[i + 1][0]]
    # #     yl = [path3d[i][1], path3d[i + 1][1]]
    # #     zl = [path3d[i][2], path3d[i + 1][2]]
    # #     line = art3d.Line3D(xl, yl, zl, c='g')
    # #     ax.add_line(line)

    # # ax.plot3D(path3d[:, 0], path3d[:, 1], path3d[:, 2], "-k")
    # # ax.plot3D(a,b,c, "-b")
    # plt.show()
    

    # for i in point_cloud3d[:, 0]:
    #     print(i)
    # for i in point_cloud3d[:, 1]:
    #     print(i)
    # for i in point_cloud3d[:, 2]:
    #     print(i)

    # print(path3d)
    # print(path3d[:, 0])
    try:
        a, b, c = path3d[:, 0], path3d[:, 1], path3d[:, 2]
    except:
        a, b, c = pseudox[1:], pseudoy[1:], pseudoz[1:]
    a, b, c = diminuir_pontos3D(a, b, c, p.xobs, p.yobs, p.zobs, apf=False)
    a, b, c = tirarRepertido3D(a, b, c)
    a, b, c = generate_curve(a, b, c)
    
    if show:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(a,b,c, "-r")
        ax.set_xlim(0,21)
        ax.set_ylim(0,21)
        ax.set_zlim(0,5)
        ax.plot3D(p.xobs,p.yobs,p.zobs, ".k")
        plt.show()

    distancia = distancia_rota3D(a, b, c)

    # print(a)

    return distancia, time.time() - tempoInicio, a, b, c

if __name__ == '__main__':
    run()