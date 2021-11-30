# -*- coding: utf-8 -*-
from helper.utils import diminuir_pontos, distancia_rota, dist_euclidiana, simulate_points, pseudo3D
from curves import bSpline
from helper.ambiente import Pontos
from classic.rrt import RRT
from curves.spline3D import generate_curve

import matplotlib.pyplot as plt
import numpy as np
import time
import math

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None

def calc_distance_and_angle(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.sqrt(dx ** 2 + dy ** 2)
    theta = math.atan2(dy, dx)
    return d, theta

def steer(from_node, to_node, extend_length=float("inf")):
    new_node = Node(from_node.x, from_node.y)
    d, theta = calc_distance_and_angle(new_node, to_node)

    new_node.path_x = [new_node.x]
    new_node.path_y = [new_node.y]

    if extend_length > d:
        extend_length = d

    nExpand = int(math.floor(extend_length / 0.5))

    for _ in range(nExpand):
        new_node.x += 0.5 * math.cos(theta)
        new_node.y += 0.5 * math.sin(theta)
        new_node.path_x.append(new_node.x)
        new_node.path_y.append(new_node.y)

    d, _ = calc_distance_and_angle(new_node, to_node)
    if d <= 0.5:
        new_node.path_x.append(to_node.x)
        new_node.path_y.append(to_node.y)

    new_node.parent = from_node

    return new_node

def run(show=False, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None):
    p = Pontos()

    ed = 2.0

    obstacleList = []

    if startx == None:
        for px1, py1 in zip(p.xobs, p.yobs):
            obstacleList.append((px1, py1, 1.0))
    else:
        for px1, py1 in zip(vmx, vmy):
            obstacleList.append((px1, py1, 1.0))

    start = time.time()

    pathx1, pathy1 = [], []
    pathx2, pathy2 = [], []

    if startx == None:
        auxSx, auxSy = p.xs, p.ys
    else:
        auxSx, auxSy = startx, starty

    auxEx, auxEy = p.xt, p.yt
    count = 0
    ppx, ppy = [], []

    # print(p.limiar)
    while len(ppx) == 0:
        if count%2 == 0:
            # print("olhando de " + str(auxSx) + " - " + str(auxSy))
            # print("ate " + str(auxEx) + " - " + str(auxEy))
            rrt = RRT(start=[auxSx, auxSy],
                    goal=[auxEx, auxEy],
                    rand_area=[0, max(p.limiar)],
                    obstacle_list=obstacleList, expand_dis=ed, path_resolution=0.2, goal_sample_rate=1)
        else:
            # print("olhando de " + str(auxEx) + " - " + str(auxEy))
            # print("ate " + str(auxSx) + " - " + str(auxSy))
            rrt = RRT(start=[auxEx, auxEy],
                    goal=[auxSx, auxSy],
                    rand_area=[0, max(p.limiar)],
                    obstacle_list=obstacleList, expand_dis=ed, path_resolution=0.2, goal_sample_rate=1)

        path = rrt.planning(animation=show, bi=True, iterations=p.limiar * 4)

        if count%2==0:
            auxSx, auxSy = path[0][0], path[0][1]
            # print("s foi para " + str(auxSx) + " - " + str(auxSy))
            path.reverse()
            for value in path:
                pathx1.append(value[0])
                pathy1.append(value[1])
        else:
            auxEx, auxEy = path[0][0], path[0][1]
            # print("e foi para " + str(auxEx) + " - " + str(auxEy))
            path.reverse()
            for value in path:
                pathx2.append(value[0])
                pathy2.append(value[1])
        
        count += 1

        # try:
        # print("distancia:")
        # print(math.sqrt(((pathx1[-1] - pathx2[-1])**2) + ((pathy1[-1] - pathy2[-1])**2)))
        radius_robot = 1.0
        if count > 1:
            if dist_euclidiana(pathx1[-1], pathy1[-1], pathx2[-1], pathy2[-1]) < radius_robot:
                new_node = steer(Node(pathx1[-1], pathy1[-1]), Node(pathx2[-1], pathy2[-1]), ed)
                if RRT.check_collision(new_node, obstacleList):
                    pathx2.reverse()
                    pathy2.reverse()
                    ppx = np.concatenate((pathx1, pathx2), axis=0)
                    ppy = np.concatenate((pathy1, pathy2), axis=0)
                    # print("tempo")
                    # print(time.time() - start)
                    # plt.plot(ox, oy, ".k")
                    # plt.plot(ppx,ppy,"-b")
                    # plt.show()
        # except:
        #     print("path dont found")


    # print("1")
    # _, px, py = distancia_rota(path)
    
    if startx==None: startx, starty = p.xs, p.ys
    a, b = diminuir_pontos(ppx, ppy, p.xobs, p.yobs)
    
    # x3D, y3D, z3D = pseudo3D(a, b, vmx, vmy, p.visXz, p.visYz, 1.2)
    # pontosComCurva3D = generate_curve(x3D, y3D, z3D)
    

    if abs(startx - p.xs) < 5 and abs(starty - p.ys) < 5:
        curv = bSpline.B_spline(a, b)  
    else: 
        curv = bSpline.B_spline(a[:-1], b[:-1]) 
    
    xnew, ynew = curv.get_curv()

    cam1, cam2 = simulate_points(xnew[-1], p.xt, ynew[-1], p.yt)

    # PRECISA SUAVIZAR AO FAZER ISSO
    xnew = np.concatenate((xnew, cam1), axis=0).tolist()
    ynew = np.concatenate((ynew, cam2), axis=0).tolist()

    end = time.time() - start

    if path is None:
        print("Nao achou caminho")
    else:
        if show:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-b')
            plt.plot(a, b, "-r")
            plt.grid(True)
            plt.pause(0.01) 
            plt.show()
    
    distancia2 = distancia_rota(xnew, ynew)   

    # print(x3D)
    # print(pontosComCurva3D[:][0])

    # plt.plot(pontosComCurva3D[:][0], pontosComCurva3D[:][1], "-b")
    # plt.show()

    return distancia2, end, xnew, ynew
    # return distancia2, end, pontosComCurva3D[:][0], pontosComCurva3D[:][1], pontosComCurva3D[:][2]
    
