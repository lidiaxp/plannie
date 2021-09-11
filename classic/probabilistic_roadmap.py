# -*- coding: utf-8 -*-
import random
import math
import time
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
from helper.utils import dist_euclidiana, distancia_rota, diminuir_pontos
from helper.ambiente import Pontos
from curves import bSpline

# parametros
N_SAMPLE = 200  # numero de amostras
N_KNN = 15  # número de arestas a partir de um ponto amostrado
MAX_EDGE_LEN = 20.0  # maximo comprimento de uma linha [m]

class Node:
    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

class KDTree:
    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        if len(inp.shape) >= 2:  
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist

        dist, index = self.tree.query(inp, k=k)
        return index, dist

    def search_in_distance(self, inp, r):
        index = self.tree.query_ball_point(inp, r)
        return index


def PRM_planning(sx, sy, gx, gy, ox, oy, rr, show):
    obkdtree = KDTree(np.vstack((ox, oy)).T)

    sample_x, sample_y = sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree)
    if show:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, rr, obkdtree)

    rx, ry = dijkstra_planning(
        sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y, show)

    return rx, ry


def is_collision(sx, sy, gx, gy, rr, okdtree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    nstep = round(d / D)

    for _ in range(int(nstep)):
        _, dist = okdtree.search(np.array([x, y]).reshape(2, 1))
        if dist[0] <= rr:
            return True  # colide
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # checar no ponto final
    _, dist = okdtree.search(np.array([gx, gy]).reshape(2, 1))
    if dist[0] <= rr:
        return True  # colide

    return False  # OK


def generate_roadmap(sample_x, sample_y, rr, obkdtree):
    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (_, ix, iy) in zip(range(nsample), sample_x, sample_y):

        index, _ = skdtree.search(
            np.array([ix, iy]).reshape(2, 1), k=nsample)
        inds = index[0]
        edge_id = []

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obkdtree):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, gx, gy, ox, oy, rr, road_map, sample_x, sample_y, show):
    nstart = Node(sx, sy, 0.0, -1)
    ngoal = Node(gx, gy, 0.0, -1)

    openset, closedset = dict(), dict()
    openset[len(road_map) - 2] = nstart

    path_found = True

    while True:
        if not openset:
            # print("Cannot find path")
            path_found = False
            break

        c_id = min(openset, key=lambda o: openset[o].cost)
        current = openset[c_id]

        if show and len(closedset.keys()) % 2 == 0:
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            # goal is found!
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # remover item do openSet
        del openset[c_id]

        # adicioanr item ao closedSet
        closedset[c_id] = current

        # expandir o grid de busca
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closedset:
                continue
            # Caso contrario, se ja estiver no conjunto aberto
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    if path_found is False:
        return [], []

    # gerar o caminho final
    rx, ry = [ngoal.x], [ngoal.y]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        pind = n.pind

    return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    minx = min(ox)
    miny = min(oy)

    sample_x, sample_y = [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() * (maxx - minx)) + minx
        ty = (random.random() * (maxy - miny)) + miny

        _, dist = obkdtree.search(np.array([tx, ty]).reshape(2, 1))

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y


def run(show=False, vmx=None, vmy=None, startx=None, starty=None, p1=None):
    start = time.time()

    p = Pontos()
    px = [p.xs, p.xt]
    py = [p.ys, p.yt]
    robot_size = 1.0  # [m]

    ox = p.xobs
    oy = p.yobs

    if show:
        plt.plot(ox, oy, ".k")
        plt.plot(px[0], py[0], "^c")
        for i in range(1, len(px), 1):
            plt.plot(px[i], py[i], "^r")
        plt.grid(True)
        plt.axis("equal")

    if startx == None:
        rx, ry = PRM_planning(px[0], py[0], px[1], py[1], ox, oy, robot_size, show)
    else:
        rx, ry = PRM_planning(startx, starty, px[1], py[1], vmx, vmy, robot_size, show)
    
    a, b = diminuir_pontos(rx, ry, p.xobs, p.yobs)
    
    curv = bSpline.B_spline(a, b) if abs(startx - px[0]) < 5 and abs(starty - py[0]) < 5 else bSpline.B_spline(a[:-1], b[:-1]) 
    xnew, ynew = curv.get_curv()

    xnew = np.concatenate(([px[1]], xnew), axis=0).tolist()
    ynew = np.concatenate(([py[1]], ynew), axis=0).tolist()
    
    end = time.time()-start
    
    # plt.plot(ox, oy, ".k")
    # plt.plot(px[0], py[0], "^c")
    # plt.plot(rx, ry, "-r")
    # plt.show()

    # assert rx, 'Cannot found path'

    if show:
        plt.plot(ox, oy, ".k")
        plt.plot(px[0], py[0], "^c")
        plt.plot(px[1], py[1], "^c")
        plt.plot(a, b, "-r")
        plt.plot(rx, ry, "-b")
        plt.show()

    distancia = distancia_rota(xnew, ynew)

    return distancia, end, xnew, ynew

if __name__ == '__main__':
    run()
