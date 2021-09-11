# -*- coding: utf-8 -*-
import random
import math
import time
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
from helper.utils import dist_euclidiana3D, distancia_rota3D, diminuir_pontos3D, pseudo3D, tirarRepertido3D, colidir3D
from helper.ambiente import Pontos
from curves import bSpline
from curves.spline3D import generate_curve

# parametros
N_SAMPLE = 2000  # numero de amostras
N_KNN = 15  # número de arestas a partir de um ponto amostrado
MAX_EDGE_LEN = 20.0  # maximo comprimento de uma linha [m]

class Node:
    def __init__(self, x, y, z, cost, pind):
        self.x = x
        self.y = y
        self.z = z
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.z) + "," + str(self.cost) + "," + str(self.pind)

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


def PRM_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, rr, show):
    obkdtree = KDTree(np.vstack((ox, oy, oz)).T)

    sample_x, sample_y, sample_z = sample_points(sx, sy, sz, gx, gy, gz, rr, ox, oy, oz, obkdtree)
    if show:
        plt.plot(sample_x, sample_y, ".b")

    road_map = generate_roadmap(sample_x, sample_y, sample_z, rr, obkdtree, ox, oy, oz)

    rx, ry, rz = dijkstra_planning(
        sx, sy, sz, gx, gy, gz, ox, oy, oz, rr, road_map, sample_x, sample_y, sample_z, show)

    return rx, ry, rz


def is_collision(sx, sy, sz, gx, gy, gz, rr, ox, oy, oz):
    return colidir3D(ox, oy, oz, sx, sy, sz, gx, gy, gz, value=3)
    # x = sx
    # y = sy
    # dx = gx - sx
    # dy = gy - sy
    # dz = gz - sz
    # yaw = math.atan2(gy - sy, gx - sx)
    # d = math.hypot(dx, dy)

    # if d >= MAX_EDGE_LEN:
    #     return True

    # D = rr
    # nstep = round(d / D)

    # for _ in range(int(nstep)):
    #     _, dist = okdtree.search(np.array([x, y, z]).reshape(3, 1))
    #     if dist[0] <= rr:
    #         return True  # colide
    #     x += D * math.cos(yaw)
    #     y += D * math.sin(yaw)

    # # checar no ponto final
    # _, dist = okdtree.search(np.array([gx, gy, gz]).reshape(3, 1))
    # if dist[0] <= rr:
    #     return True  # colide

    # return False  # OK


def generate_roadmap(sample_x, sample_y, sample_z, rr, obkdtree, ox, oy, oz):
    road_map = []
    nsample = len(sample_x)
    skdtree = KDTree(np.vstack((sample_x, sample_y, sample_z)).T)

    for (_, ix, iy, iz) in zip(range(nsample), sample_x, sample_y, sample_z):

        index, _ = skdtree.search(
            np.array([ix, iy, iz]).reshape(3, 1), k=nsample)
        inds = index[0]
        edge_id = []

        for ii in range(1, len(inds)):
            nx = sample_x[inds[ii]]
            ny = sample_y[inds[ii]]
            nz = sample_z[inds[ii]]

            if not is_collision(ix, iy, iz, nx, ny, nz, rr, ox, oy, oz):
                edge_id.append(inds[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    #  plot_road_map(road_map, sample_x, sample_y)

    return road_map


def dijkstra_planning(sx, sy, sz, gx, gy, gz, ox, oy, oz, rr, road_map, sample_x, sample_y, sample_z, show):
    nstart = Node(sx, sy, sz, 0.0, -1)
    ngoal = Node(gx, gy, gz, 0.0, -1)

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
            dz = sample_z[n_id] - current.z
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id], sample_z[n_id],
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
    rx, ry, rz = [ngoal.x], [ngoal.y], [ngoal.z]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x)
        ry.append(n.y)
        rz.append(n.z)
        pind = n.pind

    return rx, ry, rz


def plot_road_map(road_map, sample_x, sample_y, sample_z):  
    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]

            plt.plot([sample_x[i], sample_x[ind]],
                     [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, sz, gx, gy, gz, rr, ox, oy, oz, obkdtree):
    maxx = max(ox)
    maxy = max(oy)
    maxz = max(oz)
    minx = min(ox)
    miny = min(oy)
    minz = min(oz)

    sample_x, sample_y, sample_z = [], [], []

    while len(sample_x) <= N_SAMPLE:
        tx = (random.random() * (maxx - minx)) + minx
        ty = (random.random() * (maxy - miny)) + miny
        tz = (random.random() * (maxz - minz)) + minz

        _, dist = obkdtree.search(np.array([tx, ty, tz]).reshape(3, 1))

        if dist[0] >= rr:
            sample_x.append(tx)
            sample_y.append(ty)
            sample_z.append(tz)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_z.append(sz)
    sample_x.append(gx)
    sample_y.append(gy)
    sample_z.append(gz)

    return sample_x, sample_y, sample_z


def run(show=False, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None):
    start = time.time()

    p = Pontos()
    if startx == None:
        px = [p.xs, p.xt]
        py = [p.ys, p.yt]
        pz = [p.zs, p.zt]
    else:
        px = [startx, p.xt]
        py = [starty, p.yt]
        py = [startz, p.zt]

    robot_size = 1.0  # [m]

    ox = p.xobs
    oy = p.yobs
    oz = p.zobs

    if show:
        plt.plot(ox, oy, ".k")
        plt.plot(px[0], py[0], "^c")
        for i in range(1, len(px), 1):
            plt.plot(px[i], py[i], "^r")
        plt.grid(True)
        plt.axis("equal")

    if startx == None:
        rx, ry, rz = PRM_planning(px[0], py[0], pz[0], px[1], py[1], pz[1], ox, oy, oz, robot_size, show)
    else:
        rx, ry, rz = PRM_planning(startx, starty, startz, px[1], py[1], pz[1], vmx, vmy, vmz, robot_size, show)
    
    a, b, c = diminuir_pontos3D(rx, ry, rz, p.xobs, p.yobs, p.zobs)
    a, b, c = tirarRepertido3D(a, b, c)
    a, b, c = generate_curve(a, b, c)

    # x3D, y3D, z3D = pseudo3D(a, b, vmx, vmy, p.visXz, p.visYz, 2)

    end = time.time()-start

    print("Tempo PRM: " + str(end))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.plot3D(p.xobs, p.yobs, p.zobs, ".k")
    ax.plot3D(rx, ry, rz, "-b")
    ax.plot3D(a, b, c, "-r")
    plt.show()

    distancia = distancia_rota3D(a, b, c)
    
    return distancia, end, a, b, c
    
    # curv = bSpline.B_spline(a, b) if abs(startx - px[0]) < 5 and abs(starty - py[0]) < 5 else bSpline.B_spline(a[:-1], b[:-1]) 
    # xnew, ynew = curv.get_curv()

    # xnew = np.concatenate(([px[1]], xnew), axis=0).tolist()
    # ynew = np.concatenate(([py[1]], ynew), axis=0).tolist()
    
    # end = time.time()-start
    
    # # plt.plot(ox, oy, ".k")
    # # plt.plot(px[0], py[0], "^c")
    # # plt.plot(rx, ry, "-r")
    # # plt.show()

    # # assert rx, 'Cannot found path'

    # if show:
    #     plt.plot(ox, oy, ".k")
    #     plt.plot(px[0], py[0], "^c")
    #     plt.plot(px[1], py[1], "^c")
    #     plt.plot(a, b, "-r")
    #     plt.plot(rx, ry, "-b")
    #     plt.show()

    # distancia = distancia_rota(xnew, ynew)

    # return distancia, end, xnew, ynew

if __name__ == '__main__':
    run()
