# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import numpy as np
from classic import probabilistic_roadmap as ajudante
from helper.utils import dist_euclidiana, distancia_rota, diminuir_pontos, simulate_points, pseudo3D, draw_bar, draw_cylinder
from helper.ambiente import Pontos
from curves import bSpline
from curves.spline3D import generate_curve
import time

class AStarPlanner:
    def __init__(self, ox, oy, reso, rr):
        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, pind):
            self.x = x  # index 
            self.y = y  # index
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy, show, vmx, vmy):
        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            if len(open_set) == 0:
                # print("Open set is empty..")
                _, _, rotax, rotay = ajudante.run(show=False, vmx=vmx, vmy=vmy, startx=sx, starty=sy)
                return rotax, rotay
                # break

            c_id = min(
                open_set, key=lambda o: open_set[o].cost + dist_euclidiana(ngoal.x, ngoal.y, open_set[o].x, open_set[o].y))
            current = open_set[c_id]

            if show:  
                plt.plot(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny), "xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y:
                ngoal.pind = current.pind
                ngoal.cost = current.cost
                break

            # tirar item do openSet
            del open_set[c_id]

            # adicionar item no openSet
            closed_set[c_id] = current

            # expandir o grid de busca
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)


                # se o no n for seguro faz nd
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # novo no
                else:
                    if open_set[n_id].cost > node.cost:
                        # melhor caminho ate agora, gravar
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # fazer caminho final
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return rx, ry

    def calc_grid_position(self, index, minp):
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # checar colisao
        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        # gerar mapa de obstaculo
        self.obmap = [[False for i in range(int(self.ywidth))] for i in range(int(self.xwidth))]
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.sqrt((iox - x) ** 2 + (ioy - y) ** 2)
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

# --------------------------------------------------------------------------------------------------

def run(show=False, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None):
    start = time.time()
    pathx = []
    pathy = []

    p = Pontos()
    xs = p.xs
    ys = p.ys
    xt = p.xt
    yt = p.yt
    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    ox = p.xobs
    oy = p.yobs

    if startx == None:
        vmx = ox
        vmy = oy

    if show:  
        plt.plot(vmx, vmy, ".k")
        plt.plot(xs, ys, "og")
        plt.plot(xt, yt, "xb")
        plt.grid(True)
        plt.axis("equal")

    if startx == None:
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        pathx, pathy = a_star.planning(xs, ys, xt, yt, show, ox, oy)
    else:
        # plt.plot(vmx, vmy, ".k")
        # plt.show()
        a_star = AStarPlanner(vmx, vmy, grid_size, robot_radius)
        pathx, pathy = a_star.planning(startx, starty, xt, yt, show, vmx, vmy)

    a, b = diminuir_pontos(pathx, pathy, p.xobs, p.yobs, apf=True)

    ########## 3D ########## CENARIO 1 ##########
    # x3D, y3D, z3D = pseudo3D(a, b, vmx, vmy, p.visXz, p.visYz, 2)
    # pontosComCurva3D = generate_curve(x3D, y3D, z3D)

    # ax = plt.axes(projection = "3d")
    
    # ax.plot(a, b, 'g-') 
    # ax.plot(p.visX, p.visY, ".y")

    # positions = [(10,8,0), (25,20,0), (40,40,0), (35,0,0), (25,20,0), (30,30,0)]
    # sizes = [(1,9,4), (1,24,4), (1,10,2), (1,30,1), (10,1,4), (10,1,4)]
    # colors = ["yellow"] * len(positions)
    # pc = draw_bar(positions,sizes,colors=colors, edgecolor="k")
    # ax.add_collection3d(pc) 

    # ax.plot3D(x3D, y3D, z3D, 'r-') 
    # ax.plot3D(pontosComCurva3D[:][0], pontosComCurva3D[:][1], pontosComCurva3D[:][2], 'b-') 

    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    # ax.set_zlabel("Z")
    # ax.set_xlim(0,50)
    # ax.set_ylim(0,50)
    # ax.set_zlim(0,5)

    # plt.show()

    ########## CENARIO 2 ##########
    # ax = plt.axes(projection = "3d")

    # ax.plot(p.visX, p.visY, ".y")

    # positions = [(25,47,0), (25,30,0), (55,27,0), (55,18,0), (50,50,0), (80,75,0), (35,15,0), (25,30,0), (35,30,0), (55,18,0), (65,18,0), (15,5,0), (40,60,0), (70,65,0), (80,30,0), (18,70,0)]
    # sizes = [(10,1,4), (10,1,4), (10,1,4), (10,1,4), (13,1,4), (10,1,2), (13,1,4),(1,17,4), (1,17,4), (1,9,4), (1,9,4), (1,9,4), (1,20,2), (1,21,2), (1,10,4), (1,10,4)]
    # colors = ["yellow"] * len(positions)
    # pc = draw_bar(positions,sizes,colors=colors, edgecolor="k")
    # ax.add_collection3d(pc) 

    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    # ax.set_zlabel("Z")
    # ax.set_xlim(0,100)
    # ax.set_ylim(0,100)
    # ax.set_zlim(0,5)

    # plt.show()

    ########## END ##########

    curv = bSpline.B_spline(a, b)
    # if abs(startx - xs) <= p.xs and abs(starty - ys) <= p.ys:
    #     curv = bSpline.B_spline(a, b)
    # else:
    #     curv = bSpline.B_spline(a[:-1], b[:-1])
        
    xnew, ynew = curv.get_curv()
    xnew = np.concatenate(([xt], xnew), axis=0).tolist()
    ynew = np.concatenate(([yt], ynew), axis=0).tolist()

    end = time.time()-start
    if show:
        plt.plot(vmx, vmy, ".k")
        plt.plot(xs, ys, "og")
        plt.plot(xt, yt, "og")
        plt.plot(pathx, pathy, "-r")
        plt.show()

    if show:  
        plt.plot(vmx, vmy, ".k")
        plt.plot(pathx, pathy, "-r")
        plt.show()

    distancia = distancia_rota(xnew, ynew)
    
    return distancia, end, xnew, ynew
    # return distancia, end, pontosComCurva3D[:][0], pontosComCurva3D[:][1], pontosComCurva3D[:][2]

if __name__ == '__main__':
    run()