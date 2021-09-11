# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import numpy as np
from classic import probabilistic_roadmap as ajudante
from helper.utils import dist_euclidiana3D, distancia_rota3D, diminuir_pontos3D, simulate_points, pseudo3D, draw_bar, draw_cylinder
from helper.ambiente import Pontos
from curves import bSpline
from curves.spline3D import generate_curve
import time

ax = plt.axes(projection = "3d")

class AStarPlanner:
    def __init__(self, ox, oy, oz, reso, rr):
        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy, oz)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, z, cost, pind):
            self.x = x  # index 
            self.y = y  # index
            self.z = z  # index
            self.cost = cost
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.z) + "," + str(self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, sz, gx, gy, gz, show, vmx, vmy, vmz):
        nstart = self.Node(self.calc_xyzindex(sx, self.minx),
                           self.calc_xyzindex(sy, self.miny),
                           self.calc_xyzindex(sy, self.minz), 0.0, -1)
        ngoal = self.Node(self.calc_xyzindex(gx, self.minx),
                          self.calc_xyzindex(gy, self.miny),
                          self.calc_xyzindex(gz, self.minz), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            # if len(open_set) == 0:
            #     # print("Open set is empty..")
            #     _, _, rotax, rotay, rotaz = ajudante.run(show=False, vmx=vmx, vmy=vmy,vmz=vmz, startx=sx, starty=sy, startz=sz)
            #     return rotax, rotay, rotaz
            #     # break

            c_id = min(
                open_set, key=lambda o: open_set[o].cost + dist_euclidiana3D(ngoal.x, ngoal.y, ngoal.z, open_set[o].x, open_set[o].y, open_set[o].z))
            current = open_set[c_id]

            if show:  
                ax.plot3D(self.calc_grid_position(current.x, self.minx),
                         self.calc_grid_position(current.y, self.miny),
                         self.calc_grid_position(current.z, self.minz), "xc")
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == ngoal.x and current.y == ngoal.y and current.z == ngoal.z:
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
                                 current.z + self.motion[i][2],
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

        rx, ry, rz = self.calc_final_path(ngoal, closed_set)

        return rx, ry, rz

    def calc_final_path(self, ngoal, closedset):
        # fazer caminho final
        rx, ry, rz = [self.calc_grid_position(ngoal.x, self.minx)], [self.calc_grid_position(ngoal.y, self.miny)], [self.calc_grid_position(ngoal.z, self.minz)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            rz.append(self.calc_grid_position(n.z, self.minz))
            pind = n.pind

        return rx, ry, rz

    def calc_grid_position(self, index, minp):
        pos = index * self.reso + minp
        return pos

    def calc_xyzindex(self, position, min_pos):
        return round((position - min_pos) / self.reso)

    def calc_grid_index(self, node):
        return (node.y - self.miny) * self.xwidth + (node.x - self.minx) * self.ywidth + (node.z - self.minz)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)
        pz = self.calc_grid_position(node.z, self.minz)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif pz < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False
        elif pz >= self.maxy:
            return False
        

        # checar colisao
        if self.obmap[int(node.x)][int(node.y)][int(node.z)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy, oz):
        self.minx = round(min(ox))
        self.miny = round(min(oy))
        self.minz = round(min(oz))
        self.maxx = round(max(ox))
        self.maxy = round(max(oy))
        self.maxz = round(max(oz))

        self.xwidth = round((self.maxx - self.minx) / self.reso)
        self.ywidth = round((self.maxy - self.miny) / self.reso)
        self.zwidth = round((self.maxz - self.minz) / self.reso)
        # gerar mapa de obstaculo
        # self.obmap = [[False for i in range(int(self.ywidth))] for i in range(int(self.xwidth))]
        self.obmap = np.zeros((int(self.zwidth),int(self.xwidth),int(self.ywidth))).tolist()
        for ix in range(int(self.xwidth)):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(int(self.ywidth)):
                y = self.calc_grid_position(iy, self.miny)
                for iz in range(int(self.zwidth)):
                    z = self.calc_grid_position(iz, self.minz)
                    for iox, ioy, ioz in zip(ox, oy, oz):
                        d = math.sqrt((iox - x) ** 2 + (ioy - y) ** 2 + (ioz - z) ** 2)
                        if d <= self.rr:
                            self.obmap[ix][iy][iz] = True
                            break

    @staticmethod
    def get_motion_model():
        # dx, dy, dz, cost
        motion = [[1, 0, 1, math.sqrt(2)],
                  [0, 1, 1, math.sqrt(2)],
                  [-1, 0, 1, math.sqrt(2)],
                  [0, -1, 1, math.sqrt(2)],
                  [-1, -1, 1, math.sqrt(3)],
                  [-1, 1, 1, math.sqrt(3)],
                  [1, -1, 1, math.sqrt(3)],
                  [1, 1, 1, math.sqrt(3)],
                  [1, 0, -1, math.sqrt(2)],
                  [0, 1, -1, math.sqrt(2)],
                  [-1, 0, -1, math.sqrt(2)],
                  [0, -1, -1, math.sqrt(2)],
                  [-1, -1, -1, math.sqrt(3)],
                  [-1, 1, -1, math.sqrt(3)],
                  [1, -1, -1, math.sqrt(3)],
                  [1, 1, -1, math.sqrt(3)],
                  [1, 0, 0, 1],
                  [0, 1, 0, 1],
                  [-1, 0, 0, 1],
                  [0, -1, 0, 1],
                  [-1, -1, 0, math.sqrt(2)],
                  [-1, 1, 0, math.sqrt(2)],
                  [1, -1, 0, math.sqrt(2)],
                  [1, 1, 0, math.sqrt(2)]]

        return motion

# --------------------------------------------------------------------------------------------------

def run(show=0, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None):
    start = time.time()
    pathx = []
    pathy = []
    pathz = []

    p = Pontos()
    xs = 1
    ys = 1
    zs = 2
    xt = 5
    yt = 5
    zt = 4
    grid_size = 1.0  # [m]
    robot_radius = 1.0  # [m]

    # ox = [1,2,3,4,5,1,2,3,4,5,1,2,3,4,5]
    # oy = [3,3,3,3,3,3,3,3,3,3,3,3,3,3,3]
    # oz = [1,1,1,1,1,2,2,2,2,2,3,3,3,3,3]
    ox = p.xobs
    oy = p.yobs
    oz = p.zobs

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
        a_star = AStarPlanner(ox, oy, oy, grid_size, robot_radius)
        pathx, pathy, pathz = a_star.planning(xs, ys, zs, xt, yt, zt, show, ox, oy, oz)
    else:
        # plt.plot(vmx, vmy, ".k")
        # plt.show()
        a_star = AStarPlanner(vmx, vmy, vmz, grid_size, robot_radius)
        pathx, pathy, pathz = a_star.planning(startx, starty, startz, xt, yt, zt, show, vmx, vmy, vmz)

    a, b, c = diminuir_pontos3D(pathx, pathy, pathz, ox, oy, oz, apf=False)
    a1, b1, c1 = generate_curve(a, b, c)

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
    # # ax.plot3D(pontosComCurva3D[:][0], pontosComCurva3D[:][1], pontosComCurva3D[:][2], 'b-') 

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

    # curv = bSpline.B_spline(a, b)
    # if abs(startx - xs) <= p.xs and abs(starty - ys) <= p.ys:
    #     curv = bSpline.B_spline(a, b)
    # else:
    #     curv = bSpline.B_spline(a[:-1], b[:-1])
        
    # xnew, ynew = curv.get_curv()
    # xnew = np.concatenate(([xt], xnew), axis=0).tolist()
    # ynew = np.concatenate(([yt], ynew), axis=0).tolist()

    end = time.time()-start
    # if show:
    #     plt.plot(vmx, vmy, ".k")
    #     plt.plot(xs, ys, "og")
    #     plt.plot(xt, yt, "og")
    #     plt.plot(pathx, pathy, "-r")
    #     plt.show()

    # if show:  
    #     plt.plot(vmx, vmy, ".k")
    #     plt.plot(pathx, pathy, "-r")
    #     plt.show()

    distancia = distancia_rota3D(a1, b1, c1)
    
    ax.plot3D(a1, b1, c1)
    plt.show()

    # return distancia, end, xnew, ynew
    return distancia, end, a1, b1, c1
    # return distancia, end, x3D.tolist(), y3D.tolist(), z3D.tolist()
    # return distancia, end, pontosComCurva3D[:][0], pontosComCurva3D[:][1], pontosComCurva3D[:][2]

if __name__ == '__main__':
    run()