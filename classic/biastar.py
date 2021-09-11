# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import numpy as np
from classic import a_star as ajudante
from helper.utils import dist_euclidiana, distancia_rota, diminuir_pontos, simulate_points
from helper.ambiente import Pontos
from curves import bSpline
import time

class AStarPlanner:
    def __init__(self, ox, oy, resolution, rr):
        self.min_x, self.min_y = None, None
        self.max_x, self.max_y = None, None
        self.x_width, self.y_width, self.obstacle_map = None, None, None
        self.resolution = resolution
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index no mapa
            self.y = y  # index no mapa
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy, show, vmx, vmy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()
        open_set_A[self.calc_grid_index(start_node)] = start_node
        open_set_B[self.calc_grid_index(goal_node)] = goal_node

        current_A = start_node
        current_B = goal_node
        meet_point_A, meet_point_B = None, None

        while 1:
            if len(open_set_A) == 0 or len(open_set_B) == 0:
                _, _, rotax, rotay = ajudante.run(show=False, vmx=vmx, vmy=vmy, startx=sx, starty=sy)
                return rotax, rotay
                
            c_id_A = min(
                open_set_A,
                key=lambda o: self.find_total_cost(open_set_A, o, current_B))

            current_A = open_set_A[c_id_A]

            c_id_B = min(
                open_set_B,
                key=lambda o: self.find_total_cost(open_set_B, o, current_A))

            current_B = open_set_B[c_id_B]

            if show:  
                plt.plot(self.calc_grid_position(current_A.x, self.min_x),
                         self.calc_grid_position(current_A.y, self.min_y),
                         "xc")
                plt.plot(self.calc_grid_position(current_B.x, self.min_x),
                         self.calc_grid_position(current_B.y, self.min_y),
                         "xc")
                # cancelar com o esc
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set_A.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current_A.x == current_B.x and current_A.y == current_B.y:
                print("Found goal")
                meet_point_A = current_A
                meet_point_B = current_B
                break

            # remover do conjunto aberto
            del open_set_A[c_id_A]
            del open_set_B[c_id_B]

            # adicionar ao conjunto fechado
            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            # busca expand_grid  no mapa baseada no modelo de movimento
            for i, _ in enumerate(self.motion):
                c_nodes = [self.Node(current_A.x + self.motion[i][0],
                                     current_A.y + self.motion[i][1],
                                     current_A.cost + self.motion[i][2],
                                     c_id_A),
                           self.Node(current_B.x + self.motion[i][0],
                                     current_B.y + self.motion[i][1],
                                     current_B.cost + self.motion[i][2],
                                     c_id_B)]

                n_ids = [self.calc_grid_index(c_nodes[0]),
                         self.calc_grid_index(c_nodes[1])]

                # se o no nao for seguro, n faca nada
                continue_ = self.check_nodes_and_sets(c_nodes, closed_set_A,
                                                      closed_set_B, n_ids)

                if not continue_[0]:
                    if n_ids[0] not in open_set_A:
                        # novo no
                        open_set_A[n_ids[0]] = c_nodes[0]
                    else:
                        if open_set_A[n_ids[0]].cost > c_nodes[0].cost:
                            # esse caminho eh melhor ate agora
                            open_set_A[n_ids[0]] = c_nodes[0]

                if not continue_[1]:
                    if n_ids[1] not in open_set_B:
                        # novo no
                        open_set_B[n_ids[1]] = c_nodes[1]
                    else:
                        if open_set_B[n_ids[1]].cost > c_nodes[1].cost:
                            # esse caminho eh melhor ate agora
                            open_set_B[n_ids[1]] = c_nodes[1]

        rx, ry = self.calc_final_bidirectional_path(
            meet_point_A, meet_point_B, closed_set_A, closed_set_B)

        return rx, ry

    # pegar dois conjuntos e dois nos proxims para retornar um caminho otimo
    def calc_final_bidirectional_path(self, n1, n2, setA, setB):
        rx_A, ry_A = self.calc_final_path(n1, setA)
        rx_B, ry_B = self.calc_final_path(n2, setB)

        rx_A.reverse()
        ry_A.reverse()

        rx = rx_A + rx_B
        ry = ry_A + ry_B

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # fazer o caminho final
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def check_nodes_and_sets(self, c_nodes, closedSet_A, closedSet_B, n_ids):
        continue_ = [False, False]
        if not self.verify_node(c_nodes[0]) or n_ids[0] in closedSet_A:
            continue_[0] = True

        if not self.verify_node(c_nodes[1]) or n_ids[1] in closedSet_B:
            continue_[1] = True

        return continue_

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0 
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def find_total_cost(self, open_set, lambda_, n1):
        g_cost = open_set[lambda_].cost
        h_cost = self.calc_heuristic(n1, open_set[lambda_])
        f_cost = g_cost + h_cost
        return f_cost

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[int(node.x)][int(node.y)]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_x)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_x)
        # print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        # print("x_width:", self.x_width)
        # print("y_width:", self.y_width)

        # gerar mapa de obstaculos
        self.obstacle_map = [[False for _ in range(int(self.y_width))]
                             for _ in range(int(self.x_width))]
        for ix in range(int(self.x_width)):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(int(self.y_width)):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
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

def run(show=False, vmx=None, vmy=None, startx=None, starty=None, p1=None):
    start = time.time()
    pathx = []
    pathy = []

    p = Pontos()
    xs = p.xs
    ys = p.ys
    xt = p.xt
    yt = p.yt
    grid_size = 3.0  # [m]
    robot_radius = 2.0  # [m]

    # print(str(xt) + " - " + str(yt))

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

    # print(a)
    # print(b)

    # curv = bSpline.B_spline(a, b)
    if abs(startx - xs) <= p.xs and abs(starty - ys) <= p.ys:
        curv = bSpline.B_spline(a, b)
    else:
        curv = bSpline.B_spline(a[:-1], b[:-1])
        
    xnew, ynew = curv.get_curv() # as vezes ele tira o ultimo valor
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

    del xnew[0]
    del ynew[0]

    return distancia, end, xnew[::-1], ynew[::-1]

if __name__ == '__main__':
    run()