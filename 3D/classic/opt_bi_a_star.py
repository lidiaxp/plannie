"""
A* algorithm
Author: Weicent
randomly generate obstacles, start and goal point
searching path from start and end simultaneously
"""

import numpy as np
import matplotlib.pyplot as plt
import math
import time
from helper.ambiente import Pontos
from helper.utils import distancia_rota3D, diminuir_pontos3D, tirarRepertido3D
from curves.spline3D import generate_curve


class Node:
    def __init__(self, G=0, H=0, coordinate=None, parent=None):
        self.G = G
        self.H = H
        self.F = G + H
        self.parent = parent
        self.coordinate = coordinate

    def reset_f(self):
        self.F = self.G + self.H


def hcost(node_coordinate, goal, dimension_3):
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    if dimension_3:
        dz = abs(node_coordinate[2] - goal[2])
        hcost = dx + dy + dz
    else:
        hcost = dx + dy

    return hcost


def gcost(fixed_node, update_node_coordinate, dimension_3):
    dx = abs(fixed_node.coordinate[0] - update_node_coordinate[0])
    dy = abs(fixed_node.coordinate[1] - update_node_coordinate[1])
    if dimension_3:
        dz = abs(fixed_node.coordinate[2] - update_node_coordinate[2])
        gcost = math.hypot(dx, dy, dz)  # gc = move from fixed_node to update_node
    else:
        gcost = math.hypot(dx, dy)

    return gcost


def find_neighbor(node, ob, closed, bottom_vertex, top_vertex, dimension_3):
    ob_list = ob.tolist()
    neighbor: list = []
    if dimension_3:
        for x in range(int(node.coordinate[0]) - 1, int(node.coordinate[0]) + 2):
            for y in range(int(node.coordinate[1]) - 1, int(node.coordinate[1]) + 2):
                for z in range(int(node.coordinate[2]) - 1, int(node.coordinate[2]) + 2):
                    x_boundaries_ok = top_vertex[0] > x > bottom_vertex[0]
                    y_boundaries_ok = top_vertex[1] > y > bottom_vertex[1]
                    z_boundaries_ok = top_vertex[2] > z > bottom_vertex[2]
                    if [x, y, z] not in ob_list and x_boundaries_ok and y_boundaries_ok and z_boundaries_ok:
                        neighbor.append([x, y, z])
    else:
        for x in range(node.coordinate[0] - 1, node.coordinate[0] + 2):
            for y in range(node.coordinate[1] - 1, node.coordinate[1] + 2):
                if [x, y] not in ob_list:
                    neighbor.append([x, y])

    neighbor.remove(node.coordinate)

    if dimension_3:
        # top bottom left right neighbors of node
        top_nei = [node.coordinate[0], node.coordinate[1] + 1, node.coordinate[2]]
        bottom_nei = [node.coordinate[0], node.coordinate[1] - 1, node.coordinate[2]]
        left_nei = [node.coordinate[0] - 1, node.coordinate[1], node.coordinate[2]]
        right_nei = [node.coordinate[0] + 1, node.coordinate[1], node.coordinate[2]]
        # neighbors in four vertex
        lt_nei = [node.coordinate[0] - 1, node.coordinate[1] + 1, node.coordinate[2]]
        rt_nei = [node.coordinate[0] + 1, node.coordinate[1] + 1, node.coordinate[2]]
        lb_nei = [node.coordinate[0] - 1, node.coordinate[1] - 1, node.coordinate[2]]
        rb_nei = [node.coordinate[0] + 1, node.coordinate[1] - 1, node.coordinate[2]]

        # Up
        top_nei_up = [node.coordinate[0], node.coordinate[1] + 1, node.coordinate[2] + 1]
        bottom_nei_up = [node.coordinate[0], node.coordinate[1] - 1, node.coordinate[2] + 1]
        left_nei_up = [node.coordinate[0] - 1, node.coordinate[1], node.coordinate[2] + 1]
        right_nei_up = [node.coordinate[0] + 1, node.coordinate[1], node.coordinate[2] + 1]
        lt_nei_up = [node.coordinate[0] - 1, node.coordinate[1] + 1, node.coordinate[2] + 1]
        rt_nei_up = [node.coordinate[0] + 1, node.coordinate[1] + 1, node.coordinate[2] + 1]
        lb_nei_up = [node.coordinate[0] - 1, node.coordinate[1] - 1, node.coordinate[2] + 1]
        rb_nei_up = [node.coordinate[0] + 1, node.coordinate[1] - 1, node.coordinate[2] + 1]

        # Down
        top_nei_down = [node.coordinate[0], node.coordinate[1] + 1, node.coordinate[2] - 1]
        bottom_nei_down = [node.coordinate[0], node.coordinate[1] - 1, node.coordinate[2] - 1]
        left_nei_down = [node.coordinate[0] - 1, node.coordinate[1], node.coordinate[2] - 1]
        right_nei_down = [node.coordinate[0] + 1, node.coordinate[1], node.coordinate[2] - 1]
        lt_nei_down = [node.coordinate[0] - 1, node.coordinate[1] + 1, node.coordinate[2] - 1]
        rt_nei_down = [node.coordinate[0] + 1, node.coordinate[1] + 1, node.coordinate[2] - 1]
        lb_nei_down = [node.coordinate[0] - 1, node.coordinate[1] - 1, node.coordinate[2] - 1]
        rb_nei_down = [node.coordinate[0] + 1, node.coordinate[1] - 1, node.coordinate[2] - 1]
    else:
        # top bottom left right neighbors of node
        top_nei = [node.coordinate[0], node.coordinate[1] + 1]
        bottom_nei = [node.coordinate[0], node.coordinate[1] - 1]
        left_nei = [node.coordinate[0] - 1, node.coordinate[1]]
        right_nei = [node.coordinate[0] + 1, node.coordinate[1]]
        # neighbors in four vertex
        lt_nei = [node.coordinate[0] - 1, node.coordinate[1] + 1]
        rt_nei = [node.coordinate[0] + 1, node.coordinate[1] + 1]
        lb_nei = [node.coordinate[0] - 1, node.coordinate[1] - 1]
        rb_nei = [node.coordinate[0] + 1, node.coordinate[1] - 1]

    # remove the unnecessary neighbors
    if top_nei and left_nei in ob_list and lt_nei in neighbor:
        neighbor.remove(lt_nei)
    if top_nei and right_nei in ob_list and rt_nei in neighbor:
        neighbor.remove(rt_nei)
    if bottom_nei and left_nei in ob_list and lb_nei in neighbor:
        neighbor.remove(lb_nei)
    if bottom_nei and right_nei in ob_list and rb_nei in neighbor:
        neighbor.remove(rb_nei)

    if dimension_3:
        if top_nei_up and left_nei_up in ob_list and lt_nei_up in neighbor:
            neighbor.remove(lt_nei_up)
        if top_nei_up and right_nei_up in ob_list and rt_nei_up in neighbor:
            neighbor.remove(rt_nei_up)
        if bottom_nei_up and left_nei_up in ob_list and lb_nei_up in neighbor:
            neighbor.remove(lb_nei_up)
        if bottom_nei_up and right_nei_up in ob_list and rb_nei_up in neighbor:
            neighbor.remove(rb_nei_up)

        if top_nei_down and left_nei_down in ob_list and lt_nei_down in neighbor:
            neighbor.remove(lt_nei_down)
        if top_nei_down and right_nei_down in ob_list and rt_nei_down in neighbor:
            neighbor.remove(rt_nei_down)
        if bottom_nei_down and left_nei_down in ob_list and lb_nei_down in neighbor:
            neighbor.remove(lb_nei_down)
        if bottom_nei_down and right_nei_down in ob_list and rb_nei_down in neighbor:
            neighbor.remove(rb_nei_down)

    neighbor = [x for x in neighbor if x not in closed]
    return neighbor


def find_node_index(coordinate, node_list):
    # find node index in the node list via its coordinate
    ind = 0
    for node in node_list:
        if node.coordinate == coordinate:
            target_node = node
            ind = node_list.index(target_node)
            break
    return ind


def find_path(open_list, closed_list, goal, obstacle, bottom_vertex, top_vertex, dimension_3):
    flag = len(open_list)
    for i in range(flag):
        node = open_list[0]
        open_coordinate_list = [node.coordinate for node in open_list]
        closed_coordinate_list = [node.coordinate for node in closed_list]
        temp = find_neighbor(node, obstacle, closed_coordinate_list, bottom_vertex, top_vertex, dimension_3)
        for element in temp:
            if element in closed_list:
                continue
            elif element in open_coordinate_list:
                # if node in open list, update g value
                ind = open_coordinate_list.index(element)
                new_g = gcost(node, element, dimension_3)
                if new_g <= open_list[ind].G:
                    open_list[ind].G = new_g
                    open_list[ind].reset_f()
                    open_list[ind].parent = node
            else:  # new coordinate, create corresponding node
                ele_node = Node(coordinate=element, parent=node,
                                G=gcost(node, element, dimension_3), H=hcost(element, goal, dimension_3))
                open_list.append(ele_node)
        open_list.remove(node)
        closed_list.append(node)
        open_list.sort(key=lambda x: x.F)
    return open_list, closed_list


def node_to_coordinate(node_list):
    # convert node list into coordinate list and array
    coordinate_list = [node.coordinate for node in node_list]
    return coordinate_list


def check_node_coincide(close_ls1, closed_ls2):
    cl1 = node_to_coordinate(close_ls1)
    cl2 = node_to_coordinate(closed_ls2)
    intersect_ls = [node for node in cl1 if node in cl2]
    return intersect_ls


def get_path(org_list, goal_list, coordinate):
    path_org: list = []
    path_goal: list = []
    ind = find_node_index(coordinate, org_list)
    node = org_list[ind]
    while node != org_list[0]:
        path_org.append(node.coordinate)
        node = node.parent
    path_org.append(org_list[0].coordinate)
    ind = find_node_index(coordinate, goal_list)
    node = goal_list[ind]
    while node != goal_list[0]:
        path_goal.append(node.coordinate)
        node = node.parent
    path_goal.append(goal_list[0].coordinate)
    path_org.reverse()
    path = path_org + path_goal
    path = np.array(path)
    return path


def random_coordinate(bottom_vertex, top_vertex):
    coordinate = [np.random.randint(bottom_vertex[0] + 1, top_vertex[0]),
                  np.random.randint(bottom_vertex[1] + 1, top_vertex[1])]
    return coordinate


def draw(close_origin, close_goal, start, end, bound):
    if not close_goal.tolist():  
        close_goal = np.array([end])
    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy')
    plt.plot(close_goal[:, 0], close_goal[:, 1], 'og')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(end[0], end[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)


def draw_control(org_closed, goal_closed, flag, start, end, bound):
    stop_loop = 0  # stop sign for the searching
    org_closed_ls = node_to_coordinate(org_closed)
    org_array = np.array(org_closed_ls)
    goal_closed_ls = node_to_coordinate(goal_closed)
    goal_array = np.array(goal_closed_ls)
    path = None

    if flag == 0:
        node_intersect = check_node_coincide(org_closed, goal_closed)
        if node_intersect:  # a path is find
            path = get_path(org_closed, goal_closed, node_intersect[0])
            stop_loop = 1
            print('Path found!')
    elif flag == 1:  # start point blocked first
        stop_loop = 1
        print('There is no path to the goal! Start point is blocked!')
    elif flag == 2:  # end point blocked first
        stop_loop = 1
        print('There is no path to the goal! End point is blocked!')

    return stop_loop, path


def searching_control(start, end, bound, bottom_vertex, top_vertex, dimension_3):
    origin = Node(coordinate=start, H=hcost(start, end, dimension_3))
    goal = Node(coordinate=end, H=hcost(end, start, dimension_3))

    origin_open: list = [origin]
    origin_close: list = []

    goal_open = [goal]
    goal_close: list = []

    target_goal = end

    # flag = 0 (not blocked) 1 (start point blocked) 2 (end point blocked)
    flag = 0  # init flag
    path = None
    while True:
        # searching from start to end
        origin_open, origin_close = \
            find_path(origin_open, origin_close, target_goal, bound, bottom_vertex, top_vertex, dimension_3)
        if not origin_open:  # no path condition
            flag = 1  # origin node is blocked
            draw_control(origin_close, goal_close, flag, start, end, bound)
            break
        # update target for searching from end to start
        target_origin = min(origin_open, key=lambda x: x.F).coordinate

        # searching from end to start
        goal_open, goal_close = \
            find_path(goal_open, goal_close, target_origin, bound, bottom_vertex, top_vertex, dimension_3)
        if not goal_open:  # no path condition
            flag = 2  # goal is blocked
            draw_control(origin_close, goal_close, flag, start, end, bound)
            break
        # update target for searching from start to end
        target_goal = min(goal_open, key=lambda x: x.F).coordinate

        # continue searching, draw the process
        stop_sign, path = draw_control(origin_close, goal_close, flag, start,
                                       end, bound)
        if stop_sign:
            break
    return path



def run(show=False, vmx=[None], vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    start_time = time.time()
    p = Pontos()

    dimension_3 = True
    top_vertex = [60, 60, 10]  # top right vertex of boundary
    bottom_vertex = [0, 0, 0]  # bottom left vertex of boundary

    # obstacles
    if vmx[0] == None:
        Obstacles = []
        for x1, y1, z1 in zip(p.xobs, p.yobs, p.zobs):
            Obstacles.append((x1, y1, z1))
        Obstacles = np.array(Obstacles)
        start = (int(p.xs), int(p.ys), int(p.zs))  # starting location
    else:
        Obstacles = []
        for x1, y1, z1 in zip(vmx, vmy, vmz):
            Obstacles.append((x1, y1, z1))
        Obstacles = np.array(Obstacles)
        start = (int(startx), int(starty), int(startz))

    end = (int(p.xt), int(p.yt), int(p.zt))

    bound = np.asarray(Obstacles)

    start = [3, 3, 1]
    end = [30, 4, 3]
    bound = np.asarray([])

    print('bound', bound)
    print('start', start)
    print('end', end)
    path = searching_control(start, end, bound, bottom_vertex, top_vertex, dimension_3)

    x, y, z = [], [], []
    for caminho in path:
        x.append(caminho[0])
        y.append(caminho[1])
        z.append(caminho[2])

    # x,y,z = generate_curve(x,y,z, show)

    # xxx1, yyy1, zzz1 = diminuir_pontos3D(x, y, z, p.xobs, p.yobs, p.zobs, value=0.8)
    xx, yy, zz = tirarRepertido3D(x, y, z)

    distance = distancia_rota3D(xx, yy, zz)

    return distance, time.time() - start_time, xx, yy, zz


if __name__ == '__main__':
    distance, t, xx, yy, zz = run(show=False, vmx=[-1], vmy=[-1], vmz=[-1], startx=3, starty=3, startz=1)
    
    print(xx)
    print(yy)
    