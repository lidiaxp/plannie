"""
https://en.wikipedia.org/wiki/Bidirectional_search
based on http://python.algorithmexamples.com/web/graphs/bidirectional_a_star.html
"""
 
import time
from math import sqrt
from typing import List, Tuple
from helper.ambiente import Pontos
from helper.utils import *

import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D,art3d
 
# 1 for manhattan, 0 for euclidean
HEURISTIC = 0
 
grid = [
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
    [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]],
]



# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.plot3D(p.xobs, p.yobs, p.zobs, ".k")
# plt.show()
 
delta = [[1,0,0],[0,1,0],[0,0,1],[-1,0,0],[0,-1,0],[0,0,-1],[1,1,0],[1,-1,0],[1,1,1],[1,1,-1],[1,-1,1],[1,-1,-1],[-1,1,0],[-1,-1,0],[-1,1,1],[-1,1,-1],[-1,-1,1],[-1,-1,-1]]

delta = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0], [0, -1, 0], [0, 0, -1]]  # up, left, down, right
delta = [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]]  # up, left, down, right
 


class Node:
    """
    >>> k = Node(0, 0, 4, 3, 0, None)
    >>> k.calculate_heuristic()
    5.0
    >>> n = Node(1, 4, 3, 4, 2, None)
    >>> n.calculate_heuristic()
    2.0
    >>> l = [k, n]
    >>> n == l[0]
    False
    >>> l.sort()
    >>> n == l[0]
    True
    """
 
    def __init__(self, pos_x, pos_y, pos_z, goal_x, goal_y, goal_z, g_cost, parent):
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.pos = (pos_x, pos_y, pos_z)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_z = goal_z
        self.g_cost = g_cost
        self.parent = parent
        self.h_cost = self.calculate_heuristic()
        self.f_cost = self.g_cost + self.h_cost
 
    def calculate_heuristic(self):
        """
        Heuristic for the A*
        """
        dy = self.pos_x - self.goal_x
        dx = self.pos_y - self.goal_y
        # dz = self.pos_z - self.goal_z
        if HEURISTIC == 1:
            return abs(dx) + abs(dy) #+ abs(dz)
        else:
            return sqrt(dy ** 2 + dx ** 2)# + dz ** 2)
 
    def __lt__(self, other):
        return self.f_cost < other.f_cost
 
 
class AStar():
    """
    >>> astar = AStar((0, 0), (len(grid) - 1, len(grid[0]) - 1))
    >>> (astar.start.pos_y + delta[3][0], astar.start.pos_x + delta[3][1])
    (0, 1)
    >>> [x.pos for x in astar.get_successors(astar.start)]
    [(1, 0), (0, 1)]
    >>> (astar.start.pos_y + delta[2][0], astar.start.pos_x + delta[2][1])
    (1, 0)
    >>> astar.retrace_path(astar.start)
    [(0, 0)]
    >>> astar.search()  # doctest: +NORMALIZE_WHITESPACE
    [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3), (3, 3),
     (4, 3), (4, 4), (5, 4), (5, 5), (6, 5), (6, 6)]
    """
 
    def __init__(self, start, goal):
        self.start = Node(start[0], start[1], 0, goal[0], goal[1], 0, 0, None)
        self.target = Node(goal[0], goal[1], 0, goal[0], goal[1], 0, 99999, None)
 
        self.open_nodes = [self.start]
        self.closed_nodes = []
 
        self.reached = False
 
    def search(self,obs,limiar):
        while self.open_nodes:
            # Open Nodes are sorted using __lt__
            self.open_nodes.sort()
            current_node = self.open_nodes.pop(0)
 
            if current_node.pos == self.target.pos:
                self.reached = True
                return self.retrace_path(current_node)
 
            self.closed_nodes.append(current_node)
            successors = self.get_successors(current_node,obs,limiar)
 
            for child_node in successors:
                if child_node in self.closed_nodes:
                    continue
 
                if child_node not in self.open_nodes:
                    self.open_nodes.append(child_node)
                else:
                    # retrieve the best current path
                    better_node = self.open_nodes.pop(self.open_nodes.index(child_node))
 
                    if child_node.g_cost < better_node.g_cost:
                        self.open_nodes.append(child_node)
                    else:
                        self.open_nodes.append(better_node)
 
        if not (self.reached):
            return [(self.start.pos)]
 
    def get_successors(self, parent, obs, limiar):
        """
        Returns a list of successors (both in the grid and free spaces)
        """
        successors = []
        for action in delta:
            pos_x = parent.pos_x + action[0]
            pos_y = parent.pos_y + action[1]
            pos_z = parent.pos_z + action[2]
            if not (0 <= pos_x <= limiar[0] and 0 <= pos_y <= limiar[1]):# and 0 <= pos_z <= 5):
                continue
 
            # v = 0
            # for i in range(len(obs)):
            #     if dist_euclidiana3D(obs[i][0],obs[i][1],obs[i][2],pos_x, pos_y, pos_z) < 1.5:
            #         v = 1
            #         break
            
            # if v == 1:
            #     continue

            if [pos_x, pos_y] in obs:
                continue
 
            successors.append(
                Node(
                    pos_x,
                    pos_y,
                    pos_z,
                    self.target.pos_x,
                    self.target.pos_y,
                    self.target.pos_z,
                    parent.g_cost + 1,
                    parent,
                )
            )
        return successors
 
    def retrace_path(self, node):
        """
        Retrace the path from parents to parents until start node
        """
        current_node = node
        path = []
        while current_node is not None:
            path.append((current_node.pos_x, current_node.pos_y, current_node.pos_z))
            current_node = current_node.parent
        path.reverse()
        return path
 
 
class BidirectionalAStar:
    """
    >>> bd_astar = BidirectionalAStar((0, 0), (len(grid) - 1, len(grid[0]) - 1))
    >>> bd_astar.fwd_astar.start.pos == bd_astar.bwd_astar.target.pos
    True
    >>> bd_astar.retrace_bidirectional_path(bd_astar.fwd_astar.start,
    ...                                     bd_astar.bwd_astar.start)
    [(0, 0)]
    >>> bd_astar.search()  # doctest: +NORMALIZE_WHITESPACE
    [(0, 0), (0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (2, 4),
     (2, 5), (3, 5), (4, 5), (5, 5), (5, 6), (6, 6)]
    """
 
    def __init__(self, start, goal):
        self.fwd_astar = AStar(start, goal)
        self.bwd_astar = AStar(goal, start)
        self.reached = False
 
    def search(self,obs,limiar):
        while self.fwd_astar.open_nodes or self.bwd_astar.open_nodes:
            self.fwd_astar.open_nodes.sort()
            self.bwd_astar.open_nodes.sort()
            current_fwd_node = self.fwd_astar.open_nodes.pop(0)
            current_bwd_node = self.bwd_astar.open_nodes.pop(0)
 
            if current_bwd_node.pos == current_fwd_node.pos:
                self.reached = True
                return self.retrace_bidirectional_path(
                    current_fwd_node, current_bwd_node
                )
 
            self.fwd_astar.closed_nodes.append(current_fwd_node)
            self.bwd_astar.closed_nodes.append(current_bwd_node)
 
            self.fwd_astar.target = current_bwd_node
            self.bwd_astar.target = current_fwd_node
 
            successors = {
                self.fwd_astar: self.fwd_astar.get_successors(current_fwd_node,obs,limiar),
                self.bwd_astar: self.bwd_astar.get_successors(current_bwd_node,obs,limiar),
            }
 
            for astar in [self.fwd_astar, self.bwd_astar]:
                for child_node in successors[astar]:
                    if child_node in astar.closed_nodes:
                        continue
 
                    if child_node not in astar.open_nodes:
                        astar.open_nodes.append(child_node)
                    else:
                        # retrieve the best current path
                        better_node = astar.open_nodes.pop(
                            astar.open_nodes.index(child_node)
                        )
 
                        if child_node.g_cost < better_node.g_cost:
                            astar.open_nodes.append(child_node)
                        else:
                            astar.open_nodes.append(better_node)
 
        if not self.reached:
            return [self.fwd_astar.start.pos]
 
    def retrace_bidirectional_path(self, fwd_node, bwd_node):
        fwd_path = self.fwd_astar.retrace_path(fwd_node)
        bwd_path = self.bwd_astar.retrace_path(bwd_node)
        bwd_path.pop()
        bwd_path.reverse()
        path = fwd_path + bwd_path
        return path
 
def run(obsx=[None], obsy=None, obsz=None, start=None, end=None):
    # all coordinates are given in format [y,x]
    import doctest
    p = Pontos()
    limiar = p.limiar
    obs = []

    doctest.testmod()
    if obsx[0] == None:
        init = (p.xs, p.ys)#, p.zs)
        goal = (p.xt,p.yt)#,p.zt)

        for i in range(len(p.xobs)):
            obs.append([p.xobs[i],p.yobs[i],p.zobs[i]])
    else:
        init = start
        goal = end

        for i in range(len(obsx)):
            obs.append([obsx[i],obsy[i],obsz[i]])
        
    # for elem in grid:
    #     print(elem)
 
    # start_time = time.time()
    # a_star = AStar(init, goal)
    # path = a_star.search()
    # end_time = time.time() - start_time
    # print("AStar execution time = " + str(end_time) + " seconds")
 
    bd_start_time = time.time()
    bidir_astar = BidirectionalAStar(init, goal)
    path = bidir_astar.search(obs,limiar)
    x, y, z = [], [], []
    for i in range(len(path)):
        x.append(path[i][0])
        y.append(path[i][1])
        # z.append(path[i][2])
    
    z = [1] * len(x)

    plt.plot(x, y)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(x,y,z)
    ax.plot3D(p.xobs, p.yobs, p.zobs, ".k")
    plt.show()
    bd_end_time = time.time() - bd_start_time
    print("BidirectionalAStar execution time = " + str(bd_end_time) + " seconds")

    a, b, c = x, y, z
    # a, b, c = diminuir_pontos3D(x, y, z, p.xobs, p.yobs, p.zobs, apf=False)
    a, b, c = generate_curve(a, b, c)
    distancia = distancia_rota3D(a, b, c)

    return distancia, bd_end_time, a, b, c

if __name__ == "__main__":
    run()










