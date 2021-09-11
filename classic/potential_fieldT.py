# -*- coding: utf-8 -*-
from classic.vetor2D import APF, Vector2d
import matplotlib.pyplot as plt
import time
import math
import random
from helper.utils import distancia_rota, diminuir_pontos
from helper.ambiente import Pontos
from curves import bSpline

def check_vec_angle(v1, v2):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
    return angle


class APF_Improved(APF):
    def __init__(self, start: (), goal: (), obstacles: [], obsx: [], obsy: [], k_att: float, k_rep: float, rr: float,
                 step_size: float, max_iters: int, goal_threshold: float, show=False):
        self.start = Vector2d(start[0], start[1])
        self.current_pos = Vector2d(start[0], start[1])
        self.goal = Vector2d(goal[0], goal[1])
        self.obstacles = [Vector2d(OB[0], OB[1]) for OB in obstacles]
        self.k_att = k_att
        self.k_rep = k_rep
        self.rr = rr
        self.step_size = step_size
        self.max_iters = max_iters
        self.iters = 0
        self.goal_threashold = goal_threshold
        self.path = list()
        self.is_path_plan_success = False
        self.is_plot = show
        self.delta_t = 0.01
        self.obsx = obsx
        self.obsy = obsy

    def repulsion(self):
        rep = Vector2d(0, 0)  
        for obstacle in self.obstacles:
            obs_to_rob = (self.current_pos - obstacle)
            rob_to_goal = (self.goal - self.current_pos)
            if (obs_to_rob.length > self.rr):  
                pass
            else:
                rep_1 = Vector2d(obs_to_rob.direction[0], obs_to_rob.direction[1]) * self.k_rep * (
                        1.0 / obs_to_rob.length - 1.0 / self.rr) / (obs_to_rob.length ** 2) * (rob_to_goal.length ** 2)
                rep_2 = Vector2d(rob_to_goal.direction[0], rob_to_goal.direction[1]) * self.k_rep * ((1.0 / obs_to_rob.length - 1.0 / self.rr) ** 2) * rob_to_goal.length
                rep +=(rep_1+rep_2)
        return rep

def run(show=False, vmx=None, vmy=None, startx=None, starty=None, p1=None):
    p = Pontos()
    k_att, k_rep = 1.0, 0.8
    rr = 4
    step_size, max_iters, goal_threashold = .2, 1000, .2 
    step_size_ = 1

    start, goal = (int(p.xs), int(p.ys)), (int(p.xt), int(p.yt))
    
    if startx == None:
        ox, oy = p.xobs, p.yobs
        obs = []
        for x, y in zip(ox, oy):
            obs.append([x, y])
    else: 
        ox, oy = vmx, vmy
        obs = []
        for x, y in zip(ox, oy):
            obs.append([x, y])

    if show:
        fig = plt.figure(figsize=(7, 7))
        subplot = fig.add_subplot(111)
        subplot.plot(ox, oy, ".k")
        subplot.plot(start[0], start[1], '*r')
        subplot.plot(goal[0], goal[1], '*r')
    
    starti = time.time()
    if startx == None:
        apf = APF_Improved(start, goal, obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
        apf.path_plan()
    else:
        apf = APF_Improved((startx, starty), goal, obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
        apf.path_plan()

    if apf.is_path_plan_success:
        path = apf.path
        path_ = path
        # print(path)
        # path_ = []
        # i = int(step_size_ / step_size)
        # while (i < len(path)):
        #     path_.append(path[i])
        #     i += int(step_size_ / step_size)
        # if path_[-1].all() != path[-1].all():  
        #     path_.append(path[-1])
        
        # # print('planed path points:{}'.format(path_))
        print('path plan success')
        # if show:
        #     plt.plot(ox, oy, ".k")
        #     px, py = [K[0] for K in path_], [K[1] for K in path_]  
        #     subplot.plot(px, py, '^k')
        #     plt.show()
    else:
        path = apf.path
        path_ = []
        i = int(step_size_ / step_size)
        while (i < len(path)):
            path_.append(path[i])
            i += int(step_size_ / step_size)

        if path_[-1] != path[-1]:  
            path_.append(path[-1])
        
        print('path plan failed')

    distancia, px, py = distancia_rota(path_)


    # a, b = diminuir_pontos(px, py, p.xobs, p.yobs)

    # curv = bSpline.B_spline(a, b)
    # xnew, ynew = curv.get_curv()

    end = time.time() - starti

    # distancia2 = distancia_rota(xnew, ynew)

    return distancia, end, px, py

if __name__ == '__main__':
    run(show=True)