# -*- coding: utf-8 -*-
from classic.biAuxApf import APF
from classic.vetor2D import Vector2d
import matplotlib.pyplot as plt
import time
import math
import random
import numpy as np
from helper.utils import distancia_rota, diminuir_pontos, simulate_points
from helper.ambiente import Pontos
from curves import bSpline

def check_vec_angle(v1, v2):
    v1_v2 = v1.deltaX * v2.deltaX + v1.deltaY * v2.deltaY
    angle = math.acos(v1_v2 / (v1.length * v2.length)) * 180 / math.pi
    return angle

class APF_Improved(APF):
    def __init__(self, start, goal, obstacles, obsx, obsy, k_att, k_rep, rr,
                 step_size, max_iters, goal_threshold, show=False):
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

def run(show=False, vmx=None, vmy=None, startx=None, starty=None, normal=False, p1=None, signal=False):
    p = Pontos()
    k_att, k_rep = 1.0, 1.8
    rr = 4
    step_size, max_iters, goal_threashold = .2, 1000, .2 
    auxgx, auxgy = p.xt, p.yt

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
    if startx == p.xs and starty == p.ys or normal:
        if startx == None:
            apf = APF_Improved(start, goal, obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
            apf.path_plan(one=True, p1=p, signal=signal)
        else:
            apf = APF_Improved((startx, starty), goal, obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
            apf.path_plan(one=True, p1=p, signal=signal)
    else:
        if startx == None:
            apf = APF_Improved(goal, start, obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
            apf.path_plan(one=True, p1=p, signal=signal)
        else:
            apf = APF_Improved(goal, (startx, starty), obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
            apf.path_plan(one=True, p1=p, signal=signal)
    # print("one")
    if apf.is_path_plan_success:
        path = apf.path
        path_AUX = path
        # if startx != p.xs and starty != p.ys:
        #     try:
        #         path_AUX.reverse()
        #     except:
        #         path_AUX = np.flip(path_AUX, 0)
        auxgx = apf.newgx
        auxgy = apf.newgy
    
        
    # ------------------------------------------------
    # print("two")
    if not signal:
        if startx == p.xs and starty == p.ys or normal:
            if startx == None:
                apf = APF_Improved(goal, (auxgx, auxgy), obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
                apf.path_plan(segundaVez=True, p1=p1, signal=signal)
            else:
                apf = APF_Improved(goal, (auxgx, auxgy), obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
                apf.path_plan(segundaVez=True, p1=p1, signal=signal)
        else:
            if startx == None:
                apf = APF_Improved(start, (auxgx, auxgy), obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
                apf.path_plan(segundaVez=True, p1=p1, signal=signal)
            else:
                apf = APF_Improved((startx, starty), (auxgx, auxgy), obs, ox, oy, k_att, k_rep, rr, step_size, max_iters, goal_threashold, show)
                apf.path_plan(segundaVez=True, p1=p1, signal=signal)

        if apf.is_path_plan_success:
            path = apf.path
            path_ = path
            # print(path)
            if startx != p.xs and starty != p.ys:
                path_ = np.concatenate((path_AUX, path_), axis=0)
            else:
                path_ = np.concatenate((path_AUX, np.flip(path_, 0)), axis=0)
        #     print('path plan success')
        # else:
        #     print('path plan failed')



    if len(apf.px) == 0:
        distancia, px, py = distancia_rota(path_)

        px, py = diminuir_pontos(px, py, ox, oy, apf=True)
        # px.reverse()
        # py.reverse()
        
        if startx==None: startx, starty = p.xs, p.ys
        curv = bSpline.B_spline(px, py) #if abs(startx - p.xs) < 5 and abs(starty - p.ys) < 5 else bSpline.B_spline(px[:-1], py[:-1]) 
        xnew, ynew = curv.get_curv()

        # del xnew[-1]
        # del ynew[-1]
        if startx != p.xs and starty != p.ys:
            # xx, yy = simulate_points(xnew[-1], p.xt, ynew[-1], p.yt)
            xnew = np.concatenate(([p.xt], xnew), axis=0).tolist()
            ynew = np.concatenate(([p.yt], ynew), axis=0).tolist()

        # plt.plot(xnew, ynew, "-r")
        # plt.show()
    else:
        xnew, ynew = apf.px, apf.py
        distancia = distancia_rota(xnew, ynew)

    # print(xnew)

    # print(xnew)

    end = time.time() - starti
    
    return distancia, end, xnew, ynew

if __name__ == '__main__':
    run(show=True)