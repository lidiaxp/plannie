# -*- coding: utf-8 -*-
import math
import random
from matplotlib import pyplot as plt
from helper.ambiente import Pontos
from helper.utils import colidir, simulate_points, definir_angulo, dist_euclidiana, pseudo3D, distancia_rota3D
import numpy as np
from classic.vetor2D import Vector2d
from classic.minLocalAPF import deuRuim

class APF():
    def __init__(self, start, goal, obstacles, obsx, obsy, k_att, k_rep, rr,
                 step_size, max_iters, goal_threshold, is_plot=False):
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
        self.is_plot = is_plot
        self.delta_t = 0.01
        self.obsx = obsx
        self.obsy = obsy
        self.newgx = 0
        self.newgy = 0
        self.px, self.py = [], []
        self.checkEstatico = True

    def attractive(self):
        att = (self.goal - self.current_pos) * self.k_att  
        return att

    def repulsion(self):
        rep = Vector2d(0, 0)  
        for obstacle in self.obstacles:
            self.current_pos.deltaX - obstacle.deltaX
            self.current_pos.deltaY - obstacle.deltaY
            t_vec = self.current_pos - obstacle
            if (t_vec.length > self.rr): 
                pass
            else:
                rep += Vector2d(t_vec.direction[0], t_vec.direction[1]) * self.k_rep * (1.0 / t_vec.length - 1.0 / self.rr) / (t_vec.length ** 2)  
        return rep

    def path_plan(self, one=False, segundaVez=False, p1=None, signal=False):
        valorAntecessorX, valorAntecessorY = self.start.deltaX, self.start.deltaY
        nLimiar = 0.3
        count = 0
        while (self.iters < self.max_iters and (self.current_pos - self.goal).length > self.goal_threashold):
            self.px, self.py = [], []
            f_vec = self.attractive() + self.repulsion()
            self.current_pos += Vector2d(f_vec.direction[0], f_vec.direction[1]) * self.step_size
            self.iters += 1
            
            if one:
                if self.iters%5 == 0:
                    if abs(self.current_pos.deltaX - valorAntecessorX) < nLimiar and abs(self.current_pos.deltaY - valorAntecessorY) < nLimiar:
                        self.is_path_plan_success = False
                        print("minimo local")
                        break
                        
                    valorAntecessorX = self.current_pos.deltaX
                    valorAntecessorY = self.current_pos.deltaY
                # if self.iters > 200:
                #     self.is_path_plan_success = True
                #     break
            
                self.newgx = self.current_pos.deltaX
                self.newgy = self.current_pos.deltaY

            if segundaVez:
                if self.iters%5 == 0:
                    if abs(self.current_pos.deltaX - valorAntecessorX) < nLimiar and abs(self.current_pos.deltaY - valorAntecessorY) < nLimiar:
                        self.is_path_plan_success = False
                        print("minimo local 2")
                        # if p1 != None: p1.checkEstatico = False
                        # self.px, self.py = deuRuim(int(self.current_pos.deltaX), int(self.current_pos.deltaY))
                        # self.is_path_plan_success = True
                        break
                        
                    valorAntecessorX = self.current_pos.deltaX
                    valorAntecessorY = self.current_pos.deltaY
            
            if signal:
                if p1 != None: p1.checkEstatico = False
                self.px, self.py = deuRuim(int(self.current_pos.deltaX), int(self.current_pos.deltaY))
                self.is_path_plan_success = True
                break
                
            self.path.append([self.current_pos.deltaX, self.current_pos.deltaY])
            
            # print(str(self.goal.deltaX) + " - " + str(self.goal.deltaY))
            
            if not colidir(self.obsx, self.obsy, self.current_pos.deltaX, self.current_pos.deltaY, self.goal.deltaX, self.goal.deltaY, value=5):
                count += 1
                if count > 10:
                    self.path = np.concatenate((self.path, simulate_points(self.current_pos.deltaX, self.goal.deltaX, self.current_pos.deltaY, self.goal.deltaY, juntos=True)), axis=0)
                    # print(self.path)
                    self.is_path_plan_success = True
                    break
            if self.is_plot:
                p = Pontos()
                plt.plot(self.current_pos.deltaX, self.current_pos.deltaY, '.b')
                plt.plot(p.xobs, p.yobs, ".k")
                plt.pause(self.delta_t)
        if (self.current_pos - self.goal).length <= self.goal_threashold:
            self.is_path_plan_success = True

    