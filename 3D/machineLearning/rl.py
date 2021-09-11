# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from helper.ambiente import Pontos
from helper.utils import distancia_rota3D, diminuir_pontos3D, simulate_points, tirarRepertido3D, rotaToGazebo3D, memory_usage
from machineLearning.ambienteML import MazeEnv
from machineLearning.commom import cruz, setas, show_heatmap, plotMatrix, histogram3D
import time
from curves import bSpline
from curves.spline3D import generate_curve
import psutil
import os

alpha = 0.9  # taxa de aprendizado, ie, qual fracao dos valores Q deve ser atualizada
gamma = 0.9  # fator de desconto, ie, algoritmo considera possíveis recompensas futuras
epsilon = 0.9  # probabilidade de escolher uma acao aleatória em vez da melhor acao
maxIt = 400
epsilon_decay = 0.999999

p = Pontos()


def discover_direction(tx, ty, tz, tx2, ty2, tz2):
    p1 = abs(tx2 - tx)
    p2 = abs(ty2 - ty)
    p3 = abs(tz2 - tz)
    p = [p1, p2, p3]
    maxIndex = p.index(max(p))
    

    if maxIndex == 2 and tz2 >= tz: # cima
        return 0
    elif maxIndex == 2 and tz2 < tz: # baixo
        return 1
    elif maxIndex == 1 and ty2 >= ty: # up
        return 2
    elif maxIndex == 1 and ty2 < ty: # down
        return 3
    elif maxIndex == 0 and tx2 >= tx: # left
        return 4
    else: # right
        return 5

def choose_action(state, Q, mx, my, mz, epsilon):
    action=0
    if np.random.rand() <= epsilon:
        action = np.random.randint(6)
    else:
        action = np.argmax(Q[state_index(state, mx, my, mz)])
    epsilon *= epsilon_decay
    return action, epsilon

def history(mx, my, mz, env, Q, alpha, epsilon, show):
    history = []
    recompensas = []
    a1, a2, a3 = np.zeros((mx)), np.zeros((my)), np.zeros((mz))
    heatmap = np.zeros((mx, my, mz))
    count = 0
    thershold = -float("inf")

    for indexx in range(maxIt):
        print(indexx)
        # print(epsilon)
        if indexx == maxIt - 1: print("Treinamento concluído") 
        state = env.reset()
        done = False
        rw = 0
        c = 0
        t1, t2, t3 = [], [], []
        while not done:
            heatmap[state] += 1
            a1[state[0]] += 1
            a2[state[1]] += 1
            a3[state[2]] += 1
            t1.append(state[0])
            t2.append(state[1])
            t3.append(state[2])

            action, epsilon = choose_action(state, Q, mx, my, mz, epsilon)
            
            new_state, reward, done, info = env.step(action)
            
            rw += reward
            
            Q[state_index(state, mx, my, mz), action] += alpha * (reward + gamma * np.max(Q[state_index(new_state, mx, my, mz)]) - Q[state_index(state, mx, my, mz), action])
            
            state = new_state
            c += 1
            if c > 100000: 
                print("drop")
                break

        if indexx == maxIt - 1:
            t1.append(p.xt)
            t2.append(p.yt)
            t3.append(p.zt)
            t1x, t2x, t3x = diminuir_pontos3D(t1, t2, t3, p.xobs, p.yobs, p.zobs, apf=0, value=1)
            px, py, pz = tirarRepertido3D(t1x, t2x, t3x)
            if show:
                ax = plt.axes(projection = "3d")
                ax.plot3D(t1, t2, t3, 'b-')
                ax.plot3D(px, py, pz, 'y-')
                ax.plot3D(t1x, t2x, t3x, 'g-')
                ax.plot3D(p.xobs, p.yobs, p.zobs, '.k')
                ax.set_zlim(0,5)
                plt.show()

        if len(recompensas) > 51 and recompensas[-1] == rw and recompensas[-5] == rw: thershold = rw
        recompensas.append(rw)
        history.append(info['time'])

    return heatmap, recompensas, px, py, pz

def state_index(state, mx, my, mz):
    return (state[0] + mx * state[1]) * mz

def movingaverage(interval, window_size):
    window= np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'same')

def run(show=False, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    start = time.time()
    cpu = psutil.cpu_percent()
    memory = memory_usage()
    p = Pontos()
    mx = 125#p.limiar[1] + 3
    my = 30#p.limiar[0] + 3
    mz = 26

    # print("1")
    if startx == None:
        env = MazeEnv(mx, my, mz, p.xs, p.ys, p.zs, p.xt, p.yt, p.zt, p.xobs, p.yobs, p.zobs, p.limiar)
    else:
        env = MazeEnv(mx, my, mz, startx, starty, startz, p.xt, p.yt, p.zt, vmx, vmy, vmz, p.limiar)
    # print("2")
    # env.render()    
    env.reset()

    Q = np.random.rand(mx * my * mz, 6)  # 4 acoes possiveis

    heatmap, recompensas, px, py, pz = history(mx, my, mz, env, Q, alpha, epsilon, show)
    

    ax = plt.axes(projection = "3d")
    ax.plot3D(px, py, pz, 'g-')
    ax.plot3D(p.xobs, p.yobs, p.zobs, '.k')
    ax.set_zlim(0,5)
    plt.show()

    # if show:
    # histogram3D(a1, a2, a3)
    # show_heatmap(mx, my, heatmap, env)
    # cruz(mx, my, env, Q)
    # setas(mx, my, env, Q)
    # plt.plot(np.arange(len(recompensas)),recompensas)
    # y_av = movingaverage(recompensas, 20)
    # plt.plot(np.arange(len(y_av)), y_av,"r")
    # plt.show()
        
    xx, yy, zz = px, py, pz

    xx1, yy1, zz1 = diminuir_pontos3D(px, py, pz, p.xobs, p.yobs, p.zobs)
    xx1, yy1, zz1 = tirarRepertido3D(xx1, yy1, zz1)

    xx, yy, zz = generate_curve(xx1, yy1, zz1)
    
    xx, yy, zz = generate_curve(xx, yy, zz)
    distancia = distancia_rota3D(xx, yy, zz)
    
    # if show:
    ax = plt.axes(projection = "3d")
    ax.plot3D(px, py, pz, 'g-')
    ax.plot3D(xx, yy, zz, 'r-')
    ax.plot3D(p.xobs, p.yobs, p.zobs, '.k')
    plt.show()
    
    print("end")

    return distancia, time.time()-start, xx1, yy1, zz1

if __name__ == "__main__":
    run()