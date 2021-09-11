# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from helper.ambiente import Pontos
from helper.utils import distancia_rota, diminuir_pontos, simulate_points
from machineLearning.ambienteML import MazeEnv
from machineLearning.commom import cruz, setas, show_heatmap
import time
from curves import bSpline

alpha = 0.9  # taxa de aprendizado, ie, qual fracao dos valores Q deve ser atualizada
gamma = 0.9  # fator de desconto, ie, algoritmo considera possíveis recompensas futuras
epsilon = 0.9  # probabilidade de escolher uma acao aleatória em vez da melhor acao
maxIt = 3000
epsilon_decay = 0.9

def choose_action(state, Q, mx, epsilon):
    action=0
    if np.random.rand() <= epsilon:
        action = np.random.randint(4)
    else:
        action = np.argmax(Q[state_index(state, mx)])
    epsilon *= epsilon_decay
    return action, epsilon

def history(mx, my, env, Q, alpha, epsilon):
    history = []
    recompensas = []
    heatmap = np.zeros((mx, my))
    count = 0
    thershold = -float("inf")

    for indexx in range(maxIt): 
        # print(indexx)
        if indexx == maxIt - 1: print("Treinamento concluído") 
        state = env.reset()
        done = False
        rw = 0
        c = 0
        
        while not done:
            heatmap[state] += 1
            
            action, epsilon = choose_action(state, Q, mx, epsilon)
            
            new_state, reward, done, info = env.step(action)
            rw += reward
            
            Q[state_index(state, mx), action] += alpha * (reward + gamma * np.max(Q[state_index(new_state, mx)]) - Q[state_index(state, mx), action])
            state = new_state

            c += 1
            if c > 100000: 
                print("drop")
                return None, None
        
        if len(recompensas) > 6 and recompensas[-1] == rw and recompensas[-5] == rw: thershold = rw
        recompensas.append(rw)
        history.append(info['time'])

    return heatmap, recompensas

def state_index(state, mx):
    return state[0] + mx * state[1]

def movingaverage(interval, window_size):
    window= np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'same')

def run(show=False, vmx=None, vmy=None, startx=None, starty=None, p1=None):
    start = time.time()
    p = Pontos()
    mx = max(p.limiar) + 1
    my = max(p.limiar) + 1

    # print("1")
    if startx == None:
        env = MazeEnv(mx, my, p.xs, p.ys, p.xt, p.yt, p.capaX, p.capaY, p.limiar)
    else:
        env = MazeEnv(mx, my, startx, starty, p.xt, p.yt, vmx, vmy, p.limiar)
    env.reset()

    Q = np.random.rand(mx * my, 4)  # 4 acoes possiveis

    heatmap, recompensas = history(mx, my, env, Q, alpha, epsilon)

    if recompensas == None:
        return -1, -1, [0,-10], [0,-10]

    if show:
        show_heatmap(mx, my, heatmap, env)
        cruz(mx, my, env, Q)
        setas(mx, my, env, Q)
    
    state = env.reset()
    done = False
    c = 0
    while not done:
        action = np.argmax(Q[state_index(state, mx)])
        
        new_state, _, done, _ = env.step(action)
        state = new_state
        c += 1
        if c > 10000: 
            print("drop")
            return -1, -1, [0,-10], [0,-10]

    plt1, px, py = env.render()

    xx, yy = diminuir_pontos(px, py, p.xobs, p.yobs, apf=True)
    
    curv = bSpline.B_spline(xx, yy)
    xnew, ynew = curv.get_curv()
    
    xxx, yyy = simulate_points(xnew[-1], p.xt, ynew[-1], p.yt)

    xnew = np.concatenate((xnew, xxx, [p.xt]), axis=0)
    ynew = np.concatenate((ynew, yyy, [p.yt]), axis=0)

    distancia = distancia_rota(px, py)
    
    return distancia, time.time()-start, xnew, ynew

if __name__ == "__main__":
    run()