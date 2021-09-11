# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np

def state_index(state, mx):
    return state[0] + mx * state[1]

def cruz(mx, my, env, Q):
    # faz setas para cada direção, onde a largura da seta representa o valor Q
    env.reset()
    env.render()
    arrow_length = 0.2
    c = 'k'
    width_factor = 0.001 / np.max(Q)
    for x in range(mx):
        for y in range(my):
            plt.arrow(x, y, 0., arrow_length, ec=c, fc=c, width=width_factor*Q[state_index((x, y), mx), 0])  # up
            plt.arrow(x, y, 0., -arrow_length, ec=c, fc=c, width=width_factor*Q[state_index((x, y), mx), 1])  # down
            plt.arrow(x, y, -arrow_length, 0., ec=c, fc=c, width=width_factor*Q[state_index((x, y), mx), 2])  # left
            plt.arrow(x, y, arrow_length, 0., ec=c, fc=c, width=width_factor*Q[state_index((x, y), mx), 3])  # right

def setas(mx, my, env, Q):
    # Plota as setas para a direção com o maior Q
    env.reset()
    env.render()
    l = 0.3
    c = 'k'
    w = 0.01
    for x in range(mx):
        for y in range(my):
            best_action = np.argmax(Q[state_index((x, y), mx)])
            if best_action == 0:            
                plt.arrow(x, y-0.5*l, 0, l, ec=c, fc=c, width=w)  # cima
            elif best_action == 1:
                plt.arrow(x, y+0.5*l, 0., -l, ec=c, fc=c, width=w)  # baixo
            elif best_action == 2:
                plt.arrow(x+0.5*l, y, -l, 0., ec=c, fc=c, width=w)  # esquerda
            elif best_action == 3:
                plt.arrow(x-0.5*l, y, l, 0., ec=c, fc=c, width=w)  # direita

def show_heatmap(mx, my, heatmap, env):
    plt.imshow(heatmap.T, interpolation='none', origin='lower')
    plt.plot(env.treasure[0], env.treasure[1], 'y*', mec='none', markersize=17)  
    plt.xlim(-0.5, mx - 0.5)
    plt.ylim(-0.5, my - 0.5)