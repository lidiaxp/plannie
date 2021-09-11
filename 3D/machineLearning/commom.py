# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import matplotlib

def state_index(state, mx, my=20, mz=20):
    return (state[0] + mx * state[1]) * mz

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

def cuboid_data(center, size=(1,1,1)):
    # code taken from
    # http://stackoverflow.com/questions/30715083/python-plotting-a-wireframe-3d-cuboid?noredirect=1&lq=1
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the (left, outside, bottom) point
    o = [a - b / 2 for a, b in zip(center, size)]
    # get the length, width, and height
    l, w, h = size
    x = [[o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in bottom surface
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in upper surface
         [o[0], o[0] + l, o[0] + l, o[0], o[0]],  # x coordinate of points in outside surface
         [o[0], o[0] + l, o[0] + l, o[0], o[0]]]  # x coordinate of points in inside surface
    y = [[o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in bottom surface
         [o[1], o[1], o[1] + w, o[1] + w, o[1]],  # y coordinate of points in upper surface
         [o[1], o[1], o[1], o[1], o[1]],          # y coordinate of points in outside surface
         [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w]]    # y coordinate of points in inside surface
    z = [[o[2], o[2], o[2], o[2], o[2]],                        # z coordinate of points in bottom surface
         [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],    # z coordinate of points in upper surface
         [o[2], o[2], o[2] + h, o[2] + h, o[2]],                # z coordinate of points in outside surface
         [o[2], o[2], o[2] + h, o[2] + h, o[2]]]                # z coordinate of points in inside surface
    return x, y, z

def plotCubeAt(pos=(0,0,0), c="b", alpha=0.1, ax=None):
    # Plotting N cube elements at position pos
    if ax !=None:
        X, Y, Z = cuboid_data( (pos[0],pos[1],pos[2]) )
        ax.plot_surface(X, Y, Z, color=c, rstride=1, cstride=1, alpha=0.1)

def plotMatrix(ax, x, y, z, data, cmap="jet", cax=None, alpha=0.1):
    # plot a Matrix 
    norm = matplotlib.colors.Normalize(vmin=data.min(), vmax=data.max())
    colors = lambda i,j,k : matplotlib.cm.ScalarMappable(norm=norm,cmap = cmap).to_rgba(data[i,j,k]) 
    for i, xi in enumerate(x):
            for j, yi in enumerate(y):
                for k, zi, in enumerate(z):
                    plotCubeAt(pos=(xi, yi, zi), c=colors(i,j,k), alpha=alpha,  ax=ax)

    if cax !=None:
        cbar = matplotlib.colorbar.ColorbarBase(cax, cmap=cmap,
                                norm=norm,
                                orientation='vertical')  
        cbar.set_ticks(np.unique(data))
        # set the colorbar transparent as well
        cbar.solids.set(alpha=alpha) 
           
def histogram3D(x, y, z):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    _x = x
    _y = y
    _xx, _yy = np.meshgrid(_x, _y)
    x, y = _xx.ravel(), _yy.ravel()
    _z = np.array(z)

    # There may be an easier way to do this, but I am not aware of it
    z = np.zeros(len(x))
    for i in range(1, len(x)):
        z[i] = _z[(i*len(_z)) / len(x)]

    bottom = np.zeros_like(z)
    width = depth = 1

    ax.bar3d(x, y, bottom, width, depth, z, shade=True)
    plt.show()
