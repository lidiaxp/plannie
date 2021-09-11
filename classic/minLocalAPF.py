# -*- coding: utf-8 -*-
from helper.ambiente import Pontos
import matplotlib.pyplot as plt
import numpy as np
from helper.utils import newColidir, colidir, smooth_reta, simulate_points, minValue, maxValue, suavizar_curva
import time
from curves import bSpline

def deuRuim(entradax, entraday):
    p = Pontos()
    # tempin = time.time()

    valor = 10

    dx = np.linspace(0, p.limiar, int(valor))
    dy = np.linspace(0, p.limiar, int(valor))

    last = [p.xt, p.yt]
    # start = [p.xs, p.ys]
    start = [entradax, entraday]

    # print(newColidir(p.xobs, p.yobs, 50, 50, 55, 55))
    # print(not colidir(p.xobs, p.yobs, last[0], last[1], start[0], start[1]))
    # print(colidir(p.xobs, p.yobs, 88, 55, start[0], start[1]))
    rotax, rotay = [p.xt], [p.yt]

    # print(colidir(p.xobs, p.yobs, 77, 66, start[0], start[1], value=0.1))
    while colidir(p.xobs, p.yobs, last[0], last[1], start[0], start[1]):
        perdas = []
        perda_f = []

        for i in range(int(valor)):
            perda_f.append([])

        perda = np.zeros((int(valor),int(valor)))
        coincidencia = 0
        for i in range(int(valor)):
            for j in range(int(valor)):
                # d = 50/np.log10(np.sqrt(np.power(dx[i] - last[1], 2.0) + np.power(dy[j] - last[0], 2.0)))
                d1 = np.log10(np.sqrt(np.power(dx[i] - start[1], 2.0) + np.power(dy[j] - start[0], 2.0)))
                c, cc = 0, 0

                c += newColidir(p.xobs, p.yobs, last[0], last[1], dx[j], dy[i])
                cc += newColidir(p.xobs, p.yobs, start[0], start[1], dx[j], dy[i])

                # perda[i][j] = d + d1
                # if cc == 0: perda[i][j] += d*2 - c*50
                if cc == 0 and c == 0:
                    coincidencia = 1
                # print(coincidencia)
                if coincidencia:
                    if c == 0 and cc == 0: perda[i][j] += d1*2
                else:
                    if c == 0: perda[i][j] += d1*2 - cc*50 

            perdas.append(perda)

        for i in range(int(valor)):
            for j in range(int(valor)):
                n_teste = []
                for k in range(1):
                    n_teste.append(perdas[k][i][j])
            
                perda_f[i].append(min(n_teste)) 

        # plt.plot(p.visX, p.visY, ".k")
        # plt.imshow(perda_f,cmap='jet',extent=[0,p.limiar,0,p.limiar],origin='lower')
        # plt.show()        

        if coincidencia:
            # print("maior q zero")
            _, e = minValue(perda_f, zero=True)
        else:
            _, e = minValue(perda_f)
        # print(a)
        # print(e)
        rotax.append(int(dx[e[1]]))
        rotay.append(int(dy[e[0]])+3) #acho q da pra ser 1 a mais doq o casaco do ambiente
        last = [int(dy[e[1]]), int(dx[e[0]])]
        

        # plt.imshow(perda_f,cmap='jet',extent=[0,p.limiar,0,p.limiar],origin='lower')
        # plt.plot(p.visX, p.visY, ".k")
        # plt.colorbar()
        # plt.show()        


    rotax.append(start[0])
    rotay.append(start[1])

    # rotax.reverse()
    # rotay.reverse()

    print(rotax)
    print(rotay)

    xx, yy = suavizar_curva(rotax, rotay)

    auxx1, auxy1 = simulate_points(xx[0], xx[1], yy[0], yy[1])
    auxx2, auxy2 = simulate_points(xx[-2], xx[-1], yy[-2], yy[-1])

    xx = np.concatenate((auxx1, xx[1:-1], auxx2), axis=0)
    yy = np.concatenate((auxy1, yy[1:-1], auxy2), axis=0)

    # for xxx, yyy in zip(xx, yy):
    #     path.append([xxx, yyy])

    # print(time.time() - tempin)

    # plt.plot(p.visX, p.visY, ".k")
    # plt.plot(xx, yy, "-r")
    # plt.show()
    return xx, yy#, path

