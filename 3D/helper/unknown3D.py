# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import math

from helper.utils import *
from helper.ambiente import Pontos

import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D,art3d

from curves import bSpline

nome = ["Bi A-Star", "Rapid Random Tree Connect", "Particle Swarm Optimization", "Particle Swarm Optimization with Reinforcement Learning", "Reinforcement Learning", "Rapid Exploring Random Tree"]
obs = {}

# # Cena 1
vel = 50 # 80 quanto maior mais lento
tam = 50
# # subindo
# obs["0"] = {"dx": [35]*vel, "dy": np.linspace(0,tam,vel).tolist(), "dyr": tam_obs_dim(5), "dxr": tam_obs_dim(1)}
# obs["1"] = {"dx": [45]*vel, "dy": np.linspace(0,tam,vel).tolist(), "dyr": tam_obs_dim(5), "dxr": tam_obs_dim(1)}
# obs["2"] = {"dx": [10]*vel, "dy": np.linspace(0,tam,vel).tolist(), "dyr": tam_obs_dim(5), "dxr": tam_obs_dim(1)}
# # para esquerda
# obs["0"] = {"dx": np.linspace(tam,0,vel).tolist(), "dy": [48]*vel, "dyr": tam_obs_dim(1), "dxr": tam_obs_dim(5)}
# obs["1"] = {"dx": np.linspace(tam,0,vel).tolist(), "dy": [18]*vel, "dyr": tam_obs_dim(1), "dxr": tam_obs_dim(5)}
# obs["2"] = {"dx": np.linspace(tam,0,vel).tolist(), "dy": [38]*vel, "dyr": tam_obs_dim(1), "dxr": tam_obs_dim(5)}
# # para direita
# obs["0"] = {"dx": np.linspace(0,tam,vel).tolist(), "dy": [15]*vel, "dyr": tam_obs_dim(1), "dxr": tam_obs_dim(5)}
# # descendo
# obs["0"] = {"dx": [35]*vel, "dy": np.linspace(tam,0,vel).tolist(), "dyr": tam_obs_dim(5), "dxr": tam_obs_dim(1)}

# # Cena 2
# vel = 100
# obs["0"] = {"dx": [20]*vel, "dy": np.linspace(0,100,vel).tolist(), "dyr": tam_obs_dim(5), "dxr": tam_obs_dim(1)}

# # Cena 3
# vel = 80
# obs["0"] = {"dx": [38]*vel, "dy": np.linspace(0,50,vel).tolist(), "dyr": tam_obs_dim(5), "dxr": tam_obs_dim(1)}
# obs["1"] = {"dx": np.linspace(0,50,vel).tolist(), "dy": [15]*vel, "dyr": tam_obs_dim(1), "dxr": tam_obs_dim(5)}

p = Pontos()
def run(valid, q, alg, show=False, rmv=False):
    tempos = []
    # o ox e oy so existem ate implementar alguma visao
    xs, ys, zs, xt, yt, zt, ox, oy, oz, vx, vy, vz = p.xs, p.ys, p.zs, p.xt, p.yt, p.zt, p.xobs, p.yobs, p.zobs, p.visX, p.visY, p.visZ
    vddIndex = []
    
    if valid[q]:
        checkObsD = 0 # pode checar novamente obstaculo dinamico
        vAuxAux = 0.4 # o quanto a frente vai considerar
        obsD = 0 # confirmar que estou vendo um obstaculo dinamico
        vObsD = {"1": [], "2": []}
        finalx, finaly, finalz = [xs], [ys], [zs] # rota final
        a, b, c = [], [], [] # mapa com os pontos desde o inicio com capa
        a1, b1, c1 = [], [], [] # mapa com os pontos desde o inicio sem capa
        value = [] # como ta com spline tem mt ponto, isso eh pra n deixar travado em pontos mt proximos
        # n = len(finalx) # tamanho da rota
        i = 0 # ponto q vai ser andado
        step = 1 # de quantos em quantos metros vai andar

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        while abs(finalx[-1] != xt) or finaly[-1] != yt or finalz[-1] != zt:
            if False:# not colidir(ox, oy, finalx[-1], finaly[-1], xt, yt):
                aux = max(abs(xt - finalx[-1]), abs(yt - finaly[-1]))
                arrx = np.linspace(finalx[-1], xt, int(aux))
                arry = np.linspace(finaly[-1], yt, int(aux))
                arrz = np.linspace(finalz[-1], zt, int(aux))
                finalx = np.concatenate((finalx, arrx), axis=0)
                finaly = np.concatenate((finaly, arry), axis=0)
                finalz = np.concatenate((finaly, arrz), axis=0)
                if not rmv: print("acabou")
                break


            else:
                if len(finalx) == 1:
                    a, b, c = p.xobs, p.yobs, p.zobs #atualizarMapa3D(finalx[-1],finaly[-1],finalz[-1], ox, oy, oz, a, b, c) # com capa
                    a1, b1, c1 = p.xobs, p.yobs, p.zobs # atualizarMapa3D(finalx[-1],finaly[-1],finalz[-1], vx, vy, vz, a1, b1, c1) # sem capa
                    # a, b = atualizarMapa(finalx[-1],finaly[-1], vx, vy, a1, b1) # sem capa
                    # a1, b1 = a, b
                    # c1, c = [2] * len(a), [2] * len(a)
                    a = np.append(a, p.limiar[0])
                    b = np.append(b, p.limiar[1])
                    c = np.append(c, 5)
                    a = np.append(a, 0)
                    b = np.append(b, 0)
                    c = np.append(c, 0)
                    ax.set_xlabel('x (m)')
                    ax.set_ylabel('y (m)')
                    ax.set_zlabel('z (m)')
                    ax.plot3D(a1, b1, c1, ".k")
                    plt.pause(0.01)

                    t = -1
                    while t == -1 :
                        _, t, rx, ry, rz = alg.run(show=show, vmx=a, vmy=b, vmz=c, startx=finalx[0], starty=finaly[0], startz=finalz[0]) 
                        # alg.run(obsx=p.xobs, obsy=p.yobs, obsz=p.zobs, start=(p.xs,p.ys,p.zs), end=(p.xt,p.yt,p.zt))
                    
                    tempos.append(t)
                    # if q not in [0, 1, 3, 4, 6, 7, 8, 9, 10]: ##########################################################
                    #     rx.reverse() # os algoritmos retornam as rotas invertidas
                    #     ry.reverse()
                    #     rz.reverse()
                    
                    finalx = np.concatenate((finalx, rx), axis=0) # rota ate agora com os novos pontos do algoritmo
                    finaly = np.concatenate((finaly, ry), axis=0)
                    finalz = np.concatenate((finalz, rz), axis=0)
                    N = len(finalx)
                    value = [float("inf"), float("inf"), float("inf")]

                while i < N:
                    if value[0] <= finalx[i] - step or value[0] >= finalx[i] + step or value[1] <= finaly[i] - step or value[1] >= finaly[i] + step or value[2] <= finalz[i] - step or value[2] >= finalz[i] + step:
                    # if True:
                        # print(len(finalx))
                        # print(len(finaly))
                        # print(len(finalz))
                        tx, ty = [], [] # guardar valor temporario de obstaculo com obstaculo dinamico
                        vddIndex.append(i)
                        a, b, c = p.xobs, p.yobs, p.zobs # atualizarMapa3D(finalx[i],finaly[i],finalz[i], ox, oy, oz, a, b, c)
                        a1, b1, c1 = p.xobs, p.yobs, p.zobs # atualizarMapa3D(finalx[i],finaly[i],finalz[i], vx, vy, vz, a1, b1, c1)
                        # a, b = atualizarMapa(finalx[i],finaly[i], vx, vy, a1, b1)
                        # a1, b1 = a, b
                        # c1, c = [2] * len(a), [2] * len(a)
                        if not rmv: ax.set_ylabel('y (m)')
                        if not rmv: ax.set_xlabel('x (m)')
                        if not rmv: ax.set_zlabel('z (m)')
                        if not rmv: ax.plot3D(a1, b1, c1, ".k")
                        if not rmv: ax.plot3D(finalx, finaly, finalz, "-y")
                        if not rmv: plt.title(nome[q])
                        # try:
                        #     for letra in range(len(obs)):
                        #         qObs = str(letra)
                        #         for tamx in obs[qObs]["dxr"]:
                        #             tx.append(obs[qObs]["dx"][len(vddIndex)]+tamx)
                        #             ty.append(obs[qObs]["dy"][len(vddIndex)])
                        #             if not rmv: plt.plot(obs[qObs]["dx"][len(vddIndex)]+tamx, obs[qObs]["dy"][len(vddIndex)], ".g")
                        #         for tamy in obs[qObs]["dyr"]:
                        #             tx.append(obs[qObs]["dx"][len(vddIndex)])
                        #             ty.append(obs[qObs]["dy"][len(vddIndex)]+tamy)
                        #             if not rmv: plt.plot(obs[qObs]["dx"][len(vddIndex)], obs[qObs]["dy"][len(vddIndex)]+tamy, ".g")
                        # except:
                        #     pass




                        # ---------------------------------------------------------
                        # print(finalx[i])
                        # print(finaly[i])
                        if not rmv: ax.plot3D([finalx[i]], [finaly[i]], [finalz[i]], ".r")
                        if not rmv: plt.pause(0.1)
                        # print(p.checkEstatico)
                        if finalx[i] == xt and finaly[i] == yt and finalz[i] == zt:
                            break
                        if not rmv: plt.cla()
                        valorAuxiliar = int(len(rx)*vAuxAux)
                        if q == 1: valorAuxiliar *= 3
                        # ---------------------------------------------------------


                        # try: #----------------------------------------------------------
                        # checa se colide com algum obstaculo dinamico
                        distanciaObstaculo = 15
                        checarAFrente = 5 # [m]
                        pontoX, pontoY, _ = pontoAFrente(finalx[i], finaly[i], finalx, finaly, xt, yt, i, checarAFrente)
                        
                    #     if colidir(tx, ty, finalx[i], finaly[i], pontoX, pontoY, value=distanciaObstaculo, d=True) and checkObsD == 0:
                    #         if obsD == 1:

                    #             for txPoint, tyPoint in zip(tx, ty):
                    #                 # TA VENDO PONTO REPETIDO (ta vendo atras <----)
                    #                 if dist_euclidiana(finalx[i], finaly[i], txPoint, tyPoint) < distanciaObstaculo:# and abs(angle2 - angle0) > math.radians(60):
                    #                     vObsD["2"].append([txPoint, tyPoint])
                                 
                    #             if not rmv: print(vObsD["1"])
                    #             if not rmv: print(vObsD["2"])
                    #             # PRECISA IDENTIFICAR A DIRECAO SE NAO VAI DAR RUIM - OK
                    #             pontoColisao, direc = intersecao_arrays(vObsD["1"], vObsD["2"])
                    #             if not rmv: print(pontoColisao)
                    #             angle1 = definir_angulo(finalx[i], finaly[i], pontoColisao[0], pontoColisao[1])
                    #             angle2 = definir_angulo(finalx[i], finaly[i], pontoX, pontoY)
                    #             distParede, deixaTriangulo = 2, False

                    #             # ESSE 90 TEM Q SER 45 PRA CADA LADO - OK ?
                    #             if abs(angle1 - angle2) > math.radians(180) or colidir(a, b, finalx[i], finaly[i], pontoColisao[0], pontoColisao[1]):
                    #                 # TEM ALGO ERRADO AKI
                    #                 if not rmv: print("angle")
                    #                 if not rmv: print(math.degrees(angle1))
                    #                 if not rmv: print(math.degrees(angle2))
                    #                 if not rmv: print(math.degrees(angle1 - angle2))
                    #                 if not rmv: print("obstaculo dinamico n ira interferir")
                    #             elif pontoColisao[0] < distParede or pontoColisao[1] < distParede or abs(pontoColisao[0] - p.limiar[0]) < distParede or abs(pontoColisao[1] - p.limiar[1]) < distParede:
                    #                 if not rmv: print("o desvio ficará muito perto das paredes")
                    #                 deixaTriangulo = True
                    #             elif direc == 0 and pontoColisao[1] > pontoY: # subindo
                    #                 if not rmv: print("Obastaculo dinamico ja passou")
                    #             elif direc == 1 and pontoColisao[1] < pontoY: # descendo
                    #                 if not rmv: print("Obastaculo dinamico ja passou")
                    #             elif direc == 2 and pontoColisao[0] > pontoX: # esquerda
                    #                 if not rmv: print("Obastaculo dinamico ja passou")
                    #             elif direc == 3 and pontoColisao[0] > pontoX: # direita
                    #                 if not rmv: print("Obastaculo dinamico ja passou")
                    #             else: 
                    #                 # ?????????????????????
                    #                 # if len(finalx) > i+int(valorAuxiliar*2.5):
                    #                 #     print("1")
                    #                 #     rxx, ryy = replanning(finalx[i], finaly[i], pontoColisao[0], pontoColisao[1], finalx[i+int(valorAuxiliar*1.2)], finaly[i+int(valorAuxiliar*1.2)], finalx[i+int(valorAuxiliar*2.5)], finaly[i+int(valorAuxiliar*2.5)])

                    #                 #     auxi = max(abs(rxx[:-1][-1] - finalx[i+int(valorAuxiliar*2.5):][0]), abs(ryy[:-1][-1] - finaly[i+int(valorAuxiliar*2.5):][0]))
                    #                 #     a11 = np.linspace(rxx[:-1][-1], finalx[i+int(valorAuxiliar*2.5):][0], int(auxi))
                    #                 #     a22 = np.linspace(ryy[:-1][-1], finaly[i+int(valorAuxiliar*2.5):][0], int(auxi))

                    #                 #     finalx = np.concatenate((finalx[:i], rxx[:-1], a11[1:-1], finalx[i+int(valorAuxiliar*2.5):]), axis=0)
                    #                 #     finaly = np.concatenate((finaly[:i], ryy[:-1], a22[1:-1], finaly[i+int(valorAuxiliar*2.5):]), axis=0)
                    #                 # else:
                    #                 # print("2")
                                    

                    #                 if direc == 0: pontoColisao[1] -= 2
                    #                 if direc == 1: pontoColisao[1] += 2
                    #                 if direc == 2: pontoColisao[0] += 2
                    #                 if direc == 3: pontoColisao[0] -= 2

                    #                 if deixaTriangulo:
                    #                     # apesar da linha amarela, n apaga isso
                    #                     valorDeEspacox1, valorDeEspacox2 = 0, 0
                    #                     valorDeEspacoy1, valorDeEspacoy2 = 0, 0
                    #                 else:
                    #                     stepSpace = 1
                    #                     if direc == 0 or direc == 1: # obstaculo na vertical
                    #                         if finalx[i] < xt:
                    #                             valorDeEspacox1 = -stepSpace
                    #                             valorDeEspacox2 = stepSpace
                    #                             valorDeEspacoy1 = 0
                    #                             valorDeEspacoy2 = 0
                    #                         else:
                    #                             valorDeEspacox1 = stepSpace
                    #                             valorDeEspacox2 = +stepSpace
                    #                             valorDeEspacoy1 = 0
                    #                             valorDeEspacoy2 = 0
                    #                     else: # obstaculo na horizontal
                    #                         if finaly[i] < yt:
                    #                             valorDeEspacoy1 = -stepSpace
                    #                             valorDeEspacoy2 = stepSpace
                    #                             valorDeEspacox1 = 0
                    #                             valorDeEspacox2 = 0
                    #                         else:
                    #                             valorDeEspacoy1 = stepSpace
                    #                             valorDeEspacoy2 = -stepSpace
                    #                             valorDeEspacox1 = 0
                    #                             valorDeEspacox2 = 0

                    #                 # IMPLEMENTAR ESSA FUNCAO PARA PEGAR PONTO LOGO ACIMA OU CONTINUO
                    #                 # cfX, cfY, _ = pontoAFrente(pontoColisao[0], pontoColisao[1], finalx, finaly, xt, yt, i, 2)
                    #                 # ESSA FUNCAO N TA FUNCIONANDO, MAS PRECISO DELA NA LINHA 263
                    #                 # miraiPontoX, miraiPontoY = pontoReplanning(finalx, finaly, i, pontoColisao[0]+valorDeEspacox2, pontoColisao[1]+valorDeEspacoy2, direc, a1, b1, pontoX, pontoY)
                                    
                    #                 # print(pontoColisao[0]+valorDeEspacox2)
                    #                 # print(pontoColisao[0]+valorDeEspacoy2)
                    #                 # print(miraiPontoX)

                    #                 # isso vai dar ruim pq esse +2 deveria ser de acordo com a direcao
                    #                 # o sinal vai ser de acordo com o angulo entre o no atual e miraiPonto
                    #                 # rxx, ryy = replanning(finalx[i], finaly[i], pontoColisao[0]+valorDeEspacox1, pontoColisao[1]+valorDeEspacoy1, pontoColisao[0]+valorDeEspacox2, pontoColisao[1]+valorDeEspacoy2, miraiPontoX, miraiPontoY)#, xt, yt)
                    #                 # DEIXAR ESSE 2 E ESSE 1 DINAMICO
                    #                 rxx, ryy = simulate_points(finalx[i], pontoColisao[0]+2, finaly[i], pontoColisao[1]-1)

                    #                 # print(pontoColisao[1]+2)
                    #                 pontoX1, pontoY1, i1 = pontoAFrente(rxx[0], ryy[0], rxx, ryy, xt, yt, 0, 1.5)
                    #                 pontoX2, pontoY2, i2 = pontoAFrente(rxx[0], ryy[0], rxx, ryy, xt, yt, 0, 2)

                    #                 # suaviza a ENTRADA do replanning
                    #                 xx, yy = newSmooth([finalx[i], finalx[i+10], pontoX1, pontoX2], [finaly[i], finaly[i+10], pontoY1, pontoY2], offset=4)
                                    
                    #                 # suaviza a SAIDA do replanning
                    #                 xxx, yyy = newSmooth([rxx[-2], rxx[-1], 34, xt], [ryy[-2], ryy[-1], 28, yt], offset=4)
                                    
                    #                 # auxi = max(abs(rxx[:-1][-1] - xt), abs(ryy[:-1][-1] - yt))
                    #                 # a11 = np.linspace(rxx[:-1][-1], xt, int(auxi))
                    #                 # a22 = np.linspace(ryy[:-1][-1], yt, int(auxi))

                    #                 finalx = np.concatenate((finalx[:i], xx, rxx[i2:-1], xxx[1:-1], [xt]), axis=0)
                    #                 finaly = np.concatenate((finaly[:i], yy, ryy[i2:-1], yyy[1:-1], [yt]), axis=0)

                    #                 checkObsD = len(vddIndex)
                    #                 # print("ponto de colisao")
                    #                 # print(str(pontoColisao[0]) + " - " + str(pontoColisao[1]))
                    #                 if not rmv: print("replanning")

                    #                 N = len(finalx)
                    #             obsD = 0
                    #             vObsD["1"] = []
                    #             vObsD["2"] = []
                    #         else:
                    #             if not rmv: print("obstaculo dinamico--------")
                    #             ang0 = definir_angulo(finalx[i], finaly[i], pontoX, pontoY)
                    #             for txPoint, tyPoint in zip(tx, ty):
                    #                 ang1 = definir_angulo(finalx[i], finaly[i], txPoint, tyPoint)
                    #                 if dist_euclidiana(finalx[i], finaly[i], txPoint, tyPoint) < distanciaObstaculo and abs(ang0 - ang1) < math.radians(60):
                    #                     vObsD["1"].append([txPoint, tyPoint])
                    #             if len(vObsD["1"]) > 0:
                    #                 obsD = 1
                        
                        
                        
                        




                        
                        
                        
                        # checa se colide com obstaculo ou obstaculo dinamico
                        # elif colidir(a, b, finalx[i], finaly[i], pontoX, pontoY, value=0.1) and ((checkObsD + 3 <= len(vddIndex)) or checkObsD == 0):
                        # usa 0.1 no rl e 0.5 para os outros
                        # if colidir(a1, b1, finalx[i], finaly[i], pontoX, pontoY, value=0.1, d=True, direcionalAng=[finalx[i+1], finaly[i+1]]) and p.checkEstatico and ((checkObsD + 6 <= len(vddIndex)) or checkObsD == 0):
                        #     obsD = 0
                        #     vObsD = {"1": [], "2": []}
                        #     vax, vay, vaz = finalx[i], finaly[i], finalz[i]
                        #     if not rmv: print("recalculando a partir do ponto " + str(finalx[i]) + " - " + str(finaly[i]))
                        #     # _, _, rx, ry, rz = alg.run(show=False, vmx=np.concatenate((a, tx), axis=0), vmy=np.concatenate((b, ty), axis=0), startx=finalx[i], starty=finaly[i])
                        #     # try:
                        #     _, t, rx, ry, rz = alg.run(show=show, vmx=a, vmy=b, vmz=c, startx=finalx[i], starty=finaly[i], startz=finalz[i], p1=p)
                        #     # _, t, rx, ry, rz = alg.run()
                        #     # except:
                        #     #     if q == 1:
                        #     #         try:
                        #     #             _, t, rx, ry, rz = alg.run(show=show, vmx=a1, vmy=b1, startx=finalx[i], starty=finaly[i])
                        #     #         except:
                        #     #             try:
                        #     #                 _, t, rx, ry, rz = alg.run(show=show, vmx=a1, vmy=b1, startx=finalx[i], starty=finaly[i], normal=True)
                        #     #             except:
                        #     #                 _, t, rx, ry, rz = alg.run(show=show, vmx=a1, vmy=b1, startx=finalx[i], starty=finaly[i], signal=True)
                        #     #     else:
                        #     #         _, t, rx, ry, rz = alg.run(show=show, vmx=a1, vmy=b1, startx=finalx[i], starty=finaly[i])

                        #     tempos.append(t)
                        #     # if q not in [3, 4, 6, 7, 8, 9, 10]: ################################################3
                        #     #     try:
                        #     #         rx.reverse()
                        #     #         ry.reverse()
                        #     #         rz.reverse()
                        #     #     except:
                        #     #         rx = np.flip(rx, 0)
                        #     #         ry = np.flip(ry, 0)
                        #     #         rz = np.flip(rz, 0)

                        #     # rx[0], ry[0], rz[0] = vax, vay, vaz

                            


                        #     if rx[-1] != xt or ry[-1] != yt:
                        #         auxX0, auxY0, auxZ0 = simulate_points3D(rx[-1], xt, ry[-1], yt, rz[-1], zt)
                        #         rx = np.concatenate((rx, auxX0), axis=0)
                        #         ry = np.concatenate((ry, auxY0), axis=0)
                        #         rz = np.concatenate((rz, auxZ0), axis=0)


                        #     print(rx)

                        #     xx, yy, zz = [], [], []
                        #     xxx, yyy, zzz = [], [], []
                        #     xxxx, yyyy, zzzz = [], [], []

                        #     i1, i2, i3 = 0, 0, 0
                            
                        #     # if q not in [7, 8]:
                        #     #     pontoX1, pontoY1, i1 = pontoAFrente(rx[0], ry[0], rx, ry, xt, yt, 0, 1.5)
                        #     #     pontoX2, pontoY2, i2 = pontoAFrente(rx[0], ry[0], rx, ry, xt, yt, 0, 2)

                        #     #     xx, yy = newSmooth([finalx[i], finalx[i+10], pontoX1, pontoX2], [finaly[i], finaly[i+10], pontoY1, pontoY2], offset=4)

                        #     #     angle1 = definir_angulo(xx[-2], yy[-2], rx[int(i1):][0], ry[int(i1):][0])
                        #     #     angle2 = definir_angulo(xx[-4], yy[-4], xx[-3], yy[-4])
                        #     #     if  abs(angle1 - angle2) > math.pi/(180/60):
                        #     #         if abs(xx[-2] - xx[-1]) >= step:
                        #     #             xxx, yyy, zzz = simulate_points3D(xx[-2], rx[int(i2):][0], yy[-2], ry[int(i1):][0], zz[-2], rz[int(i1):][0])

                        #     #         if abs(xx[0] - xx[1]) >= step:
                        #     #             xxxx, yyyy, zzz = simulate_points3D(xx[0], xx[1], yy[0], yy[1], zz[0], zz[1])

                        #     # AINDA NAO TESTADO - Serve para desbloquear o mapa de calor do apf
                        #     nSaveCheck = len(finalx) + len(xx) + len(xxx) + len(xxxx) + len(rx)/2
                        #     if i >= nSaveCheck:
                        #         p.checkEstatico = True
                            
                            
                        #     # finalx = np.concatenate((finalx[:i], xxxx, xx[1:-1], xxx, rx[i2:]), axis=0) #, rx[int(nn*0.2):] , rx[len(xx):]
                        #     # finaly = np.concatenate((finaly[:i], yyyy, yy[1:-1], yyy, ry[i2:]), axis=0) # , ry[int(nn*0.2):], ry[len(yy):]
                        #     # finalz = np.concatenate((finalz[:i], zzzz, zz[1:-1], zzz, rz[i2:]), axis=0) # , rz[int(nn*0.2):], rz[len(zz):]
                        #     finalx = np.concatenate((finalx[:i], rx), axis=0) #, rx[int(nn*0.2):] , rx[len(xx):]
                        #     finaly = np.concatenate((finaly[:i], ry), axis=0) # , ry[int(nn*0.2):], ry[len(yy):]
                        #     finalz = np.concatenate((finalz[:i], rz), axis=0) # , rz[int(nn*0.2):], rz[len(zz):]

                        #     print(finalx[:i])
                        #     print(rx)

                        #     # v1, v2 = diminuir_pontos(finalx, finaly, p.xobs, p.yobs)
                        #     # curv = bSpline.B_spline(v1, v2)
                        #     # finalx, finaly = curv.get_curv()
                        #     # mx, my = smooth_reta(mx, my, offset=4)  


                        #     N = len(finalx)

                        
                        # if checkObsD + (distanciaObstaculo * 2) < len(vddIndex):
                        #     checkObsD = 0
                        # except:
                        #     pass
                        value[0] = finalx[i]
                        value[1] = finaly[i]
                        value[2] = finalz[i]
                    i += 1
                    # # print(i)
                    # # print(N)
                    # # print("--end--")





                # print(finalx[:])
                # print(finaly)
                
                ax.set_xlabel('x (m)')
                ax.set_ylabel('y (m)')
                ax.set_zlabel('z (m)')
                ax.plot3D([xs], [ys], [zs], color="green", marker="o", markersize=5)
                ax.plot3D([xt], [yt], [ys], color="blue", marker="o", markersize=5)
                ax.plot3D(finalx, finaly, finalz, "-r")
                ax.plot3D(a1, b1, c1, ".k")
                if not rmv: plt.show()
                distanciaFinal = distancia_rota3D(finalx, finaly, finalz)
                # print(tempos)
                return distanciaFinal, tempos, finalx, finaly, finalz, completness3D(finalx, finaly, finalz, a1, b1, c1)
