# -*- coding: utf-8 -*-
# import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from curves.bezier import Bezier
from curves import bSpline
import psutil
import os
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def rotacionar(x1, y1, x2, y2):
    a = definir_angulo(x1, y1, x2, y2)

    return a

def rotationMatrix(psi0, x1, y1, z1):
    r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
    pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
    return pos_local

def pseudo3D(rotax, rotay, visx, visy, visxz, visyz, altura=2):
    # cada ponto tem q checar todos os pontos a sua frente
    # se tiver colisao em 2D e n tiver em 3D
    # fazer uma trajetoria curvada por cima
    x, y, z = [], [], []
    # print(rotax)
    nMexe = True
    count = 0

    for i in range(len(rotay)):
        if nMexe:
            x.append(rotax[count])
            y.append(rotay[count])
            z.append(altura)

        for j in range(count, len(rotax)):
            # print(j)
            nMexe = True
            col2D = colidir(visx, visy, x[-1], y[-1], rotax[j], rotay[j])
            col3D = colidir(visxz, visyz, x[-1], y[-1], rotax[j], rotay[j])

            if col2D and col3D == False:
                m, n, _ = criar_reta(x[-1], y[-1], rotax[j], rotay[j])
                x.append(((y[-1] + ((rotay[j] - y[-1])/2)) - n)/m)
                y.append((m * (x[-1] + ((rotax[j] - x[-1])/2))) + n)
                z.append(altura+2)
                x.append(rotax[j])
                y.append(rotay[j])
                z.append(altura)
                nMexe = False
                count = j
                break

        count += 1
        
        if count == len(rotax):
            break
            #     m, n, _ = criar_reta(x[-1], y[-1], rotax[j], rotay[j])
            #     print(m)
            #     print(n)
            #     x.append(((y[-1] + (rotay[j] - y[-1])) + n)/m)
            #     y.append((m * (x[-1] + (rotax[j] - x[-1]))) + n)
            #     z.append(altura+1)

            #     x.append(rotax[j])
            #     y.append(rotay[j])
            #     z.append(altura)

            #     nMexe = False
            #     i = j
            #     break
            
    return x, y, z

def rotaToGazebo2(rotax, rotay, ox, oy):
    newPath_x, newPath_y = [rotax[0]], [rotay[0]]
    
    for i in range(2, len(rotax)):
        if newPath_x[-1] != rotax[i] and rotax[i] != rotay[i]:
            angle1 = definir_angulo(rotax[i-2],rotay[i-2],rotax[i-1],rotay[i-1])
            angle2 = definir_angulo(rotax[i-1],rotay[i-1],rotax[i],rotay[i])
            if angle1 < 0: angle1 = 360 - angle1
            if angle2 < 0: angle2 = 360 - angle2
            
            # if colidir(ox, oy, rotax[i-2], rotay[i-2], rotax[i], rotay[i]):
            #     newPath_x.append(rotax[i-1])
            #     newPath_y.append(rotay[i-1])
            
            elif abs(angle1 - angle2) >= math.radians(20):
                newPath_x.append(rotax[i-1])
                newPath_y.append(rotay[i-1])

    newPath_x.append(rotax[-1])
    newPath_y.append(rotay[-1])

    return newPath_x, newPath_y

def rotaToGazebo3D(rotax, rotay, rotaz, ox, oy, oz, value=1):
    newPath_x, newPath_y, newPath_z = [rotax[0]], [rotay[0]], [rotaz[0]]
    goalx, goaly, goalz = rotax[-1], rotay[-1], rotaz[-1]
    check = 1

    while newPath_x[-1] != goalx and newPath_y[-1] != goaly and newPath_z[-1] != goalz:
        # FALTA ALTERAR PARA O 3D
        if colidir(ox, oy, newPath_x[-1], newPath_y[-1], rotax[check], rotay[check]):
            newPath_x.append(rotax[check])
            newPath_y.append(rotay[check])
            newPath_z.append(rotaz[check])

        if dist_euclidiana3D(newPath_x[-1], newPath_y[-1], newPath_z[-1], rotax[check], rotay[check], rotaz[check]) > value:
            newPath_x.append(rotax[check])
            newPath_y.append(rotay[check])
            newPath_z.append(rotaz[check])

        if ((rotax[check] == goalx) and (rotay[check] == goaly) and (rotaz[check] == goalz)) and ((newPath_x[-1] != goalx) and (newPath_y[-1] != goaly) and (newPath_z[-1] != goalz)):
            newPath_x.append(rotax[check])
            newPath_y.append(rotay[check])
            newPath_z.append(rotaz[check])

        check += 1

    return newPath_x, newPath_y, newPath_z

def rotaToGazebo(rotax, rotay, ox, oy, value=1):
    newPath_x, newPath_y = [rotax[0]], [rotay[0]]
    goalx, goaly = rotax[-1], rotay[-1]
    check = 1

    while newPath_x[-1] != goalx and newPath_y[-1] != goaly:
        if colidir(ox, oy, newPath_x[-1], newPath_y[-1], rotax[check], rotay[check]):
            newPath_x.append(rotax[check])
            newPath_y.append(rotay[check])

        if dist_euclidiana(newPath_x[-1], newPath_y[-1], rotax[check], rotay[check]) > value:
            newPath_x.append(rotax[check])
            newPath_y.append(rotay[check])

        if ((rotax[check] == goalx) and (rotay[check] == goaly)) and ((newPath_x[-1] != goalx) and (newPath_y[-1] != goaly)):
            newPath_x.append(rotax[check])
            newPath_y.append(rotay[check])

        check += 1

    return newPath_x, newPath_y

def pontoAFrente(x, y, vetorx, vetory, gx, gy, pos, dist):
    px, py = None, None
    newPos = 0

    for i in range(pos, len(vetorx)):
        if abs(vetorx[i] - x) >= dist or abs(vetory[i] - y) >= dist:
            px = vetorx[i]
            py = vetory[i] 
            newPos = i
            break
    
    if px == None:
        px = gx
        py = gy
        newPos = len(vetorx)-1
    
    return px, py, newPos

def intersecao_arrays(a1, a2):
    # quer descobrir oq esta no a1 mas n ta no a2
    direc = 0
    
    if a1[0][0] == a2[0][0]: # obstaculo na vertical
        minValue1 = float("inf")
        minValue2 = float("inf")

        minValue1 = [value[1] for value in a1 if minValue1 > value[1]]
        minValue2 = [value[1] for value in a2 if minValue2 > value[1]]
        # for value in a1:
        #     if minValue1 > value[1]:
        #         minValue1 = value[1]

        # for value in a2:
        #     if minValue2 > value[1]:
        #         minValue2 = value[1]

        if minValue1 < minValue2: # Ta subindo
            print("SUBINDO")
            direc = 0
        else: 
            print("DESCENDO")
            direc = 1
    else: # obstaculo na horizontal
        minValue1 = float("inf")
        minValue2 = float("inf")

        minValue1 = [value[0] for value in a1 if minValue1 > value[0]]
        minValue2 = [value[0] for value in a2 if minValue2 > value[0]]

        if minValue1 < minValue2: # Ta subindo
            print("INDO PARA DIREITA")
            direc = 2
        else: 
            print("INDO PARA ESQUERDA")
            direc = 3

    for a in a1:
        if a not in a2:
            return a, direc

def inserirCapa(x, y, xy, tamCapa=1):
    capaX, capaY = x, y
    
    for a, b in zip (x, y):
        if [a+tamCapa, b] not in xy:
            capaX.append(a+tamCapa)
            capaY.append(b)
            xy.append([a+tamCapa, b])

        if [a-tamCapa, b] not in xy:
            capaX.append(a-tamCapa)
            capaY.append(b)
            xy.append([a-tamCapa, b])

        if [a, b+tamCapa] not in xy:
            capaX.append(a)
            capaY.append(b+tamCapa)
            xy.append([a, b+tamCapa])

        if [a, b-tamCapa] not in xy:
            capaX.append(a)
            capaY.append(b-tamCapa)
            xy.append([a, b-tamCapa])

        for j in range(1, tamCapa+1):
            if [a+tamCapa, b-j] not in xy:
                capaX.append(a+tamCapa)
                capaY.append(b-j)
                xy.append([a+tamCapa, b-j])

            if [a+j, b+tamCapa] not in xy:
                capaX.append(a+j)
                capaY.append(b+tamCapa)
                xy.append([a+j, b+tamCapa])

            if [a-tamCapa, b-j] not in xy:
                capaX.append(a-tamCapa)
                capaY.append(b-j)
                xy.append([a-tamCapa, b-j])

            if [a-j, b+tamCapa] not in xy:
                capaX.append(a-j)
                capaY.append(b+tamCapa)
                xy.append([a-j, b+tamCapa])
        
            if [a+j, b-tamCapa] not in xy:
                capaX.append(a+j)
                capaY.append(b-tamCapa)
                xy.append([a+j, b-tamCapa])

            if [a+tamCapa, b+j] not in xy:
                capaX.append(a+tamCapa)
                capaY.append(b+j)
                xy.append([a+tamCapa, b+j])

            if [a-j, b-tamCapa] not in xy:
                capaX.append(a-j)
                capaY.append(b-tamCapa)
                xy.append([a-j, b-tamCapa])

            if [a-tamCapa, b+j] not in xy:
                capaX.append(a-tamCapa)
                capaY.append(b+j)
                xy.append([a-tamCapa, b+j])

    return capaX, capaY

def laserROS(newOX, newOY, mapOX, mapOY, allO, tamCapa=0):
    capaX, capaY = [], []
    semMemoriaX, semMemoriaY = [], []

    # FALTA ADD A IDENTIFICACAO DE OBSTACULO DINAMICO
    for v1, v2 in zip(newOX, newOY):
        if [v1, v2] not in allO:
            semMemoriaX.append(v1)
            semMemoriaY.append(v2)
            mapOX.append(v1)
            mapOY.append(v2)
            allO.append([v1, v2])

    # capaX, capaY = inserirCapa(mapOX, mapOY, allO, tamCapa=tamCapa)

    # return capaX, capaY, semMemoriaX, semMemoriaY, allO
    return semMemoriaX, semMemoriaY, semMemoriaX, semMemoriaY, allO

def mapping3D(newOX, newOY, newOZ, mapOX, mapOY, mapOZ, allO, tamCapa=0):
    capaX, capaY, capaZ = [], [], []
    semMemoriaX, semMemoriaY, semMemoriaZ = [], [], []

    # FALTA ADD A IDENTIFICACAO DE OBSTACULO DINAMICO
    for v1, v2, v3 in zip(newOX, newOY, newOZ):
        if [v1, v2, v3] not in allO:
            semMemoriaX.append(v1)
            semMemoriaY.append(v2)
            semMemoriaZ.append(v3)
            mapOX.append(v1)
            mapOY.append(v2)
            mapOZ.append(v3)
            allO.append([v1, v2, v3])
            print("Mapa atualizado")

    # capaX, capaY = inserirCapa(mapOX, mapOY, allO, tamCapa=tamCapa)

    # return capaX, capaY, capaZ, semMemoriaX, semMemoriaY, semMemoriaZ, allO
    return mapOX, mapOY, mapOZ, semMemoriaX, semMemoriaY, semMemoriaZ, allO

def atualizarMapa(px, py, visaox, visaoy, preox, preoy):
    a = []
    b = []

    # checar os objetos em LoS
    for ox, oy in zip(visaox, visaoy):
        m, n, _ = criar_reta(px, py, ox, oy)
        count = 0
        for ox1, oy1 in zip(visaox, visaoy):
            c1 = min([ox, px]) <= ox1 <= max([ox, px])
            c2 = min([oy, py]) <= oy1 <= max([oy, py])
            if c1 and c2: # se tiver dentro do range da linha
                if dist_ponto(ox1, oy1, m, n) < 1 and ox1 != ox and oy1 != oy: # e colidir com um obstaculo
                    count += 1 # aumenta 1 no count
                else:
                    if ox == px: # geometria analitica n comtempla linhas verticais
                        # erro: ignora todos os pontos na vertical, estando ou n colidindo
                        # lado bom: com visao isso sera desnecessario
                        count -= 1
        
        if count == 0:   # se n colidir nenhuma vez eh oq vc ta vendo
            a.append(ox)
            b.append(oy)

    # Atualizar mapa
    path, pathex = [], []
    [path.append([x, y]) for (x, y) in zip(a, b)]
    [pathex.append([x, y]) for (x, y) in zip(preox, preoy)]
    countx, county = [], []
    for i in path:
        if i not in pathex:
            countx.append(i[0])
            county.append(i[1])

    obsx = np.concatenate((preox, countx), axis=0)
    obsy = np.concatenate((preoy, county), axis=0)

    # Faz o mapa apenas com oq ta vendo na hora
    # obsx = np.concatenate((countx, a), axis=0)
    # obsy = np.concatenate((county, b), axis=0)

    return obsx, obsy

def simulate_points(x1, x2, y1, y2, juntos=False):
    aux = math.ceil(max(abs(x1 - x2), abs(y1 - y2)))
    aux *= 2
    a1 = np.linspace(x1, x2, int(aux))
    a2 = np.linspace(y1, y2, int(aux))

    if juntos:
        jj = []
        for i in range(len(a1)):
            jj.append([a1[i], a2[i]])

        return jj
    else:
        return a1, a2

def arredondarTraj(x, y, z=0, threeD=0):
    if threeD == 0:
        trajX, trajY = [], []
        trajXY = []
        for i, j in zip(x, y):
            if [round(i), round(j)] not in trajXY: trajXY.append([round(i), round(j)])
        for i in trajXY:
            trajX.append(i[0])   
            trajY.append(i[1])
        return trajX, trajY   
    else:
        trajX, trajY, trajZ = [], [], []
        trajXYZ = []
        for i, j, k in zip(x, y, z):
            if [round(i), round(j), round(k)] not in trajXYZ: trajXYZ.append([round(i), round(j), round(k)])
        for i in trajXYZ:
            trajX.append(i[0])   
            trajY.append(i[1])
            trajZ.append(i[2])
        return trajX, trajY, trajZ

def suavizar_curva(x, y):
    curv = bSpline.B_spline(x, y)
    xnew, ynew = curv.get_curv()

    return xnew, ynew

def replanning(px, py, cx, cy, nx, ny, gx, gy):
    xnew, ynew = smooth_reta([px, cx, nx, gx], [py, cy, ny, gy], offset=1)
    
    a1, a2 = simulate_points(xnew[0], xnew[1], ynew[0], ynew[1])
    
    xnew = np.concatenate((a1, xnew[2:]), axis = 0)
    ynew = np.concatenate((a2, ynew[2:]), axis = 0)
    # curv = bSpline.B_spline([px, cx, nx, gx], [py, cy, ny, gy])
    # xnew, ynew = curv.get_curv()
    # XS = np.concatenate(([px], [cx, nx], [gx]), axis=0)
    # YS = np.concatenate(([py], [cy, ny], [gy]), axis=0)
    # k = XS.size
    # TS = np.linspace(0, 1, k)
    # tt = np.linspace(0, 1, 100)
    # tcx = interpolate.splrep(TS, XS)
    # tcy = interpolate.splrep(TS, YS)
    # xx = interpolate.splev(tt, tcx)
    # yy = interpolate.splev(tt, tcy)

    return xnew, ynew

def smooth_bspline(px, py, cx, cy, nx, ny, gx, gy, v=0):
    if v == 0:
        curv = bSpline.B_spline([px, cx, nx, gx], [py, cy, ny, gy])
        xnew, ynew = curv.get_curv()
        return xnew, ynew
    else:
        XS = np.concatenate(([px], [cx, nx], [gx]), axis=0)
        YS = np.concatenate(([py], [cy, ny], [gy]), axis=0)
        k = XS.size
        TS = np.linspace(0, 1, k)
        tt = np.linspace(0, 1, 100)
        tcx = interpolate.splrep(TS, XS)
        tcy = interpolate.splrep(TS, YS)
        xx = interpolate.splev(tt, tcx)
        yy = interpolate.splev(tt, tcy)

        return xx, yy

def tam_obs_dim(tam):
    if tam % 2 == 0 : tam += 1
    return np.linspace(tam-int(tam/2), tam+int(tam/2), tam) - tam

def memory_usage():
    # return the memory usage in percentage like top
    process = psutil.Process(os.getpid())
    mem = process.memory_percent()
    return mem

def distancia_rota(pathx, pathy=[]):
    distancia = 0
    if len(pathy) == 0:
        px, py = [], []
        for i in range(len(pathx)):
            px.append(pathx[i][0])
            py.append(pathx[i][1]) 
            for q in range(0, len(px)-1):
                distancia += math.sqrt(((px[q+1] - px[q])**2) + ((py[q+1] - py[q])**2))
        return distancia, px, py
    else:
        for q in range(0, len(pathx)-1):
            distancia += math.sqrt(((pathx[q+1] - pathx[q])**2) + ((pathy[q+1] - pathy[q])**2))
        return distancia

def colidirTrajetoria(ox, oy, rotax, rotay, pos, value=0.5):
    # value = 0.1 for outdoor environment and 0.35 for indoor environment
    for (x, y) in zip(rotax[pos:], rotay[pos:]):
        for (o1, o2) in zip(ox, oy):
            ola = dist_euclidiana(o1, o2, x, y)
            # if ola < 10: print(dist_euclidiana(o1, o2, x, y))
            if ola < value:
                return True

    return False

def colisaoTrajetoria(ox, oy, rotax, rotay, pos=0, value=0.2):
    # value = 0.1 for outdoor environment and 0.35 for indoor environment
    value = pos
    for (x, y) in zip(rotax[pos:], rotay[pos:]):
        for (o1, o2) in zip(ox, oy):
            ola = dist_euclidiana(o1, o2, x, y)
            # if ola < 10: print(dist_euclidiana(o1, o2, x, y))
            if ola < value:
                return True, x, y, value
        value += 1

    return False, None, None, None

def completness(rotaX, rotaY, ox, oy, value=0.25):
    for rx, ry in zip(rotaX, rotaY):
        for obsx, obsy in zip (ox, oy):
            if dist_euclidiana(rx, ry, obsx, obsy) <= value:
                return True

    return False

def colidir3D(ox, oy, oz, x1, y1, z1, x2, y2, z2, value=0.5, d=False, show=False, direcionalAng=[None,None]):
    # AINDA N ESTA PRONTO
    # y menor horizontal
    # x menor vertical
    vertical = True if abs(x1-x2) < abs(y1-y2) else False
    
    v, h = 0, 0
    
    if d:
        if vertical: h = value
        if not vertical: v = value
    
    m, n, ang = criar_reta(x1, y1, x2, y2)
    if ang < 0: ang + math.pi * 2

    c4 = False

    if direcionalAng[0] != None:
        angAux = math.atan2(direcionalAng[1] - y1, direcionalAng[0] - x1)
        if angAux < 0: ang + math.pi * 2
        c4 = abs(angAux - ang) <= 90
    else:
        c4 = True

    for obs in zip(ox, oy):
        c1 = min([x1, x2]) - h <= obs[0] <= max([x1, x2]) + h
        c2 = min([y1, y2]) - v <= obs[1] <= max([y1, y2]) + v
        c3 = dist_ponto(obs[0], obs[1], m, n) < value
        if c1 and c2 and c3 and c4: #colidiu
            # if show: print(str(x1) + " - " + str(y1))
            # if show: print(str(x2) + " - " + str(y2))
            # if show: print(obs)
            # if show: print(dist_ponto(obs[0], obs[1], m, n))
            return True
    return False

def colidir(ox, oy, x1, y1, x2, y2, value=0.5, d=False, show=False, direcionalAng=[None,None]):
    # y menor horizontal
    # x menor vertical
    vertical = True if abs(x1-x2) < abs(y1-y2) else False
    
    v, h = 0, 0
    
    if d:
        if vertical: h = value
        if not vertical: v = value
    
    m, n, ang = criar_reta(x1, y1, x2, y2)
    if ang < 0: ang + math.pi * 2

    c4 = False

    if direcionalAng[0] != None:
        angAux = math.atan2(direcionalAng[1] - y1, direcionalAng[0] - x1)
        if angAux < 0: ang + math.pi * 2
        c4 = abs(angAux - ang) <= 90
    else:
        c4 = True

    for obs in zip(ox, oy):
        c1 = min([x1, x2]) - h <= obs[0] <= max([x1, x2]) + h
        c2 = min([y1, y2]) - v <= obs[1] <= max([y1, y2]) + v
        c3 = dist_ponto(obs[0], obs[1], m, n) < value
        if c1 and c2 and c3 and c4: #colidiu
            # if show: print(str(x1) + " - " + str(y1))
            # if show: print(str(x2) + " - " + str(y2))
            # if show: print(obs)
            # if show: print(dist_ponto(obs[0], obs[1], m, n))
            # if value == 1:
            #     print("x " + str(obs[0]) + " y " + str(obs[1]))
            # print(obs[0])
            # print(obs[1])
            # print(dist_ponto(obs[0], obs[1], m, n))
            # print("-----")
            return True
    return False

def newColidir(ox, oy, x1, y1, x2, y2, value=0.5):
    m, n, _ = criar_reta(x1, y1, x2, y2)
    cc = 0

    for obs in zip(ox, oy):
        c1 = min([x1, x2]) <= obs[0] <= max([x1, x2]) 
        c2 = min([y1, y2]) <= obs[1] <= max([y1, y2]) 
        c3 = dist_ponto(obs[0], obs[1], m, n) < value
        if c1 and c2 and c3: #colidiu
            cc += 1
    return cc

def minValue(matriz, zero=False):
    menor = float("inf")
    index = [0, 0]
    for r in range(len(matriz)):
        for n in range(len(matriz[0])):
            if zero:
               if menor > matriz[r][n] > 0:
                    index = [r, n] 
                    menor = matriz[r][n]  
            else:
                if menor > matriz[r][n]:
                    index = [r, n] 
                    menor = matriz[r][n]
    return menor, index

def maxValue(matriz):
    maior = -float("inf")
    index = [0, 0]
    for r in range(len(matriz)):
        for n in range(len(matriz[0])):
            if maior < matriz[r][n]:
                index = [r, n] 
                maior = matriz[r][n]
    return maior, index

def pontoReplanning(vetorX, vetorY, pos, pColisionX, pColisionY, direc, a1, b1, px, py):
    miraiPontoX, miraiPontoY = float("inf"), float("inf")
    m = float("inf")

    if direc == 0: pColisionY -= 10
    if direc == 1: pColisionY += 10
    if direc == 2: pColisionX += 10
    if direc == 3: pColisionX -= 10
    
    if direc == 2 or direc == 3: # obstaculo na vertical - encontrar ponto proximo a x
        for i in range(pos, len(vetorX)):
            if m > abs(vetorX[i] - pColisionX):
                m = abs(vetorX[i] - pColisionX)
                if not colidir(a1, b1, pColisionX, pColisionY, vetorX[i], vetorY[i], value=0.5, d=True):
                    miraiPontoX, miraiPontoY = vetorX[i], vetorY[i]
                    break
    else:  # obstaculo na horizontal - encontrar ponto proximo a y
        for i in range(pos, len(vetorY)):
            if m > abs(vetorY[i] - pColisionY):
                m = abs(vetorY[i] - pColisionY)
                if not colidir(a1, b1, pColisionX, pColisionY, vetorX[i], vetorY[i], value=0.5, d=True):
                    miraiPontoX, miraiPontoY = vetorX[i], vetorY[i]
                    break

    if miraiPontoX == float("inf") or miraiPontoY == float("inf"):
        miraiPontoX, miraiPontoY = px, py

    # print("pontos do futuro")
    # print(miraiPontoX)
    # print(miraiPontoY)
    return miraiPontoX, miraiPontoY

def diminuir_pontos(x, y, ox, oy, apf=False):
    newPath_x, newPath_y = [x[0]], [y[0]]
    goalx, goaly = x[-1], y[-1]
    check = 1

    while newPath_x[-1] != goalx or newPath_y[-1] != goaly:
        if not colidir(ox, oy, newPath_x[-1], newPath_y[-1], goalx, goaly):
            newPath_x.append(goalx)
            newPath_y.append(goaly)
            print("--------------------")
            break
        else:
            try:
                if colidir(ox, oy, newPath_x[-1], newPath_y[-1], x[check], y[check], d=True, value=0.4):
                    # print("colidiu")
                    # print( str(newPath_x[-1]) + " - " + str(newPath_y[-1]))
                    # print( str(x[check]) + " - " + str(y[check]))
                    newPath_x.append(x[check-1])
                    if apf: newPath_x.append(x[check]) # comment ?
                    newPath_y.append(y[check-1])
                    if apf: newPath_y.append(y[check]) # comment ?
                    check += 1
                else:
                    check += 1
            except: 
                newPath_x.append(goalx)
                newPath_y.append(goaly)

    return newPath_x, newPath_y
                
def criar_reta(x1, y1, x2, y2):
    delta_y = y2 - y1
    delta_x = x2 - x1
    
    if delta_x == 0:
        m = 0
    else:
        m = delta_y / delta_x # equivalente a a
        
    angulo = math.atan2(delta_y, delta_x)
    n = y2 - (m * x2)     # equivalente a c
    # b sempre vai ser -1

    return m, n, angulo


def intersecao_ponto(x_ponto, y_ponto, m, n):
    aux = abs((m * x_ponto) - y_ponto + n)
    if aux == 0:
        return True # o ponto cruza a linha
    else:
        return False # o ponto nao cruza a linha

def intersecao_reta(x1, y1, x2, y2):
    if ((y2 - y1) + (x2 - x1)) == 0:
        return False # as retas nao se cruzam
    else: 
        return True # as retas se cruzam

def dist_euclidiana(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

def dist_euclidiana3D(x1, y1, z1, x2, y2, z2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2) + math.pow(z2 - z1, 2))

def triangulo(ca=None, co=None, hip=None, alfa=None, oqquer="hip"):
    if oqquer == "alfa":
        if ca == None:
            alfa = math.asin(co/hip)
        elif co == None:
            alfa = math.acos(ca/hip)
        else:
            alfa = math.atan(co/ca)

        return alfa
    
    if oqquer == "hip":
        hip = math.pow(ca, 2) + math.pow(co, 2)
        
        return hip
    
    if oqquer == "co":
        if alfa == None:
            co = math.pow(hip, 2) / math.pow(ca, 2)
        else:
            co = math.sin(math.radians(alfa)) * hip

        return co
        
    if oqquer == "ca":
        if alfa == None:
            ca = math.pow(hip, 2) / math.pow(co, 2)
        else:
            ca = math.cos(math.radians(alfa)) * hip

        return ca
 
def dist_ponto(x_ponto, y_ponto, m, n):
    dividendo = abs((m * x_ponto) - y_ponto + n)
    divisor = math.sqrt(m * m + 1)

    return dividendo/divisor

def getBezier(start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset=2, t=0.86):
    b = Bezier(start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset, t)
    p = b.calc_4points_bezier_path()

    assert p.T[0][0] == start_x, "path is invalid"
    assert p.T[1][0] == start_y, "path is invalid"
    assert p.T[0][-1] == end_x, "path is invalid"
    assert p.T[1][-1] == end_y, "path is invalid"

    return p.T[0], p.T[1]

def definir_angulo(x1, y1, x2, y2):
    delta_y = y2 - y1
    delta_x = x2 - x1
    
    angle = math.atan2(delta_y, delta_x)
    
    return angle

def smooth_reta(path_x, path_y, offset=2):
    n = len(path_x)
    
    newPath_x = [path_x[0]]
    newPath_y = [path_y[0]]
    for i in range(1, n-2, 2):
        _, _, angulo1 = criar_reta(path_x[i-1], path_y[i-1], path_x[i], path_y[i])
        _, _, angulo2 = criar_reta(path_x[i+1], path_y[i+1], path_x[i+2], path_y[i+2])

        # print(str(path_x[i-1]) + " - " + str(path_y[i-1]) + " - " + str(path_x[i]) + " - " + str(path_y[i]) + " - " + str(angulo1))
        # print(str(path_x[i+1]) + " - " + str(path_y[i+1]) + " - " + str(path_x[i+2]) + " - " + str(path_y[i+2]) + " - " + str(angulo2))
        # print("x1: " + str(path_x[i]) + " - y1: " + str(path_y[i]) + " - angle1: " + str(math.degrees(angulo1)) + " - x2: " + str(path_x[i+1]) + " - y2: " + str(path_y[i+1]) + " - angle2: " + str(math.degrees(angulo2)))
        
        c1, c2 = getBezier(path_x[i], path_y[i], angulo1, path_x[i+1], path_y[i+1], angulo2, offset)
        [newPath_x.append(valor_x) for valor_x in c1]
        [newPath_y.append(valor_y) for valor_y in c2]
    
    newPath_x.append(path_x[-1])
    newPath_y.append(path_y[-1])
    
    return newPath_x, newPath_y

def newReplanning(bsx, bsy, sx, sy, cx, cy, ex, ey, aex, aey, offset=2):
    # lembrar que (bsx, bsy) e (aex, aey) nao vao fazer parte da trajetoria
    newPath_x = [sx] 
    newPath_y = [sy]

    _, _, angulo1 = criar_reta(bsx, bsy, sx, sy)
    _, _, angulo2 = criar_reta(cx, cy, ex, ey)

    c1, c2 = getBezier(sx, sy, angulo1, cx, cy, angulo2, offset)
    [newPath_x.append(valor_x) for valor_x in c1]
    [newPath_y.append(valor_y) for valor_y in c2]

    _, _, angulo3 = criar_reta(newPath_x[-2], newPath_y[-2], cx, cy)
    _, _, angulo4 = criar_reta(ex, ey, aex, aey)

    c3, c4 = getBezier(cx, cy, angulo3, ex, ey, angulo4, offset)
    [newPath_x.append(valor_x) for valor_x in c3]
    [newPath_y.append(valor_y) for valor_y in c4]

    return newPath_x, newPath_y

def newSmooth(path_x, path_y, offset=2):
    newPath_x = [path_x[0]] 
    newPath_y = [path_y[0]]
    # print(path_y)
    _, _, angulo1 = criar_reta(path_x[0], path_y[0], path_x[1], path_y[1])
    # print(math.degrees(math.atan2(0.1, 0)))
    # print(math.degrees(angulo1))

    _, _, angulo2 = criar_reta(path_x[2], path_y[2], path_x[3], path_y[3])

    c1, c2 = getBezier(path_x[0], path_y[0], angulo1, path_x[3], path_y[3], angulo2, offset)
    [newPath_x.append(valor_x) for valor_x in c1]
    [newPath_y.append(valor_y) for valor_y in c2]

    return newPath_x, newPath_y

def distanciaTotalRota(xx, yy):
    distance =  0
    for q in range(0, len(xx)-1):
        distance += math.sqrt(((xx[q+1] - xx[q])**2) + ((yy[q+1] - yy[q])**2))
    return distance

def passos_locais(path_x, path_y):
    proximoPasso = {"x": [], "y": [], "a": []}
    aux = 0
    for i in range(1, len(path_x)-1):
        proximoPasso["x"].append(path_x[i] - path_x[i-1])
        proximoPasso["y"].append(path_y[i] - path_y[i-1])
        _, _, a1 = criar_reta(path_x[i-1], path_y[i-1], path_x[i], path_y[i])
        auxiliar = ((math.pi/2)-(a1-aux)) if i == 1 else (aux-a1)
        proximoPasso["a"].append(auxiliar)
        aux = a1

    return proximoPasso

def draw_cylinder(center_x,center_y,radius,height_z):
    z = np.linspace(0, height_z, 10)
    theta = np.linspace(0, 2*np.pi, 20)
    theta_grid, z_grid=np.meshgrid(theta, z)
    x_grid = radius*np.cos(theta_grid) + center_x
    y_grid = radius*np.sin(theta_grid) + center_y
    return x_grid,y_grid,z_grid

def cuboid_data(o, size=(1,1,1)):
    X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
         [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
         [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
         [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
         [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
         [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
    X = np.array(X).astype(float)
    for i in range(3):
        X[:,:,i] *= size[i]
    X += np.array(o)
    return X

def draw_bar(positions,sizes=None,colors=None, **kwargs):
    if not isinstance(colors,(list,np.ndarray)): colors=["C0"]*len(positions)
    if not isinstance(sizes,(list,np.ndarray)): sizes=[(1,1,1)]*len(positions)
    g = []
    for p,s,c in zip(positions,sizes,colors):
        g.append( cuboid_data(p, size=s) )
    return Poly3DCollection(np.concatenate(g),  
                            facecolors=np.repeat(colors,6), **kwargs)
