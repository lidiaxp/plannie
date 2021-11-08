# -*- coding: utf-8 -*-
import numpy as np
from math import ceil

class Pontos:
    def __init__(self):
        self.checkEstatico = True
        tamCapa = 0
        self.limiar = (20,10)

        altZ = 2
        self.xs = 2
        self.ys = 2
        self.zs = 2

        # Cena 1 e 2
        self.xt = 19#self.limiar[0] - 5
        self.yt = 9#self.limiar[1] - 5
        self.zt = 3

        

        ## -----------------------------------------------------------------------------
        obsVx = {}
        obsVy = {}

        obsHx = {}
        obsHy = {}

        obsVxz = {}
        obsVyz = {}

        obsHxz = {}
        obsHyz = {}

        self.visX = []
        self.visY = []
        for i in range(len(obsVx)):
            for ovx, ovy in zip(obsVx[str(i)], obsVy[str(i)]):
                self.visX.append(float(ovx))
                self.visY.append(float(ovy))

        for i in range(len(obsHx)):
            for ohx, ohy in zip(obsHx[str(i)], obsHy[str(i)]):
                self.visX.append(float(ohx))
                self.visY.append(float(ohy))

        self.visXz = []
        self.visYz = []
        for i in range(len(obsVxz)):
            for ovx, ovy in zip(obsVxz[str(i)], obsVyz[str(i)]):
                self.visXz.append(float(ovx))
                self.visYz.append(float(ovy))

        for i in range(len(obsHxz)):
            for ohx, ohy in zip(obsHxz[str(i)], obsHyz[str(i)]):
                self.visXz.append(float(ohx))
                self.visYz.append(float(ohy))

        self.capaX = []
        self.capaY = []

        for i in range(len(obsVx)):
            maxX, minX = max(obsVx[str(i)]), min(obsVx[str(i)])
            maxY, minY = max(obsVy[str(i)]), min(obsVy[str(i)])
            for ovx, ovy in zip(obsVx[str(i)], obsVy[str(i)]):
                self.capaX = np.append(self.capaX, float(ovx) + (tamCapa))
                self.capaX = np.append(self.capaX, float(ovx) - (tamCapa))
                self.capaY = np.append(self.capaY, float(ovy))
                self.capaY = np.append(self.capaY, float(ovy))
            for j in range(1, tamCapa+1):
                self.capaX = np.append(self.capaX, maxX+tamCapa)
                self.capaX = np.append(self.capaX, minX+tamCapa)
                self.capaX = np.append(self.capaX, minX-tamCapa)
                self.capaX = np.append(self.capaX, maxX-tamCapa)
                self.capaY = np.append(self.capaY, maxY+j-1)
                self.capaY = np.append(self.capaY, minY-j)
                self.capaY = np.append(self.capaY, maxY+j-1)
                self.capaY = np.append(self.capaY, minY-j)
            values = np.arange(((tamCapa*2) - 1)) - ceil(tamCapa/2)
            if len(values) == 1: values = [0]
            for element in values:
                self.capaX = np.append(self.capaX, maxX+element)
                self.capaX = np.append(self.capaX, maxX-element)
                self.capaY = np.append(self.capaY, maxY+tamCapa-1)
                self.capaY = np.append(self.capaY, minY-tamCapa)

        for i in range(len(obsHx)):
            maxX, minX = max(obsHx[str(i)]), min(obsHx[str(i)])
            maxY, minY = max(obsHy[str(i)]), min(obsHy[str(i)])
            for ohx, ohy in zip(obsHx[str(i)], obsHy[str(i)]):
                self.capaX = np.append(self.capaX, float(ohx))
                self.capaX = np.append(self.capaX, float(ohx))
                self.capaY = np.append(self.capaY, float(ohy) + (tamCapa))
                self.capaY = np.append(self.capaY, float(ohy) - (tamCapa))
            for j in range(1, tamCapa+1):
                self.capaX = np.append(self.capaX, maxX+j-1)
                self.capaX = np.append(self.capaX, minX-j)
                self.capaX = np.append(self.capaX, maxX+j-1)
                self.capaX = np.append(self.capaX, minX-j)
                self.capaY = np.append(self.capaY, maxY+tamCapa)
                self.capaY = np.append(self.capaY, minY+tamCapa)
                self.capaY = np.append(self.capaY, minY-tamCapa)
                self.capaY = np.append(self.capaY, maxY-tamCapa)
            values = np.arange(((tamCapa*2) - 1)) - ceil(tamCapa/2)
            if len(values) == 1: values = [0]
            for element in values:
                self.capaX = np.append(self.capaX, maxX+tamCapa-1)
                self.capaX = np.append(self.capaX, minX-tamCapa)
                self.capaY = np.append(self.capaY, maxY+element)
                self.capaY = np.append(self.capaY, maxY-element)

        obsx = np.concatenate((self.visX, self.capaX), axis=0)
        obsy = np.concatenate((self.visY, self.capaY), axis=0)
        self.capaZ = [altZ] * len(self.capaX)

        ox, oy = [], []
        passo = 1
        for i in range(0, self.limiar[1], passo):
            self.visX.append(0)
            self.visY.append(i)
            ox.append(0)
            oy.append(i)
        for i in range(0, self.limiar[1], passo):
            self.visX.append(self.limiar[0])
            self.visY.append(i)
            ox.append(self.limiar[0])
            oy.append(i)
        for i in range(0, self.limiar[0], passo):
            self.visX.append(i)
            self.visY.append(0)
            ox.append(i)
            oy.append(0)
        for i in range(0, self.limiar[0], passo):
            self.visX.append(i)
            self.visY.append(self.limiar[1])
            ox.append(i)
            oy.append(self.limiar[1])

        for (ax, ay) in zip (obsx, obsy):
            ox.append(ax)
            oy.append(ay)

        self.xobs = ox
        self.yobs = oy
        self.zobs = [0] * len(self.xobs)
        self.zobs1 = [1] * len(self.xobs)
        self.zobs2 = [2] * len(self.xobs)
        self.zobs3 = [3] * len(self.xobs)
        self.zobs4 = [4] * len(self.xobs)
        self.zobs5 = [5] * len(self.xobs)

        # You should not pass
        # Base3D
        vermelhoX = []
        vermelhoY = []
        zobs00 = [0] * len(vermelhoX)
        zobs11 = [1] * len(vermelhoX)
        zobs22 = [2] * len(vermelhoX)
        zobs33 = [3] * len(vermelhoX)
        zobs44 = [4] * len(vermelhoX)
        zobs55 = [5] * len(vermelhoX)

        # so por cima
        # Base3D
        rosaX = [6,6,6,6,6,6,6,6,6]
        rosaY = [1,2,3,4,5,6,7,8,9]
        zobs0 = [0] * len(rosaX)
        zobs1 = [1] * len(rosaX)
        zobs2 = [2] * len(rosaX)

        # so por baixo
        # Base3D
        amareloX = [14,14,14,14,14,14,14,14,14]
        amareloY = [1,2,3,4,5,6,7,8,9]
        zobs222 = [2] * len(amareloX)
        zobs3 = [3] * len(amareloX)
        zobs4 = [4] * len(amareloX)
        zobs5 = [5] * len(amareloX)

        # # # Outros
        self.xobs = np.concatenate((vermelhoX,vermelhoX,vermelhoX,vermelhoX,vermelhoX,vermelhoX, self.xobs,self.xobs,self.xobs,self.xobs,self.xobs,self.xobs,rosaX,rosaX,rosaX,rosaX,amareloX), axis=0)
        self.yobs = np.concatenate((vermelhoY,vermelhoY,vermelhoY,vermelhoY,vermelhoY,vermelhoY, self.yobs,self.yobs,self.yobs,self.yobs,self.yobs,self.yobs,rosaY,rosaY,rosaY,rosaY,amareloY), axis=0)
        self.zobs = np.concatenate((zobs00,zobs11,zobs22,zobs33,zobs44,zobs55, self.zobs,self.zobs1,self.zobs2,self.zobs3,self.zobs4,self.zobs5,zobs0,zobs1,zobs2,zobs3,zobs222), axis=0)
        
        
        self.capaX = self.xobs
        self.capaY = self.yobs
        self.capaZ = self.zobs

        self.visX = self.xobs
        self.visY = self.yobs
        self.visZ = self.zobs
        
        self.raio = 0.8
        self.robs = [self.raio] * len(self.xobs)
        
        # limite que pode andar no x
        self.xmin = min(np.concatenate(([self.xs], self.xobs, [self.xt]), axis=0)) - 1
        self.xmax = max(np.concatenate(([self.xs], self.xobs, [self.xt]), axis=0)) + 1

        # limite que pode andar no y
        self.ymin = min(np.concatenate(([self.ys], self.yobs, [self.yt]), axis=0)) - 1
        self.ymax = max(np.concatenate(([self.ys], self.yobs, [self.yt]), axis=0)) + 1

        # limite que pode andar no z
        self.zmin = 1
        self.zmax = 5

        self.xs += 0.1
        self.ys += 0.1
        self.xt += 0.1
        self.yt += 0.1
