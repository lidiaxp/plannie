# -*- coding: utf-8 -*-
import numpy as np
from math import ceil
import matplotlib.pyplot as plt

class Pontos:
    def __init__(self):
        self.checkEstatico = True
        tamCapa = 0
        self.limiar = (20,20)

        self.xs = 2
        self.ys = 2

        self.xt = 18
        self.yt = 8

        obsVxz = {"0": [25]*(47-30), "1": [35]*(47-30), "2": [55]*(27-18), "3": [65]*(27-18), "4": [15]*(28-19), "5": [80]*(40-30), "6": [18]*(80-70)}
        obsVyz = {"0": list(np.arange(47-30+1)+30), "1": list(np.arange(47-30+1)+30), "2": list(np.arange(27-18+1)+18), "3": list(np.arange(27-18+1)+18), "4": list(np.arange(28-19+1)+7), "5": list(np.arange(40-30+1)+30), "6": list(np.arange(80-70+1)+70)}

        obsHxz = {"0": list(np.arange(35-25+1)+25), "1": list(np.arange(35-25+1)+25), "2": list(np.arange(65-55+1)+55), "3": list(np.arange(65-55+1)+55), "4": list(np.arange(63-50+1)+50), "5": list(np.arange(48-35+1)+35)}
        obsHyz = {"0": [47]*(35-25+1), "1": [30]*(35-25+1), "2": [27]*(65-55+1), "3": [18]*(65-55+1), "4": [50]*(63-50), "5": [15]*(48-35)}

        ## -----------------------------------------------------------------------------

        # # 10
        # obsVx = {"0": [2]*7, "1": [4]*7, "2": [6]*7, "3": [8]*7}
        # obsVy = {"0": {2,3,4,5,6,7,8}, "1": {2,3,4,5,6,7,8}, "2": {2,3,4,5,6,7,8}, "3": {2,3,4,5,6,7,8}}
        # obsHx = {}
        # obsHy = {}

        # 20
        # obsVx = {"0": [2]*12, "1": [3]*12, "2": [5]*12, "3": [6]*12, "4": [8]*12, "5": [9]*12, "6": [11]*12, "7": [12]*12, "8": [14]*12, "9": [15]*12, "10": [17]*12, "11": [18]*12}
        # obsVy = {"0": {2,3,5,6,8,9,11,12,14,15,17,18}, "1": {2,3,5,6,8,9,11,12,14,15,17,18}, "2": {2,3,5,6,8,9,11,12,14,15,17,18}, "3": {2,3,5,6,8,9,11,12,14,15,17,18}, "4": {2,3,5,6,8,9,11,12,14,15,17,18}, "5": {2,3,5,6,8,9,11,12,14,15,17,18}, "6": {2,3,5,6,8,9,11,12,14,15,17,18}, "7": {2,3,5,6,8,9,11,12,14,15,17,18}, "8": {2,3,5,6,8,9,11,12,14,15,17,18}, "9": {2,3,5,6,8,9,11,12,14,15,17,18}, "10": {2,3,5,6,8,9,11,12,14,15,17,18}, "11": {2,3,5,6,8,9,11,12,14,15,17,18}}
        # obsHx = {}
        # obsHy = {}

        # 30
        # obsVx = {"0": [15]*13, "1": [20]*7, "2": [6]*9, "3": [11]*4, "4": [16]*4, "5": [3]*5, "6": [7]*5}
        # obsVy = {"0": {1,2,3,4,5,6,7,8,9,10,11,12,13}, "1": {16,17,18,19,20,21,22}, "2": {10,11,12,13,14,15,16,17,18}, "3": {22,23,24,25}, "4": {22,23,24,25}, "5": {23,24,25,26,27}, "6": {23,24,25,26,27}}
        # obsHy = {"0": [15]*10, "1": [26]*7, "2": [22]*4, "3": [25]*4, "4": [23]*3, "5": [10]*5, "6": [18]*5}
        # obsHx = {"0": {20,21,22,23,24,25,26,27,28,29}, "1": {23,24,25,26,27,28,29}, "2": {12,13,14,15}, "3": {12,13,14,15}, "4": {4,5,6}, "5": {2,3,4,5,6}, "6": {2,3,4,5,6}}

        # obsVx = {"0": [7]*13, "1": [20]*7, "2": [14]*13}
        # obsVy = {"0": {1,2,3,4,5,6,7,8,9,10,11,12,13}, "1": {12,13,14,15,16,17,18}, "2": {17,18,19,20,21,22,23,24,25,26,27,28,29}}
        # obsHy = {"0": [11]*10}
        # obsHx = {"0": {20,21,22,23,24,25,26,27,28,29}}

        ## -----------------------------------------------------------------------------

        # # Default
        obsVx = {"0": [10,10,10,10,10,10,10,10,10,10,10,10], "1": [20,20,20,20,20]}
        obsVy = {"0": [1,2,3,4,5,6,7,8,9,10,11,12], "1": [5,6,7,8,9]}

        obsHx = {"0": [1,2,3,4,5,6,7,8,9,10,11,12], "1": [1,2,3,4,5]}
        obsHy = {"0": [50,50,50,50,50,50,50,50,50,50,50,50], "1": [60,60,60,60,60]}


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

        # Cena Teste
        # start node
        # self.xs = 5
        # self.ys = 5
        # goal node
        # self.xt = 92
        # self.yt = 92
        # obstaculos estaticos
        # obsx = [90,90,90,90,80,80,80,80,50,50,50,50,70,70,70,70,35,35,35,35,40,40,40,40,35,45,45,45,45,91,92,93,94,81,82,83,84,36,37,38,39,41,42,43,44,45,71,72,73,74,76,77,78,79,86,87,88,89,51,52,53,54,31,32,33,34,36,37,38,39,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,35,35,35,35,35,35,35,35,31,32,33,34,36,37,38,39,20,20,20,20,86,87,88,89,61,62,63,64,90,90,90,90,90,90,90,90,90,90,90,90,85,85,85,85,85,85,85,85,75,75,75,75,75,75,75,75,65,65,65,65,85,85,85,85,60,60,60,60,41,42,43,44,45,45,45,45,45,15.0, 15,15,15,15,15.0, 15.0,15,15,15,15,16,17,18,19, 15.0,20,20,20,20, 20.0, 20.0, 20.0,26,27,28,29, 20.0, 25.0, 25.0,25,25,25, 25.0, 30.0, 30.0, 30.0,30,30,30,30, 30.0, 35.0, 35.0, 35.0, 35.0, 40.0, 40.0, 40.0, 40.0, 45.0, 45.0, 50.0, 50.0, 50.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 55.0, 60.0, 60.0, 65.0, 65.0, 75.0, 70.0, 70.0, 70.0, 75.0, 75.0, 75.0, 80.0, 80.0, 85.0, 85.0, 85.0, 85.0, 85.0, 85.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 95.0]
        # obsy = [21,22,23,24,41,42,43,44,16,17,18,19,16,17,18,19,21,22,23,24,21,22,23,24,40,26,27,28,29,25,25,25,25,40,40,40,40,25,25,25,25,25,25,25,25,25,45,45,45,45,45,45,45,45,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,41,42,43,44,46,47,48,49,51,52,53,54,56,57,58,59,61,62,63,64,41,42,43,44,46,47,48,49,40,40,40,40,40,40,40,40,31,32,33,34,70,70,70,70,80,80,80,80,56,57,58,59,61,62,63,64,66,67,68,69,79,78,77,76,74,73,72,71,61,62,63,64,69,68,67,66,79,78,77,76,84,83,82,81,84,83,82,81,80,80,80,80,80,79,78,77,76,15.0, 16,17,18,19,20.0, 80.0,81,82,83,84,85,85,85,85, 85.0,86,87,88,89, 30.0, 35.0, 85.0,65,65,65,65, 90.0, 60.0, 61.0,62,63,64, 65.0, 20.0, 40.0, 65.0,66,67,68,69, 70.0, 45.0, 50.0, 25.0, 20.0, 20.0, 25.0, 40.0, 80.0, 30.0, 75.0, 15.0, 20.0, 90.0, 20.0, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 85.0, 80.0, 80.0, 75.0, 65.0, 45.0, 20.0, 15.0, 65.0, 60.0, 45.0, 45.0, 40.0, 40.0, 20.0, 85.0, 80.0, 75.0, 70.0, 70.0, 65.0, 60.0, 55.0, 25.0, 20.0, 25.0]
        # obsx = [2,3]
        # obsy = [2,3]

        # # Cena 1
        # obsx = [1, 2, 3, 4, 4, 4, 6, 6, 6, 6, 6, 7, 8, 9]
        # obsy = [5, 5, 5, 5, 4, 3, 3, 4, 5, 6, 7, 6, 6, 6]
        # self.limiar = 10

        # # Cena 2
        # obsx = [24,1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 1, 2, 3, 4, 5, 6, 7, 1, 2, 3, 4, 5, 6, 7, 12, 13, 12, 13, 12, 13, 12, 13, 12, 13, 12, 13, 12, 13, 12, 13, 14, 15, 16, 17, 18, 19, 17, 18, 19, 17, 18, 19, 17, 18, 19, 17, 18, 19, 20, 21, 22, 20, 21, 22, 23, 24, 17, 18, 19, 20, 21, 22, 23, 24, 17, 18, 19, 20, 21, 22, 23, 24]
        # obsy = [9,4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 22, 22, 22, 23, 23, 23, 24, 24, 24, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15]
        # self.limiar = 25

        # # Cena 3
        # obsx = [19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 8, 9, 10, 8, 9, 10, 8, 9, 10, 8, 9, 10, 8, 9, 10, 8, 9, 10, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 39, 40, 41, 42, 43, 44, 39, 40, 41, 42, 43, 44, 39, 40, 41, 42, 43, 44, 39, 40, 41, 42, 43, 44, 39, 40, 41, 42, 43, 44, 39, 40, 41, 42, 43, 44, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 30, 31, 14, 15, 16, 17, 18, 19, 14, 15, 16, 17, 18, 19, 14, 15, 16, 17, 18, 19, 14, 15, 16, 17, 18, 19, 14, 15, 16, 17, 18, 19, 14, 15, 16, 17, 18, 19, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 35, 36, 37, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 5, 6, 7, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49]
        # obsy = [45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 39, 39, 39, 40, 40, 40, 41, 41, 41, 42, 42, 42, 43, 43, 43, 44, 44, 44, 45, 45, 45, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 38, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 44, 35, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 38, 38, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 21, 21, 21, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7]
        # self.limiar = 50

        # # Cena 4
        # obsx = [5]
        # obsy = [5]
        # self.limiar = 10

        # obsx = [4,5,6,7,8,9,20,20,20,20,20,20,30,30,30,30,30]
        # obsy = [6,6,6,6,6,6,12,13,14,15,16,17,28,29,30,31,32]
        # ox, oy = [], []
        # self.limiar = 50



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
            oy.append(self.limiar[0])

        for (ax, ay) in zip (obsx, obsy):
            ox.append(ax)
            oy.append(ay)

        self.xobs = ox
        self.yobs = oy
        self.raio = 0.8
        self.robs = [self.raio] * len(self.xobs)

        # limite que pode andar no x
        self.xmin = min(np.concatenate(([self.xs], self.xobs, [self.xt]), axis=0))
        self.xmax = max(np.concatenate(([self.xs], self.xobs, [self.xt]), axis=0))

        # limite que pode andar no y
        self.ymin = min(np.concatenate(([self.ys], self.yobs, [self.yt]), axis=0))
        self.ymax = max(np.concatenate(([self.ys], self.yobs, [self.yt]), axis=0))

        self.xs += 0.1
        self.ys += 0.1
        self.xt += 0.1
        self.yt += 0.1

if __name__ == '__main__':
    p = Pontos()

    plt.plot(p.xobs, p.yobs, ".k")
    plt.show()

    print(p.xobs)
    print(p.yobs)
