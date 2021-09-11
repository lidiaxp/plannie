# -*- coding: utf-8 -*-
import sys
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d

from sensor_msgs.msg import PointCloud2, LaserScan, NavSatFix
from sensor_msgs import point_cloud2

from helper.utils import *
from helper.ambiente import Pontos
from classic import Astar_test as alg
# from classic import rrt_connect_3d as alg
# from metaHeuristic import pso as alg
# from machineLearning import rl as alg

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion

from std_msgs.msg import String, Int32
from sensor_msgs.msg import BatteryState

from datetime import datetime
import statistics as stc
from haversine import haversine
from sys import exit
import psutil

from utilsUAV import *

class globalPlanner:
    def __init__(self):
        # Tags
        self.land, self.takeoff, self.hover, self.sec, self.garraOn, self.esperarCamera, self.garraOff, self.calibrarTarget = 100, 200, 300, 400, 500, 600, 700, 800

        # Flags
        self.addRota = 0
        self.log = 0
        self.pos = 0
        self.goToHome = 0
        self.letra = ""
        self.unic = {"SM": 0, "busy": 0, "print": 0, "hover": 0, "definirRota": 0, "sec": 0, "andar": 0, "bateria": 0, "bateriaGazebo": 0, "gpsrtk": 0, "pos": 0}
        self.rotina = {"normal": 0, "visao": 0}
        self.velodyne = 1
        self.trajetoriaCalculada = 0
        self.avoidStatic = 0

        # Mapa
        self.p = Pontos()
        self.distanciaObsPerto = 5
        self.a, self.b, self.c, self.abc, self.a1, self.b1, self.c1, self.a1b1c1 = [0,self.p.limiar[0]], [0,self.p.limiar[1]], [0,20], [[0,0,0], [100,100,100]], [0,100], [0,100], [0,100], [[0,0,0],[100,100,100]] # a,b = com capa | a1,b1 = sem capa
        self.newA, self.newB, self.newC, self.newA1, self.newB1, self.newC1, self.newA2, self.newB2, self.newC2 = [], [], [], [], [], [], [], [], []
        self.ABC, self.ABC1, self.ABC2 = [], [], []
        self.countDym, self.checkDym = 0, 0
        self.localX, self.localY, self.localXY = [], [], []

        # Variable Values
        self.rand = 0
        self.countV = 0
        self.anguloGeral = 0.0
        self.altura, self.distNodes = 2.2, 0.5
        self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw = 2, 2, 0, 0
        self.alturaLaser = 0
        self.bateria = {"inicial": 0, "final": 0, "uso": 0, "atual": 0}
        self.bateriaGazebo = {"inicial": 0, "final": 0, "uso": 0, "atual": 0}
        self.memoria = {"inicial": 0, "final": 0}
        self.cpu = {"inicial": 0, "final": 0}
        self.zero = {"rtkx": 0, "rtky": 0, "rtkz": 0}
        self.variaveisLog = {"tt": []}
        self.descontoPosicao = {"x": 0, "y": 0, "z": 0}

        # Trajectory
        self.rotas = {}
        self.rotas["x"], self.rotas["y"], self.rotas["z"], self.rotas["yaw"] = np.array([]), np.array([]), np.array([]), np.array([])
        
        # Values to be Changed by the User
        # self.rotas["x"] = [7.1, 10, 11, 12, 15, 17, 20, 20.1] 
        # self.rotas["y"] = [7.1, 10, 11, 12, 12, 11, 15, 15.1] 
        # # self.rotas["x"] = np.array([7.1]) 
        # # self.rotas["y"] = np.array([7.1])
        # self.rotas["z"] = np.array([self.altura] * len(self.rotas["x"]))
        # self.rotas["yaw"] = np.array([0] * len(self.rotas["x"]))

        # Generate Trajectory
        # _, t, rx, ry = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
        # self.rotas["x"], self.rotas["y"] = rotaToGazebo(rx, ry, self.a, self.b, self.distNodes)
        # self.rotas["z"] = [self.altura] * len(self.rotas["x"])
        # self.rotas["yaw"] = [0] * len(self.rotas["x"])
        # self.variaveisLog["tt"].append(t)

        # To Log
        fileName = "logPathPlanner/comUAV/" + "Velodyne" + str(datetime.now().day) + str(datetime.now().month) + str(datetime.now().hour) + str(datetime.now().minute) + str(datetime.now().second) + ".txt"
        if self.log: 
            print("NOME DO ARQUIVO: " + str(fileName))
            self.f = open(fileName, "a")

        # Times
        self.counts = {"total": 0, "parar": 0, "tempo": 0, "calibrandoTarget": 0}
        self.tempo = {"parar": 0, "takeoff": 6, "land": 6, "wait": 0, "hover": 6, "sec": 2}
        self.contador,self.contador0 = 0, 0
        self.theLast = 0

        self.contadorTotal = time()

        # Start
        self.counts["total"] = time()
        sleeping(t=1)

        # State Machine
        self.status = 2
        self.busy, self.arrived, self.idle = 1, 2, 3

        self.numeroImagem = 0
        
        # Subscribers
        ############# Main
        _ = rospy.Subscriber("/uav1/odometry/odom_local", Odometry, self.callbackMain)
        ############# Localization
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackPosicao)
        # # _ = rospy.Subscriber("/uav1/garmin/range", Range, self.callbackLaser)
        # _ = rospy.Subscriber("/tcpfix", NavSatFix, self.callbackGPSRTK)
        ############# Battery
        # _ = rospy.Subscriber("/uav1/mavros/battery", BatteryState, self.callbackBattery)
        # # # _ = rospy.Subscriber("/battery/status", BatteryState, self.callbackBatteryGazebo) # 18
        _ = rospy.Subscriber("/battery/percent", Int32, self.callbackBatteryGazebo) # 20
        ############# Avoid Obstacle
        # _ = rospy.Subscriber("/uav1/odometry/odom_main_innovation", Odometry, self.callbackDynamic)
        # _ = rospy.Subscriber("/uav1/odometry/gps_local_odom", Odometry, self.callbackStatic)
        ############# Mapping
        # # _ = rospy.Subscriber("/build_map3D", PoseArray, self.callbackBuildMap)
        _ = rospy.Subscriber("/uav1/velodyne/scan", PointCloud2, self.callbackBuildMap3D)
        # _ = rospy.Subscriber("/orb_slam2_rgbd/map_points", PointCloud2, self.callbackBuildMap3DOrbs)
        # _ = rospy.Subscriber("/orbslam/filtered/map_points", PointCloud2, self.callbackBuildMap3DOrbsFiltered)
        # _ = rospy.Subscriber("/hokuyo20/laser/scan", LaserScan, self.callbackHokuyo20)
        # _ = rospy.Subscriber("/hokuyo/laser/scan", LaserScan, self.callbackHokuyo)
        # # _ = rospy.Subscriber("/build_map", PoseArray, self.callbackBuildMap2D)

        # decolagemInicial()
        print("Espere um momento, ja iremos comecar")
        rospy.sleep(1)
        self.unic["SM"] = 1
        self.memoria["inicial"] = memory_usage()
        self.cpu["inicial"] = psutil.cpu_percent()
        set_vio()

    # ---------------------------- Loop :3 ----------------------------------
    def callbackMain(self, odom):
        if self.unic["SM"] == 1 and self.unic["definirRota"] == 1:     
            self.rotinaNormal()

    # ---------------------------- Bateria ----------------------------------
    def callbackBattery(self, bat):
        self.bateria["atual"] = bat.current
        if self.unic["bateria"] == 0:
            self.bateria["inicial"] = bat.current
            self.unic["bateria"] = 1
            self.counts["total"] = time()

        if self.unic["bateria"] == 2:
            self.bateria["final"] = bat.current
            self.bateria["uso"] = self.bateria["final"] - self.bateria["inicial"]
            self.unic["bateria"] = 4

    def callbackBatteryGazebo(self, bat):
        self.bateriaGazebo["atual"] = bat.data
        if self.unic["bateriaGazebo"] == 0:
            self.bateriaGazebo["inicial"] = bat.data
            self.unic["bateriaGazebo"] = 1

        if self.unic["bateriaGazebo"] == 2:
            self.bateriaGazebo["final"] = bat.data
            self.bateriaGazebo["uso"] = self.bateriaGazebo["final"] - self.bateriaGazebo["inicial"]
            self.unic["bateriaGazebo"] = 4

    # ---------------------------- Construir Mapa ----------------------------------
    def callbackLaser(self, alt):
        self.alturaLaser = alt.range

    # ---------------------------- Obstaculo Dinamico ----------------------------------
    def callbackDynamic(self, odom):
        # print("Obstaculo dinamico")
        v1x, v1y, v1z, v1all, v2x, v2y, v2z, v2all = [], [], [], [], [], [], [], []

        ehDym = 0
        obsX, obsY, obsZ = [], [], []
        if self.checkDym:
            for i in range(len(self.ABC)):
                v1, v2, v3 = self.ABC[i], self.ABC1[i], self.ABC2[i]
                if v1 in self.ABC: self.ABC.remove(v1)
                if v1 in self.ABC1: self.ABC1.remove(v1)
                if v1 in self.ABC2: self.ABC2.remove(v1)
                if v2 in self.ABC: self.ABC.remove(v2)
                if v2 in self.ABC1: self.ABC1.remove(v2)
                if v2 in self.ABC2: self.ABC2.remove(v2)
                if v3 in self.ABC: self.ABC.remove(v3)
                if v3 in self.ABC1: self.ABC1.remove(v3)
                if v3 in self.ABC2: self.ABC2.remove(v3)

            s01, s02 = 0, 0
            s11, s12 = 0, 0
            s21, s22 = 0, 0
            for i in self.ABC:
                s01 += i[0]
                s02 += i [1]
            for i in self.ABC1:
                s11 += i[0]
                s12 += i [1]
            for i in self.ABC2:
                s21 += i[0]
                s22 += i [1]
            
            direcao = 0 # 1: esquerda | 2: direita | 3: frente | 4: tras
            if s21 > s11 and s21 > s01:
                direcao = 1
                ehDym = 1
            if s21 < s11 and s21 < s01:
                direcao = 2
                ehDym = 1
            if s22 > s12 and s22 > s02:
                direcao = 3
                ehDym = 1
            if s22 < s12 and s22 < s02:
                direcap = 4
                ehDym = 1

            if ehDym:
                print("OBSTACULO DINAMICO IDENTIFICADO")
                distanciaBase = float("inf")
                for i in self.ABC:
                    value = dist_euclidiana3D(i[0], i[1], i[2], self.currentPosX, self.currentPosY, self.currentPosZ)
                    if value < distanciaBase:
                        pontoColisao = i
                        distanciaBase = value
                        
                for ox, oy, oz in zip(obsX, obsY, obsZ):
                    if colidirTrajetoria3D(self.newA2, self.newB2, self.newC2, self.rotas["x"], self.rotas["y"], self.rotas["z"], self.pos):
                        pontoColisao = [ox, oy, oz]
                        self.rotas["x"] = np.concatenate((self.rotas["x"][:self.pos], [pontoColisao[0]] , self.rotas["x"][self.pos+3:]), axis=0)
                        self.rotas["y"] = np.concatenate((self.rotas["y"][:self.pos], [pontoColisao[1]] , self.rotas["y"][self.pos+3:]), axis=0)
                        self.rotas["z"] = np.concatenate((self.rotas["z"][:self.pos], [pontoColisao[2]] , self.rotas["z"][self.pos+3:]), axis=0)
                        newPoints = generate_curve(self.rotas["x"], self.rotas["y"], self.rotas["z"])
                        self.rotas["x"], self.rotas["y"], self.rotas["z"] = newPoints[:][0], newPoints[:][1], newPoints[:][2]
                        self.rotas["yaw"] = np.array([0] * len(self.rotas["x"]))
                        if pontoColisao in self.abc:
                            self.abc.remove(pontoColisao)
                            self.a, self.b, self.c = [], [], []
                            for value in self.abc:
                                self.a.append(value[0])
                                self.b.append(value[1])
                                self.c.append(value[2])

                    self.a, self.b, self.c = rotaToGazebo3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], self.a, self.b, self.c)
                    ehDym = 0

            self.checkDym = 0

    # ---------------------------- Obstaculo Estatico ----------------------------------
    def callbackStatic(self, odom):
        if ((time() - self.counts["total"]) > 5) and (self.unic["definirRota"] == 1) and (self.avoidStatic == 0 or self.avoidStatic + 50 < self.pos) and (self.pos > 0):
            if abs(self.currentPosX - self.p.xt) > 1 and abs(self.currentPosY - self.p.yt) > 1 and abs(self.currentPosZ - self.p.yt) > 1:
                posicao = self.pos

                if colidirTrajetoria3D(self.a, self.b, self.c, self.rotas["x"], self.rotas["y"], self.rotas["z"], posicao, value=0.3):
                    print("Tem colisao na trajetoria")
                    
                    self.rotas["x"] = np.insert(self.rotas["x"],self.pos+1, self.hover)
                    self.rotas["y"] = np.insert(self.rotas["y"],self.pos+1, self.hover)
                    self.rotas["z"] = np.insert(self.rotas["z"],self.pos+1, self.hover)
                    self.rotas["yaw"] = np.insert(self.rotas["yaw"],self.pos+1, self.hover)
                    print("1------1------1")
                    print(self.currentPosX)
                    print(self.currentPosY)
                    print(self.pos)
                    _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p, pseudox=self.rotas["x"][self.pos+1:], pseudoy=self.rotas["y"][self.pos+1:], pseudoz=self.rotas["z"][self.pos+1:])
                    print("2")
                    rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                    print("Tempo para recalcular foi de: " + str(t))

                    self.rotas["x"] = np.concatenate((self.rotas["x"][:posicao-1], np.clip(rx, 1.1, self.p.limiar[0]-1.1).tolist()), axis=0)#[self.pos:] = rx
                    self.rotas["y"] = np.concatenate((self.rotas["y"][:posicao-1], np.clip(ry, 1.1, self.p.limiar[1]-1.1).tolist()), axis=0)#[self.pos:] = rx
                    self.rotas["z"] = np.concatenate((self.rotas["z"][:posicao-1], np.clip(rz, 0.8, 14).tolist()), axis=0)#[self.pos:] = rx # [self.altura] * len(rx)
                    self.rotas["yaw"] = [self.anguloGeral] * len(self.rotas["x"])

                    self.trajetoriaCalculada = 1

                    self.variaveisLog["tt"].append(t)

                    # print(rx)
                    # print(ry)
                    # print("-------------")
                    print(self.rotas["x"].tolist())
                    print(self.rotas["y"].tolist())
                    print(self.rotas["z"].tolist())
                    self.avoidStatic = self.pos
                    # ax = plt.axes(projection = "3d")
                    # ax.plot3D(self.a, self.b, self.c, 'k.') 
                    # ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                    # ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                    # ax.set_xlim(0,20) 
                    # ax.set_ylim(0,10) 
                    # ax.set_zlim(0,6) 
                    # plt.pause(0.01)
                    # plt.show()
                else:
                    if self.currentPosZ <= 0.4:
                        ax = plt.axes(projection = "3d")
                        ax.plot3D(self.a, self.b, self.c, 'k.') 
                        ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                        ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                        ax.set_xlim(0,20) 
                        ax.set_ylim(0,10) 
                        ax.set_zlim(0,6) 
                        plt.pause(0.01)
                        plt.show()
                    # print("sem colisao")

    # ---------------------------- Altura Laser ----------------------------------
    def callbackBuildMap3D(self, data):
        # ADD TO FIND DYNAMIC OBSTACLES
        ########################################################################################################################################
        if self.velodyne:
            print("here")
            gen = point_cloud2.read_points(data, skip_nans=True)
            int_data = list(gen)

            self.newA, self.newB, self.newC = [], [], []

            # angAbertura = 120
            # randSup = self.rand + math.radians(angAbertura/2) if self.rand + math.radians(angAbertura/2) < math.radians(360) else self.rand + math.radians(angAbertura/2) - math.radians(360)
            # randInf = self.rand - math.radians(angAbertura/2) if self.rand - math.radians(angAbertura/2) > 0 else math.radians(360) + (self.rand - math.radians(angAbertura/2))
            
            for x in int_data:
                if round(x[2] > 0):
                    self.newA.append(round(x[0]))
                    self.newB.append(round(-x[1]))
                    self.newC.append(round(x[2]))

            pl = self.rotationMatrix(-math.pi/4, self.newA, self.newB, self.newC)

            self.newA, self.newB, self.newC = [], [], []
                
            # print("Virar para: " + str(math.degrees(self.rand)))
            # print("Angulo superior: " + str(math.degrees(randSup)))
            # print("Angulo inferior: " + str(math.degrees(randInf)))

            # if self.countV == 7:
            #     self.a, self.b, self.c = [], [], []
            #     self.countV = 0

            for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                dentroRange = 1
                # angPontos = definir_angulo(self.currentPosX, self.currentPosY, i2+3, i1+3)
                # if randSup > self.rand > randInf: # 100 - 190 - 10
                #     if randSup > angPontos > randInf:
                #         dentroRange = 1
                # elif self.rand < randInf and self.rand < randSup: # 50 - 140 - 320
                #     if 0 < angPontos < randSup or math.radians(360) > angPontos > randInf:
                #         dentroRange = 1 
                # elif self.rand < randInf and self.rand < randSup: # 300 - 30 - 210
                #     if 0 < angPontos < randSup or math.radians(360) > angPontos > randInf:
                #         dentroRange = 1

                if dentroRange:
                    if ([round(i1+self.currentPosX), round(i2+self.currentPosY), round(i3)] not in self.abc and round(i2)>1) and round(i2+3) != self.currentPosX and round(i1+3) != self.currentPosY:
                        # self.newA.append(round(i2+3))
                        # self.newB.append(round(i1+3))
                        # self.newC.append(round(i3+1))
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)-1))
                            self.c.append(round(i3))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)-1),round(i3)])

                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)+1))
                            self.c.append(round(i3))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)+1),round(i3)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)-1))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3))
                            self.abc.append([round(i2+round(self.currentPosX)-1),round(i1+round(self.currentPosY)),round(i3)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)+1))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3))
                            self.abc.append([round(i2+round(self.currentPosX)+1),round(i1+round(self.currentPosY)),round(i3)])   



                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+1)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+1))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+1)])

                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+1)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)-1))
                            self.c.append(round(i3+1))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)-1),round(i3+1)])

                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+1)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)+1))
                            self.c.append(round(i3+1))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)+1),round(i3+1)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+1)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)-1))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+1))
                            self.abc.append([round(i2+round(self.currentPosX)-1),round(i1+round(self.currentPosY)),round(i3+1)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+1)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)+1))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+1))
                            self.abc.append([round(i2+round(self.currentPosX)+1),round(i1+round(self.currentPosY)),round(i3+1)])    





                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+2)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+2))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+2)])

                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+2)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)-1))
                            self.c.append(round(i3+2))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)-1),round(i3+2)])

                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+2)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)+1))
                            self.c.append(round(i3+2))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)+1),round(i3+2)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+2)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)-1))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+2))
                            self.abc.append([round(i2+round(self.currentPosX)-1),round(i1+round(self.currentPosY)),round(i3+2)])
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+2)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)+1))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+2))
                            self.abc.append([round(i2+round(self.currentPosX)+1),round(i1+round(self.currentPosY)),round(i3+2)])

                        
                        
                        
                        
                        
                        if [round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+3)] not in self.abc:
                            self.a.append(round(i2+round(self.currentPosX)))
                            self.b.append(round(i1+round(self.currentPosY)))
                            self.c.append(round(i3+3))
                            self.abc.append([round(i2+round(self.currentPosX)),round(i1+round(self.currentPosY)),round(i3+3)])
                      

            # if self.pos < 1:
            #     print(self.currentPosY)
            #     ax = plt.axes(projection = "3d")
            #     ax.plot3D(self.a, self.b, self.c, 'k.') 
            #     ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
            #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
            #     ax.set_xlim(0,20) 
            #     ax.set_ylim(0,10) 
            #     ax.set_zlim(0,6) 
            #     plt.pause(0.01)
            #     plt.show()

            if self.unic["definirRota"] == 0:
                print("Definindo rota")
                _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
                print("rota definida")  
                self.variaveisLog["tt"].append(t)                       
                rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                rx = np.clip(rx, 1.1, self.p.limiar[0]-1.1).tolist()
                ry = np.clip(ry, 1.1, self.p.limiar[1]-1.1).tolist()
                rz = np.clip(rz, 0.8, 14).tolist()
                print(rx)                
                print(ry)                
                print(rz)                
                self.rotas["x"] = np.array(rx)
                self.rotas["y"] = np.array(ry)
                self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
                self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
                # ax = plt.axes(projection = "3d")
                # ax.plot3D(self.a, self.b, self.c, 'k.') 
                # ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                # ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                # ax.set_xlim(0,20) 
                # ax.set_ylim(0,10) 
                # ax.set_zlim(0,6) 
                # plt.pause(0.01)
                # plt.show()
                self.unic["definirRota"] = 1

            self.velodyne = 0

    def callbackBuildMap2D(self, obs):
        buildMapX, buildMapY = [], []
        for value in obs.poses:
            if dist_euclidiana(self.currentPosX, self.currentPosY, value.position.x, value.position.y) < self.distanciaObsPerto:
                buildMapX.append(value.position.x)
                buildMapY.append(value.position.y)

        # a,b = com capa | a1,b1 = sem capa
        self.localX, self.localY, _, _, self.localXY =  laserROS(buildMapX, buildMapY, self.localX, self.localY, self.localXY, tamCapa=0)

    def callbackGPSRTK(self, data):
        if self.unic["gpsrtk"] == 0:
            self.zero["rtkx"] = data.latitude
            self.zero["rtky"] = data.longitude
            self.zero["rtkz"] = data.altitude
        else:
            self.currentPosX = haversine([self.zero["rtkx"], self.zero["rtky"]], [self.zero["rtkx"], data.longitude])*1000
            if data.latitude < self.zero["rtkx"]: self.currentPosX *= -1 
            self.currentPosY = haversine([self.zero["rtkx"], self.zero["rtky"]], [data.latitude, self.zero["rtky"]])*1000
            if data.longitude < self.zero["rtky"]: self.currentPosY *= -1 
            self.currentPosZ = data.altitude - self.zero["rtkz"]

    def rotationMatrix(self, psi0, x1, y1, z1):
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
        return pos_local

    def callbackBuildMap3DOrbsFiltered(self, data):
        if self.velodyne:
            gen = point_cloud2.read_points(data, skip_nans=True)
            int_data = list(gen)

            a, b, c = [], [], []

            for x in int_data:
                if round(x[2] > -1):
                    a.append(round(x[0]))
                    b.append(round(-x[1]))
                    c.append(round(x[2]))

            pl = self.rotationMatrix(self.currentPosYaw, a, b, c)

            a, b, c = [], [], []

            for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                if [round(i1), round(i2), round(i3)] not in self.abc:
                    a.append(round(i2+3))
                    b.append(round(i1+1))
                    c.append(round(i3+2))
                    self.a.append(round(i2+3))
                    self.b.append(round(i1+1))
                    self.c.append(round(i3+2))
                    self.abc.append([round(i1), round(i2), round(i3)])

                # if self.unic["definirRota"] == 0:
                #     _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
                #     rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                #     self.rotas["x"] = np.array(rx)
                #     self.rotas["y"] = np.array(ry)
                #     self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
                #     self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
                #     print("rota definida")
                #     self.variaveisLog["tt"].append(t)
                #     ax = plt.axes(projection = "3d")
                #     ax.plot3D(self.a, self.b, self.c, 'k.') 
                #     ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                #     ax.set_xlim(0,21) 
                #     ax.set_ylim(0,11) 
                #     ax.set_zlim(0,4) 
                #     plt.pause(0.01)
                #     plt.show()
                #     self.unic["definirRota"] = 1

            self.velodyne = 0

    def callbackBuildMap3DOrbs(self, data):
        if self.velodyne:
            gen = point_cloud2.read_points(data, skip_nans=True)
            
            int_data = list(gen)

            a, b, c = [], [], []

            for x in int_data:
                if round(x[2] > 0):
                    a.append(round(x[0]))
                    b.append(round(-x[1]))
                    c.append(round(x[2]))

            pl = self.rotationMatrix(-math.pi/4, a, b, c)

            a, b, c = [], [], []

            for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                if [round(i1), round(i2), round(i3)] not in self.abc:
                    a.append(round(i2+3))
                    b.append(round(i1+3))
                    c.append(round(i3+1))
                    self.a.append(round(i2+3))
                    self.b.append(round(i1+3))
                    self.c.append(round(i3+1))
                    self.abc.append([round(i1), round(i2), round(i3)])

            if self.currentPosX > 15 and self.currentPosY > 12 and self.currentPosZ > 1 and self.currentPosX < 20 and self.currentPosY < 15:
                ax = plt.axes(projection = "3d")
                ax.plot3D(self.a, self.b, self.c, 'k.') 
                ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                ax.set_xlim(-10,100) 
                ax.set_ylim(-10,100) 
                ax.set_zlim(0,4) 
                plt.pause(0.01)
                plt.show()

            self.velodyne = 0

    # def callbackHokuyo(self, data):
    #     for index, value in enumerate(data.ranges[240:720-240]):
    #         if value < 15:
    #             aux = np.linspace(90, 0, 720-480)
    #             theta = aux[index]
    #             if self.countDym == 0:
    #                 self.newA.append(np.cos(np.radians(theta)) * value)
    #                 self.newB.append(np.sin(np.radians(theta)) * value)
    #                 self.newC.append(self.currentPosZ)
    #             elif self.countDym == 1:
    #                 self.newA1.append(np.cos(np.radians(theta)) * value)
    #                 self.newB1.append(np.sin(np.radians(theta)) * value)
    #                 self.newC1.append(self.currentPosZ)
    #             else:
    #                 self.newA2.append(np.cos(np.radians(theta)) * value)
    #                 self.newB2.append(np.sin(np.radians(theta)) * value)
    #                 self.newC2.append(self.currentPosZ)

    #     if self.countDym == 0:
    #         pl = rotationMatrix3D(self.currentPosYaw, self.newA, self.newB, self.newC, "yaw")
    #         self.newA = pl[0] 
    #         self.newB = pl[1] 
    #         self.newC = pl[2] 
    #     elif self.countDym == 1:
    #         pl = rotationMatrix3D(self.currentPosYaw, self.newA1, self.newB1, self.newC1, "yaw")
    #         self.newA1 = pl[0] 
    #         self.newB1 = pl[1] 
    #         self.newC1 = pl[2] 
    #     else:
    #         pl = rotationMatrix3D(self.currentPosYaw, self.newA2, self.newB2, self.newC2, "yaw")
    #         self.newA2 = pl[0] 
    #         self.newB2 = pl[1] 
    #         self.newC2 = pl[2] 
    #     self.countDym += 1

    #     if self.countDym == 3:
    #         for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
    #             if [round(i1), round(i2), round(i3)] not in self.abc:
    #                 self.a.append(round(i1))
    #                 self.b.append(round(i2))
    #                 self.c.append(round(i3))
    #                 self.abc.append([round(i1), round(i2), round(i3)])
            
    #         self.countDym = 0
    #         self.checkDym = 1
    #         self.newA, self.newB, self.newC, self.newA1, self.newB1, self.newC1, self.newA2, self.newB2, self.newC2 = [], [], [], [], [], [], [], [], []

    def callbackHokuyo(self, data):
        x, y, z = [], [], []
        for index, value in enumerate(data.ranges[120:600]):
            if value < 30:
                aux = np.linspace(180, 0, 600-120)
                theta = aux[index]
                if dist_euclidiana(np.cos(np.radians(theta)) * value, np.sin(np.radians(theta)) * value, self.currentPosX, self.currentPosY) > 0.3:
                    x.append(np.cos(np.radians(theta)) * value)
                    y.append(np.sin(np.radians(theta)) * value)
                    z.append(self.currentPosZ)

        # for index, value in enumerate(data.ranges):
        #     if value < 30:
        #         aux = np.linspace(270, 0, 720)
        #         theta = aux[index]
        #         x.append(np.cos(np.radians(theta)) * value)
        #         y.append(np.sin(np.radians(theta)) * value)
        #         z.append(self.currentPosZ)

        # pl = rotationMatrix3D(np.radians(135), x, y, z, "yaw")
        pl = rotationMatrix3D(np.radians(90), x, y, z, "yaw")
        for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
            if [round(i1), round(i2), round(i3)] not in self.abc:
                self.a.append(round(i1))
                self.b.append(round(i2) + 6)
                self.c.append(round(i3))
                self.abc.append([round(i1), round(i2), round(i3)])

        print(time() - self.contadorTotal)
        if 300 > time() - self.contadorTotal > 20:
                print(self.currentPosY)
                ax = plt.axes(projection = "3d")
                ax.plot3D(self.a, self.b, self.c, 'k.') 
                ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                ax.set_xlim(0,20) 
                ax.set_ylim(0,10) 
                ax.set_zlim(0,4) 
                plt.pause(0.01)
                plt.show()

    def callbackHokuyo20(self, data):
        a, b, c = [], [], []
        for index, value in enumerate(data.ranges[120:600]):
            if value < 30:
                aux = np.linspace(180, 0, 600-120)
                theta = aux[index]
                a.append(np.cos(np.radians(theta)) * value)
                b.append(np.sin(np.radians(theta)) * value)
                c.append(self.currentPosZ)

        # for index, value in enumerate(data.ranges):
        #     if value < 30:
        #         aux = np.linspace(270, 0, 720)
        #         theta = aux[index]
        #         a.append(np.cos(np.radians(theta)) * value)
        #         b.append(np.sin(np.radians(theta)) * value)
        #         c.append(self.currentPosZ)

        pl = rotationMatrix3D(math.radians(-10), a, b, c, "pitch")
        # pl = rotationMatrix3D(math.radians(140), pl[0], pl[1], pl[2], "yaw")
        pl = rotationMatrix3D(math.radians(97), pl[0], pl[1], pl[2], "yaw")
        for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
            if [round(i1), round(i2), round(i3)] not in self.abc:
                self.a.append(round(i1))
                self.b.append(round(i2) + 8)
                self.c.append(round(i3))
                self.abc.append([round(i1), round(i2), round(i3)])

        # if self.currentPosX > 15 and self.currentPosY > 12 and self.currentPosZ > 1 and self.currentPosX < 20 and self.currentPosY < 15:
        print(time() - self.contadorTotal)
        if 400 > time() - self.contadorTotal > 20:
                ax = plt.axes(projection = "3d")
                ax.plot3D(self.a, self.b, self.c, 'k.') 
                ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                ax.set_xlim(0,20) 
                ax.set_ylim(0,10) 
                ax.set_zlim(0,7) 
                plt.pause(0.01)
                plt.show()

    def callbackBuildMap(self, obs):
        buildMapX, buildMapY, buildMapZ = [], [], []

        for value in obs.poses:
            # if 0 < value.position.x < 50 and 0 < value.position.y < 50:
            buildMapX.append(value.position.x)
            buildMapY.append(value.position.y)
            buildMapZ.append(value.position.z)
        
        if len(buildMapX) > 5: # if self.trocaYaw:
            # pl = self.rotationMatrix(self.currentPosYaw - math.pi/4, buildMapX, buildMapY, buildMapZ)
            # for v1, v2, v3 in zip(pl[0], pl[1], pl[2]):
            for v1, v2, v3 in zip(buildMapX, buildMapY, buildMapZ):
                self.a.append(v1)
                self.b.append(v2)
                self.c.append(v3)

        print(time() - self.counts["total"])
        if round(time() - self.counts["total"])%1==0 and 24 > time() - self.counts["total"] > 15:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            
            ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
            ax.plot3D(self.a, self.b, self.c,".k")
            ax.plot3D(buildMapX, buildMapY, buildMapZ,".y")
            ax.set_title(self.currentPosYaw)
            ax.set_xlabel("x (m)" + str(self.currentPosX))
            ax.set_ylabel("y (m)" + str(self.currentPosY))
            ax.set_zlabel("z (m)" + str(self.currentPosZ))
            ax.set_xlim(-50, 100)
            ax.set_ylim(-50, 100)
            ax.set_zlim(-3, 7)
            ax.view_init(73, -159)
            # plt.savefig("orbsslam" + str(self.contador) + ".png")
            # plt.pause(0.01)
            self.contador += 1
            plt.show()
        
        # if self.unic["definirRota"] == 0:
        #     _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
        #     # rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
        #     self.rotas["x"] = rx
        #     self.rotas["y"] = ry
        #     self.rotas["z"] = rz # [self.altura] * len(rx)
        #     self.rotas["yaw"] = [self.anguloGeral] * len(rx)
        #     print("rota definida")
        #     self.unic["definirRota"] = 1

    # ---------------------------- Onde o UAV ta ----------------------------------
    def callbackPosicao(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        if yaw < 0: yaw = math.pi + yaw
        
        # if self.unic["pos"] == 0:
        #     self.descontoPosicao["x"] = odom.pose.pose.position.x
        #     self.descontoPosicao["y"] = odom.pose.pose.position.y
        #     self.unic["pos"] = 1
        # else:
        #     if self.unic["gpsrtk"] == 0:
        #         self.currentPosX = odom.pose.pose.position.x - self.descontoPosicao["x"] + 2
        #         self.currentPosY = odom.pose.pose.position.y - self.descontoPosicao["y"] + 4
        #         self.currentPosZ = odom.pose.pose.position.z - self.descontoPosicao["z"]

        if self.unic["gpsrtk"] == 0:
            self.currentPosX = odom.pose.pose.position.x 
            self.currentPosY = odom.pose.pose.position.y
            self.currentPosZ = odom.pose.pose.position.z 
        
        self.currentPosYaw = yaw

    # ---------------------------- State Machine ----------------------------------
    def rotinaNormal(self):
        if self.unic["SM"] == 1: # Se tiver funcionando
            # ---------------- Decidindo sua vida ---------------------
            if self.status == self.arrived:
                self.unic["idle"] = 0

                # ---------------- Cabou ---------------------
                # if self.rotas["x"][-1] == 36 and self.rotas["x"][-1] == 32:
                if ((abs(self.currentPosX - self.p.xt) < 0.2 and abs(self.currentPosY - self.p.yt) < 0.2 and abs(self.currentPosZ - self.p.yt) < 0.6 or (self.pos == len(self.rotas["x"])))) and self.unic["definirRota"]==1: #
                # if self.pos == len(self.rotas["x"]):
                    if self.unic["bateria"]!= 4: self.unic["bateria"] = 2
                    if self.unic["bateriaGazebo"]!= 4: self.unic["bateriaGazebo"] = 2

                    self.memoria["final"] = memory_usage()
                    self.cpu["final"] = psutil.cpu_percent()

                    # print(self.rotas["x"].tolist())
                    # print(self.rotas["y"].tolist())
                    # print(self.rotas["z"].tolist())
                    print("Uso da bateria:")
                    print(self.bateria)
                    print("Uso da bateria Gazebo:")
                    print(self.bateriaGazebo)
                    print("CPU:")
                    print(self.cpu)
                    print("Memoria:")
                    print(self.memoria)
                    print("Comprimento: " + str(distancia_rota(self.rotas["x"], self.rotas["y"])))
                    try:
                        print("Media do tempo: " + str(stc.mean(self.variaveisLog["tt"])))
                        print("Variancia do tempo: " + str(stc.variance(self.variaveisLog["tt"])))
                        print("Desvio padrao do tempo: " + str(stc.stdev(self.variaveisLog["tt"])))
                    except:
                        pass
                    print("Maior do tempo: " + str(max(self.variaveisLog["tt"])))
                    print("Menor do tempo: " + str(min(self.variaveisLog["tt"])))
                    print("Tempo de voo: " + str(time() - self.counts["total"]))

                    print(self.rotas["x"].tolist())
                    print(self.rotas["y"].tolist())
                    print(self.rotas["z"].tolist())

                    if self.log:
                        self.f.write("Uso da bateria:")
                        self.f.write(self.bateria)
                        self.f.write("Uso da bateria Gazebo:")
                        self.f.write(self.bateriaGazebo)
                        self.f.write("CPU:")
                        self.f.write(self.cpu)
                        self.f.write("Memoria:")
                        self.f.write(self.memoria)
                        self.f.write("Comprimento: " + str(distancia_rota(self.rotas["x"], self.rotas["y"])))
                        
                        try:
                            self.f.write("Media do tempo: " + str(stc.mean(self.variaveisLog["tt"])))
                            self.f.write("Variancia do tempo: " + str(stc.variance(self.variaveisLog["tt"])))
                            self.f.write("Desvio padrao do tempo: " + str(stc.stdev(self.variaveisLog["tt"])))
                            self.f.write("Maior do tempo: " + str(max(self.variaveisLog["tt"])))
                            self.f.write("Menor do tempo: " + str(min(self.variaveisLog["tt"])))
                        except:
                            self.f.write("Tempo algoritmo: " + str(self.variaveisLog["tt"]))
                            pass
                        
                        self.f.write("Tempo de voo: " + str(time() - self.counts["total"]))

                        self.f.write("Trajetoria X")
                        for value in self.rotas["x"]:
                            self.f.write(str(value))
                            self.f.write(", ")
                        self.f.write("\n\n")
                        print("Trajetoria Y")
                        for value in self.rotas["y"]:
                            self.f.write(str(value))
                            self.f.write(", ")
                        self.f.write("\n\n")
                        print("Trajetoria Z")
                        for value in self.rotas["z"]:
                            self.f.write(str(value))
                            self.f.write(", ")
                        self.f.write("\n\n")

                    print("Acabou a missao")
                    self.unic["SM"] = 0
                    exit()

                try:
                    # ---------------- Flag Start/End ---------------------
                    if self.rotas["x"][self.pos] == self.land:
                        self.unic["print"] = logStateMachine("Landing", self.unic["print"])
                        self.tempo["wait"] = self.tempo["land"]
                        land()

                    elif self.rotas["x"][self.pos] == self.takeoff:
                        self.unic["print"] = logStateMachine("Take off", self.unic["print"])
                        self.tempo["wait"] = self.tempo["takeoff"]
                        takeoff()

                    # ---------------- Flag Tempo ---------------------
                    elif self.rotas["x"][self.pos] == self.hover:
                        self.unic["print"], self.unic["hover"] = logStateMachine("Hover", self.unic["print"], self.unic["hover"])
                        self.tempo["wait"] = self.tempo["hover"]

                    elif self.rotas["x"][self.pos] == self.sec:
                        self.unic["print"], self.unic["sec"] = logStateMachine("Wait a second", self.unic["print"], self.unic["sec"])
                        self.tempo["wait"] = self.tempo["sec"]

                    elif self.rotas["x"][self.pos] == self.esperarCamera:
                        self.unic["print"], self.unic["esperarCamera"] = logStateMachine("Waiting Camera", self.unic["print"], self.unic["esperarCamera"])
                        self.tempo["wait"] = self.tempo["esperarCamera"]

                    # ---------------- Flag Visao ---------------------
                    elif self.rotas["x"][self.pos] == self.calibrarTarget:
                        self.unic["print"] = logStateMachine("Calibrating Target", self.unic["print"])
                        self.tempo["wait"] = self.tempo["calibrarTarget"]
                        self.unic["calibrarTarget"] = 1

                    # ---------------- Puedo Caminar ---------------------
                    else:
                        print("Estou em")
                        print(str(round(self.currentPosX, 2)) + " - " + str(round(self.currentPosY, 2)) + " - " + str(round(self.currentPosZ, 2)))
                        print("Indo para")
                        print(str(round(self.rotas["x"][self.pos], 2)) + " - " + str(round(self.rotas["y"][self.pos], 2)) + " - " + str(round(self.rotas["z"][self.pos], 2)))
                        self.rand = rotacionar(self.currentPosX, self.currentPosY, self.rotas["x"][self.pos], self.rotas["y"][self.pos])
                        self.unic["print"], self.unic["andar"] = logStateMachine("Walking", self.unic["print"], self.unic["andar"])
                        acrX = 0#-0.15 if self.rotas["x"][self.pos-1] > self.rotas["x"][self.pos] else 0.15
                        acrY = 0#-0.15 if self.rotas["y"][self.pos-1] > self.rotas["y"][self.pos] else 0.15
                        acrZ = -0.5 if self.rotas["z"][self.pos-1] > self.rotas["z"][self.pos] else 1
                        # if self.rotas["z"][self.pos-1] - self.rotas["z"][self.pos] < -0.3: 
                        #     acrZ = -1
                        # elif self.rotas["z"][self.pos-1] - self.rotas["z"][self.pos] > 0.3 :
                        #     acrZ = 1.5
                        # else:
                        #     acrZ = 0
                        if self.rotas["z"][self.pos] + acrZ < 0.5:
                            acrZ = (self.rotas["z"][self.pos] - 0.5) * -1
                        self.tempo["wait"] = andarGlobal(self.rotas["x"][self.pos] + acrX, self.rotas["y"][self.pos] + acrY, self.rotas["z"][self.pos] + acrZ, self.rotas["yaw"][self.pos], self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw, rotacao=False)
                        self.velodyne = 1
                        self.countV += 1
                except:
                    pass

                if self.unic["SM"] == 1:
                    self.counts["tempo"] = time()
                    self.status = self.busy
            
            # ---------------- UAV ocupado ---------------------
            if self.status == self.busy:
                self.unic["print"] = 0
                self.unic["busy"] = logStateMachine("I am busy", self.unic["busy"])
                
                if self.unic["hover"]:
                    if time() - self.counts["tempo"] > self.tempo["wait"] - 1 and self.trajetoriaCalculada==0 and self.addRota==0:
                        if abs(self.currentPosX - self.p.xt) > 0.5 and abs(self.currentPosY - self.p.yt) > 0.5 and abs(self.currentPosZ - self.p.yt) > 0.8: 
                            self.rotas["x"] = np.insert(self.rotas["x"],self.pos+1, self.hover)
                            self.rotas["y"] = np.insert(self.rotas["y"],self.pos+1, self.hover)
                            self.rotas["z"] = np.insert(self.rotas["z"],self.pos+1, self.hover)
                            self.rotas["yaw"] = np.insert(self.rotas["yaw"],self.pos+1, self.hover)
                            self.addRota = 1
                    
                        if time() - self.counts["tempo"] > self.tempo["wait"] and self.trajetoriaCalculada: 
                            self.trajetoriaCalculada = 0

                if time() - self.counts["tempo"] > self.tempo["wait"]: 
                    self.addRota = 0
                    self.status = self.idle
                    self.counts["parar"] = time()

            # ---------------- UAV mudando de estado ---------------------
            if self.status == self.idle:
                if self.unic["hover"] == 1: self.unic["hover"] = 0
                if self.unic["sec"] == 1: self.unic["sec"] = 0

                self.unic["busy"] = 0
                self.unic["idle"] = logStateMachine("Idle", self.unic["idle"])
                if time() - self.counts["parar"] > self.tempo["parar"]: 
                    print(self.pos)
                    print(len(self.rotas["x"]))
                    self.status = self.arrived
                    self.pos += 1            

def main():
    rospy.init_node("Planejador")
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()