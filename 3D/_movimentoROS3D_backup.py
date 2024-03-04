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
# from classic import Astar_test as alg
# from classic import rrt_connect_3d as alg
from classic import opt_bi_a_star as alg

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

from std_msgs.msg import String, Int32
from sensor_msgs.msg import BatteryState
from mrs_msgs.msg import FuturePoint
from mrs_msgs.msg import FutureTrajectory

from datetime import datetime
import statistics as stc
from haversine import haversine
from sys import exit
import psutil

from utilsUAV import *

class globalPlanner:
    def __init__(self):
        self.memoriaDoPc = 16000 # MB
        self.processadorDoPc = 2800 # MHz

        ph = [[[0.0, 10.0], [0.0, 0.0]], [[0.0, 10.0], [10.0, 10.0]], [[6.5, 8.5], [2.0, 2.0]], [[2.0, 4.5], [8.5, 8.5]], [[2.0, 6.5], [4.0, 4.0]], [[3.5, 5.0], [5.2, 5.2]], [[5.0, 6.5], [6.4, 6.4]], [[8.0, 9.0], [8.0, 8.0]], [[6.5, 8.0], [5.2, 5.2]], [[2.0, 4.5], [2.0, 2.0]]]
        pv = [[[10.0, 10.0], [0.0, 10.0]], [[0.0, 0.0], [0.0, 10.0]], [[4.5, 4.5], [0.0, 2.0]], [[2.0, 2.0], [4.0, 8.5]], [[3.5, 3.5], [4.0, 5.2]], [[5.0, 5.0], [5.2, 6.4]], [[6.5, 6.5], [6.4, 8.7]], [[8.0, 8.0], [8.0, 10.0]], [[9.0, 9.0], [5.2, 8.0]], [[6.5, 6.5], [2.0, 5.2]], [[8.0, 8.0], [3.5, 5.2]]]

        self.xx, self.yy = [], []
        for i in range(len(ph)):
            for j in range(int(ph[i][0][0]*10.0), int(ph[i][0][1]*10.0), 1):
                self.xx.append(j)
                self.yy.append(int(ph[i][1][0]*10.0))

        for i in range(len(pv)):
            for j in range(int(pv[i][1][0]*10.0), int(pv[i][1][1]*10.0), 1):
                self.xx.append(int(pv[i][0][0]*10.0))
                self.yy.append(j)

        # Tags
        self.land, self.takeoff, self.hover, self.sec, self.garraOn, self.esperarCamera, self.garraOff, self.calibrarTarget = 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000

        # Flags
        self.addRota = 0
        self.log = 0
        self.pos = 0
        self.goToHome = 0
        self.letra = ""
        self.unic = {"SM": 0, "busy": 0, "print": 0, "hover": 0, "definirRota": 0, "sec": 0, "andar": 0, "bateria": 0, "bateriaGazebo": 0, "gpsrtk": 0}
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
        self.anguloGeral = 0.0
        self.altura, self.distNodes = 2.2, 1
        self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw = 2, 2, 0, 0
        self.alturaLaser = 0
        self.bateria = {"inicial": 0, "final": 0, "uso": 0, "atual": 0}
        self.bateriaGazebo = {"inicial": 0, "final": 0, "uso": 0, "atual": 0}
        self.memoria = {"inicial": 0, "final": 0}
        self.cpu = {"inicial": 0, "final": 0}
        self.zero = {"rtkx": 0, "rtky": 0, "rtkz": 0}
        self.variaveisLog = {"tt": []}
        self.knownEnvironment = 0
        self.controller = 1 # Use 1 to MPC and 0 to cmd_vel

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
        
        # To Log
        # fileName = "logPathPlanner/comUAV/" + "RRT" + str(datetime.now().day) + str(datetime.now().month) + str(datetime.now().hour) + str(datetime.now().minute) + str(datetime.now().second) + ".txt"
        fileName = "logPathPlanner/comUAV/" + "Velodynee" + str(datetime.now().day) + str(datetime.now().month) + str(datetime.now().hour) + str(datetime.now().minute) + str(datetime.now().second) + ".txt"
        if self.log: 
            print("NOME DO ARQUIVO: " + str(fileName))
            self.f = open(fileName, "a")
        # self.f = open(fileName, "a")
        # self.variaveisLog["tt"].append(t)

        # Adjust Trajectory
        # self.rotas["x"], self.rotas["y"] = rotaToGazebo(rx, ry, self.a, self.b, self.distNodes)
        # self.rotas["z"] = [self.altura] * len(self.rotas["x"])
        # self.rotas["yaw"] = [0] * len(self.rotas["x"])

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
        
        drone_number = 1
        # Subscribers
        ############# Main
        _ = rospy.Subscriber(f"/uav{drone_number}/odometry/odom_local", Odometry, self.callbackMain)
        ############# Localization
        _ = rospy.Subscriber(f"/uav{drone_number}/odometry/odom_main", Odometry, self.callbackPosicao)
        ############# Battery
        _ = rospy.Subscriber("/battery/percent", Int32, self.callbackBatteryGazebo) # 20
        ############# Avoid Obstacle
        # _ = rospy.Subscriber("/uav1/odometry/odom_main_innovation", Odometry, self.callbackDynamic)
        _ = rospy.Subscriber(f"/uav{drone_number}/odometry/gps_local_odom", Odometry, self.callbackStatic)
        ############# Mapping
        # _ = rospy.Subscriber(f"/uav{drone_number}/velodyne/scan", PointCloud2, self.callbackBuildMap3D)
        _ = rospy.Subscriber(f"/occupied_cells_vis_array", MarkerArray, self.callbackBuildMap3D_octomap)

        for i in range(2,21):
            _ = rospy.Subscriber('/uav'+str(i)+'/control_manager/mpc_tracker/predicted_trajectory', FutureTrajectory, self.callbackDynamicVision, callback_args=i)

        # decolagemInicial()
        print("Espere um momento, ja iremos comecar")
        rospy.sleep(1)
        self.unic["SM"] = 1
        self.memoria["inicial"] = memory_usage()
        self.cpu["inicial"] = psutil.cpu_percent()
        # set_vio()

    # ---------------------------- Loop :3 ----------------------------------
    def callbackMain(self, odom):
        if self.unic["SM"] == 1 and self.unic["definirRota"] == 1: 
            self.rotinaNormal()

    
    def callbackDynamicVision(self, data, args):
        uav_traj_sample = {
            'x': self.rotas["x"][self.pos:self.pos+int(len(self.rotas["x"]) * 0.4)], 
            'y': self.rotas["y"][self.pos:self.pos+int(len(self.rotas["x"]) * 0.4)], 
            'z': self.rotas["z"][self.pos:self.pos+int(len(self.rotas["x"]) * 0.4)]
        }
        collision_distance = 1.0

        n_obs_p = len(data.points)
        n_traf_p = len(uav_traj_sample['x'])
        collision = False
        start_time = time()
        i = 0
        p = [0,0,0]
        while i< n_traf_p and collision == False :
            j = 0
            while j< n_obs_p and collision == False :
                d_x = uav_traj_sample['x'][i] - (data.points[j].x )#- 465710.758973)# + self.posx)
                d_y = uav_traj_sample['y'][i] - (data.points[j].y )#- 5249465.43086)# + self.posy)
                d_z = uav_traj_sample['z'][i] - (data.points[j].z )

                if(sqrt(pow(d_x, 2)+ pow(d_y, 2) + pow(d_z, 2)) < collision_distance):
                    collision = True
                    #p = [collision_distance.points[j].x, collision_distance.points[j].y, collision_distance.points[j].z]
                    p = [data.points[j].x ,
                        data.points[j].y ,
                        data.points[j].z ]
                j+=1
            i+=1

        print("--- %s seconds ---" % (time() - start_time))
        if collision:
            print(args)
            print(p)

            distanciaBase = float("inf")
            for i in self.ABC:
                value = dist_euclidiana3D(i[0], i[1], i[2], self.currentPosX, self.currentPosY, self.currentPosZ)
                if value < distanciaBase:
                    pontoColisao = i
                    distanciaBase = value
                    
            # for ox, oy, oz in zip(obsX, obsY, obsZ):
                # if colidirTrajetoria3D(self.newA2, self.newB2, self.newC2, self.rotas["x"], self.rotas["y"], self.rotas["z"], self.pos):
            pontoColisao = [p[0], p[1], p[2]]
            self.rotas["x"] = np.concatenate((self.rotas["x"][:self.pos], [pontoColisao[0]] , self.rotas["x"][self.pos+3:]), axis=0)
            self.rotas["y"] = np.concatenate((self.rotas["y"][:self.pos], [pontoColisao[1]] , self.rotas["y"][self.pos+3:]), axis=0)
            self.rotas["z"] = np.concatenate((self.rotas["z"][:self.pos], [pontoColisao[2]] , self.rotas["z"][self.pos+3:]), axis=0)
            newPoints = generate_curve(self.rotas["x"], self.rotas["y"], self.rotas["z"])
            self.rotas["x"], self.rotas["y"], self.rotas["z"] = newPoints[:][0], newPoints[:][1], newPoints[:][2]
            self.rotas["yaw"] = np.array([np.pi/2] * len(self.rotas["x"]))

            self.rotas["x"], self.rotas["y"], self.rotas["z"] = rotaToGazebo3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], self.a, self.b, self.c)

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
            print(bat.data)
            self.bateriaGazebo["final"] = bat.data
            self.bateriaGazebo["uso"] = self.bateriaGazebo["final"] - self.bateriaGazebo["inicial"]
            # self.unic["bateriaGazebo"] = 4

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
                        self.rotas["yaw"] = np.array([np.pi/2] * len(self.rotas["x"]))
                        if pontoColisao in self.abc:
                            self.abc.remove(pontoColisao)
                            self.a.remove(pontoColisao[0])
                            self.b.remove(pontoColisao[1])
                            self.c.remove(pontoColisao[2])
                            # self.a, self.b, self.c = [], [], []
                            # for value in self.abc:
                            #     self.a.append(value[0])
                            #     self.b.append(value[1])
                            #     self.c.append(value[2])

                    self.rotas["x"], self.rotas["y"], self.rotas["z"] = rotaToGazebo3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], self.a, self.b, self.c)
                    ehDym = 0

            self.checkDym = 0


    # ---------------------------- Obstaculo Estatico ----------------------------------
    def callbackStatic(self, odom):
        # checar se tem colisao com obstaculo estatico
        # atualziar trajetoria

        # print('CHECK VARIABLES')
        # print((time() - self.counts["total"]) > 5)
        # print(self.unic["definirRota"] == 1)
        # print(self.avoidStatic == 0 or self.avoidStatic + 6 < self.pos)
        # print(self.avoidStatic + 6 < self.pos)
        # print(self.avoidStatic == 0)
        # print(self.pos > 0)
        # print(((time() - self.counts["total"]) > 5) and (self.unic["definirRota"] == 1) and (self.avoidStatic == 0 or self.avoidStatic + 6 < self.pos) and (self.pos > 0))
        # print(colidirTrajetoria3D(self.a, self.b, self.c, self.rotas["x"], self.rotas["y"], self.rotas["z"], self.pos, value=0.3))
        
        if ((time() - self.counts["total"]) > 5) and (self.unic["definirRota"] == 1) and (self.avoidStatic == 0 or self.avoidStatic + 6 < self.pos) and (self.pos > 0):
            if abs(self.currentPosX - self.p.xt) > 4 and abs(self.currentPosY - self.p.yt) > 3 and abs(self.currentPosZ - self.p.yt) > 1:
                posicao = self.pos
                # tamanho = len(self.rotas["x"])
                # print("VAI CHECAR COLISAO")
                if colidirTrajetoria3D(self.a, self.b, self.c, self.rotas["x"], self.rotas["y"], self.rotas["z"], posicao, value=0.6):
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
                    print('STATIC COLLISION')
                    print('rx -------------------')
                    print(rx)
                    print('ry -------------------')
                    print(ry)
                    print("2")
                    rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                    print("Tempo para recalcular foi de: " + str(t))

                    self.rotas["x"] = np.concatenate((self.rotas["x"][:posicao-1], np.clip(rx, 0.7, 19.3).tolist()), axis=0)#[self.pos:] = rx
                    self.rotas["y"] = np.concatenate((self.rotas["y"][:posicao-1], np.clip(ry, 0.7, 9.3).tolist()), axis=0)#[self.pos:] = rx
                    self.rotas["z"] = np.concatenate((self.rotas["z"][:posicao-1], np.clip(rz, 0.8, 14).tolist()), axis=0)#[self.pos:] = rx # [self.altura] * len(rx)
                    self.rotas["yaw"] = [self.anguloGeral] * len(self.rotas["x"])
                    # self.pos = posicao + ((self.pos - posicao) * 2)
                    self.trajetoriaCalculada = 1
                    # print(self.rotas["x"])
                    self.variaveisLog["tt"].append(t)
                    # self.counts["total"] = time()
                    print(rx)
                    print(ry)
                    print("-------------")
                    print(self.rotas["x"])
                    print(self.rotas["y"])
                    print(self.rotas["z"])
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
                # else:
                #     print("not")

    # ---------------------------- Altura Laser ----------------------------------
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
    

    def callbackHokuyo(self, data):
        for index, value in enumerate(data.ranges[240:720-240]):
            if value < 15:
                aux = np.linspace(90, 0, 720-480)
                theta = aux[index]
                if self.countDym == 0:
                    self.newA.append(np.cos(np.radians(theta)) * value)
                    self.newB.append(np.sin(np.radians(theta)) * value)
                    self.newC.append(self.currentPosZ)
                elif self.countDym == 1:
                    self.newA1.append(np.cos(np.radians(theta)) * value)
                    self.newB1.append(np.sin(np.radians(theta)) * value)
                    self.newC1.append(self.currentPosZ)
                else:
                    self.newA2.append(np.cos(np.radians(theta)) * value)
                    self.newB2.append(np.sin(np.radians(theta)) * value)
                    self.newC2.append(self.currentPosZ)

        if self.countDym == 0:
            pl = rotationMatrix3D(self.currentPosYaw, self.newA, self.newB, self.newC, "yaw")
            self.newA = pl[0] 
            self.newB = pl[1] 
            self.newC = pl[2] 
        elif self.countDym == 1:
            pl = rotationMatrix3D(self.currentPosYaw, self.newA1, self.newB1, self.newC1, "yaw")
            self.newA1 = pl[0] 
            self.newB1 = pl[1] 
            self.newC1 = pl[2] 
        else:
            pl = rotationMatrix3D(self.currentPosYaw, self.newA2, self.newB2, self.newC2, "yaw")
            self.newA2 = pl[0] 
            self.newB2 = pl[1] 
            self.newC2 = pl[2] 
        self.countDym += 1

        if self.countDym == 3:
            for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                if [round(i1), round(i2), round(i3)] not in self.abc:
                    self.a.append(round(i1))
                    self.b.append(round(i2))
                    self.c.append(round(i3))
                    self.abc.append([round(i1), round(i2), round(i3)])
            
            self.countDym = 0
            self.checkDym = 1
            self.newA, self.newB, self.newC, self.newA1, self.newB1, self.newC1, self.newA2, self.newB2, self.newC2 = [], [], [], [], [], [], [], [], []


    def callbackBuildMap3D(self, data):
        if self.velodyne:
            if not self.knownEnvironment: 
                gen = point_cloud2.read_points(data, skip_nans=True)
                int_data = list(gen)

                self.newA, self.newB, self.newC = [], [], []

                for x in int_data:
                    if round(x[2] >= 0):
                        self.newA.append(round(x[0]))
                        self.newB.append(round(x[1]))
                        self.newC.append(round(x[2]))
                        # self.newA.append(x[0])
                        # self.newB.append(x[1])
                        # self.newC.append(x[2])

                pl = self.rotationMatrix(self.currentPosYaw + math.radians(90), self.newA, self.newB, self.newC)

                self.newA, self.newB, self.newC = [], [], []

                # print("Virar para: " + str(math.degrees(self.rand)))
                # print("Angulo superior: " + str(math.degrees(randSup)))
                # print("Angulo inferior: " + str(math.degrees(randInf)))

                for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                    dentroRange = 1

                    if dentroRange:
                        if [round(i1), round(i2), round(i3)] not in self.abc and round(i2)>1:
                            self.newA.append(round(i2+3))
                            self.newB.append(round(i1+3))
                            self.newC.append(round(i3+1))
                            self.a.append(round(i2+3))
                            self.b.append(round(i1+3))
                            self.c.append(round(i3+1))
                            
                            self.abc.append([round(i1), round(i2), round(i3)])

                        value = np.sqrt((self.currentPosX - self.a[-1])**2 + (self.currentPosY - self.b[-1])**2 + (self.currentPosZ - self.c[-1])**2)      
                        if value < 15:
                            if self.countDym == 0:
                                self.newA.append(self.a[-1])
                                self.newB.append(self.b[-1])
                                self.newC.append(self.c[-1])
                            elif self.countDym == 1:
                                self.newA1.append(self.a[-1])
                                self.newB1.append(self.b[-1])
                                self.newC1.append(self.c[-1])
                            else:
                                self.newA2.append(self.a[-1])
                                self.newB2.append(self.b[-1])
                                self.newC2.append(self.c[-1])

                        if self.countDym == 0:
                            pl = rotationMatrix3D(self.currentPosYaw, self.newA, self.newB, self.newC, "yaw")
                            self.newA = pl[0].tolist() 
                            self.newB = pl[1].tolist() 
                            self.newC = pl[2].tolist()
                        elif self.countDym == 1:
                            pl = rotationMatrix3D(self.currentPosYaw, self.newA1, self.newB1, self.newC1, "yaw")
                            self.newA1 = pl[0].tolist() 
                            self.newB1 = pl[1].tolist()
                            self.newC1 = pl[2].tolist()
                        else:
                            pl = rotationMatrix3D(self.currentPosYaw, self.newA2, self.newB2, self.newC2, "yaw")
                            self.newA2 = pl[0].tolist()
                            self.newB2 = pl[1].tolist()
                            self.newC2 = pl[2].tolist()
                        self.countDym += 1

                        if self.countDym == 3:
                            for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                                if [round(i1), round(i2), round(i3)] not in self.abc:
                                    self.a.append(round(i1))
                                    self.b.append(round(i2))
                                    self.c.append(round(i3))
                                    self.abc.append([round(i1), round(i2), round(i3)])
                            
                            self.countDym = 0
                            self.checkDym = 1
                            self.newA, self.newB, self.newC, self.newA1, self.newB1, self.newC1, self.newA2, self.newB2, self.newC2 = [], [], [], [], [], [], [], [], []
            else:
                self.a = self.p.xobs
                self.b = self.p.xobs
                self.c = self.p.xobs

            print(self.a)
            print(self.b)
            print(self.c)

            if self.unic["definirRota"] == 0:
                print("Iniciando a definir rota")
                _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
                rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                print('DEFINIR ROTA')
                print('rx -------------------')
                print(rx)
                print('ry -------------------')
                print(ry)
                self.rotas["x"] = np.array(rx)
                self.rotas["y"] = np.array(ry)
                self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
                self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
                print(self.rotas["x"])
                print(self.rotas["y"])

                # plt.plot(self.a, self.b, '.k')
                
                # print(self.a)
                # print(self.b)
                # print(self.c)
                # ax = plt.axes(projection = "3d")
                # ax.plot3D(self.a, self.b, self.c, 'k.') 
                # ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                # ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                # ax.set_xlim(0,20) 
                # ax.set_ylim(0,10) 
                # ax.set_zlim(0,6) 
                # plt.pause(0.01)
                # plt.show()
                print("rota definida")                              
                self.unic["definirRota"] = 1

            self.velodyne = 0


    def callbackBuildMap3D_octomap(self, data):
        print('[callbackBuildMap3D_octomap]')
        if self.velodyne:
            if not self.knownEnvironment: 
                self.newA, self.newB, self.newC = [], [], []
                for marker in data.markers:
                    positions = marker.points
                    for position in positions:
                        round_pos_x = round(position.x) + self.currentPosX
                        round_pos_y = round(position.y) + self.currentPosY
                        round_pos_z = round(position.z) + self.currentPosZ
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                        round_pos_x = round(position.x) + self.currentPosX + 1
                        round_pos_y = round(position.y) + self.currentPosY
                        round_pos_z = round(position.z) + self.currentPosZ
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                        round_pos_x = round(position.x) + self.currentPosX - 1
                        round_pos_y = round(position.y) + self.currentPosY
                        round_pos_z = round(position.z) + self.currentPosZ
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                        round_pos_x = round(position.x) + self.currentPosX
                        round_pos_y = round(position.y) + self.currentPosY + 1
                        round_pos_z = round(position.z) + self.currentPosZ
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                        round_pos_x = round(position.x) + self.currentPosX
                        round_pos_y = round(position.y) + self.currentPosY - 1
                        round_pos_z = round(position.z) + self.currentPosZ
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                        round_pos_x = round(position.x) + self.currentPosX
                        round_pos_y = round(position.y) + self.currentPosY
                        round_pos_z = round(position.z) + self.currentPosZ + 1
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                        round_pos_x = round(position.x) + self.currentPosX
                        round_pos_y = round(position.y) + self.currentPosY
                        round_pos_z = round(position.z) + self.currentPosZ - 1
                        pos_to_add = [round_pos_x, round_pos_y, round_pos_z]
                        if pos_to_add not in self.abc:
                            # print(pos_to_add)
                            self.newA.append(round_pos_x)
                            self.newB.append(round_pos_y)
                            self.newC.append(round_pos_z)
                            self.a.append(round_pos_x)
                            self.b.append(round_pos_y)
                            self.c.append(round_pos_z)
                            self.abc.append(pos_to_add)

                # print(self.newA, self.newB, self.newC)
                # fig = plt.figure()
                # ax = fig.add_subplot(111, projection='3d')
                # ax.plot(self.a, self.b, self.c, marker='o')
                
                plt.plot(self.a, self.b, self.c, '.k')

                # plt.pause(0.01)
                # plt.show()

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY), round(self.currentPosZ)]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY))
                #     self.newC.append(round(self.currentPosZ))
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY) + 1, round(self.currentPosZ)]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY) + 1)
                #     self.newC.append(round(self.currentPosZ))
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY) - 1, round(self.currentPosZ)]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY) - 1)
                #     self.newC.append(round(self.currentPosZ))
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY), round(self.currentPosZ) + 1]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY))
                #     self.newC.append(round(self.currentPosZ) + 1)
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY) + 1, round(self.currentPosZ) + 1]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY) + 1)
                #     self.newC.append(round(self.currentPosZ) + 1)
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY) - 1, round(self.currentPosZ) + 1]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY) - 1)
                #     self.newC.append(round(self.currentPosZ) + 1)
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY), round(self.currentPosZ) - 1]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY))
                #     self.newC.append(round(self.currentPosZ) - 1)
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY) + 1, round(self.currentPosZ) - 1]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY) + 1)
                #     self.newC.append(round(self.currentPosZ) - 1)
                #     self.abc.append(pos_to_add)

                # pos_to_add = [round(self.currentPosX) - 2, round(self.currentPosY) - 1, round(self.currentPosZ) - 1]
                # if pos_to_add not in self.abc:
                #     self.newA.append(round(self.currentPosX) - 2)
                #     self.newB.append(round(self.currentPosY) - 1)
                #     self.newC.append(round(self.currentPosZ) - 1)
                #     self.abc.append(pos_to_add)

                


                # TODO validate if it is necessary or if octomap do it automatically
                # If it is not necessary set the first parameter as 0
                # pl = self.rotationMatrix(0, self.newA, self.newB, self.newC)
                
                # self.newA, self.newB, self.newC = [], [], []

                # for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                #     dentroRange = 1

                #     if dentroRange:
                #         if [round(i1), round(i2), round(i3)] not in self.abc and round(i2)>1:
                #             self.newA.append(round(i2+3))
                #             self.newB.append(round(i1+3))
                #             self.newC.append(round(i3+1))
                #             self.a.append(round(i2+3))
                #             self.b.append(round(i1+3))
                #             self.c.append(round(i3+1))
                            
                #             self.abc.append([round(i1), round(i2), round(i3)])

                #         value = np.sqrt((self.currentPosX - self.a[-1])**2 + (self.currentPosY - self.b[-1])**2 + (self.currentPosZ - self.c[-1])**2)      
                #         if value < 15:
                #             if self.countDym == 0:
                #                 self.newA.append(self.a[-1])
                #                 self.newB.append(self.b[-1])
                #                 self.newC.append(self.c[-1])
                #             elif self.countDym == 1:
                #                 self.newA1.append(self.a[-1])
                #                 self.newB1.append(self.b[-1])
                #                 self.newC1.append(self.c[-1])
                #             else:
                #                 self.newA2.append(self.a[-1])
                #                 self.newB2.append(self.b[-1])
                #                 self.newC2.append(self.c[-1])

                #         if self.countDym == 0:
                #             pl = rotationMatrix3D(self.currentPosYaw, self.newA, self.newB, self.newC, "yaw")
                #             self.newA = pl[0].tolist() 
                #             self.newB = pl[1].tolist() 
                #             self.newC = pl[2].tolist()
                #         elif self.countDym == 1:
                #             pl = rotationMatrix3D(self.currentPosYaw, self.newA1, self.newB1, self.newC1, "yaw")
                #             self.newA1 = pl[0].tolist() 
                #             self.newB1 = pl[1].tolist()
                #             self.newC1 = pl[2].tolist()
                #         else:
                #             pl = rotationMatrix3D(self.currentPosYaw, self.newA2, self.newB2, self.newC2, "yaw")
                #             self.newA2 = pl[0].tolist()
                #             self.newB2 = pl[1].tolist()
                #             self.newC2 = pl[2].tolist()
                #         self.countDym += 1

                #         if self.countDym == 3:
                #             for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                #                 if [round(i1), round(i2), round(i3)] not in self.abc:
                #                     self.a.append(round(i1))
                #                     self.b.append(round(i2))
                #                     self.c.append(round(i3))
                #                     self.abc.append([round(i1), round(i2), round(i3)])
                            
                #             self.countDym = 0
                #             self.checkDym = 1
                #             self.newA, self.newB, self.newC, self.newA1, self.newB1, self.newC1, self.newA2, self.newB2, self.newC2 = [], [], [], [], [], [], [], [], []
            else:
                self.a = self.p.xobs
                self.b = self.p.xobs
                self.c = self.p.xobs

            if self.unic["definirRota"] == 0:
                print("Iniciando a definir rota")
                _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
                rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                print('DEFINIR ROTA 2')
                print('rx -------------------')
                print(rx)
                print('ry -------------------')
                print(ry)
                print('OBSTACLES')
                print(self.a)
                print(self.b)

                self.rotas["x"] = np.array(rx)
                self.rotas["y"] = np.array(ry)
                self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
                self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))

                # plt.plot(self.a, self.b, '.k')
                # plt.plot(rx, ry, '-r')
                print("rota definida")                              
                self.unic["definirRota"] = 1

            self.velodyne = 0


    def callbackBuildMap(self, obs):
        buildMapX, buildMapY, buildMapZ = [], [], []

        for value in obs.poses:
            buildMapX.append(value.position.x)
            buildMapY.append(value.position.y)
            buildMapZ.append(value.position.z)
        
        if len(buildMapX) > 5: # if self.trocaYaw:
            for v1, v2, v3 in zip(buildMapX, buildMapY, buildMapZ):
                self.a.append(v1)
                self.b.append(v2)
                self.c.append(v3)
         
        print(time() - self.counts["total"])
        # if round(time() - self.counts["total"])%1==0 and 24 > time() - self.counts["total"] > 15:
        #     fig = plt.figure()
        #     ax = fig.add_subplot(111, projection='3d')
        #     zz = [2] * len(self.xx)
        #     ax.plot3D(self.xx, self.yy, zz, ".g")
        #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
        #     ax.plot3D(self.a, self.b, self.c,".k")
        #     ax.plot3D(buildMapX, buildMapY, buildMapZ,".y")
        #     ax.set_title(self.currentPosYaw)
        #     ax.set_xlabel("x (m)" + str(self.currentPosX))
        #     ax.set_ylabel("y (m)" + str(self.currentPosY))
        #     ax.set_zlabel("z (m)" + str(self.currentPosZ))
        #     ax.set_xlim(-50, 100)
        #     ax.set_ylim(-50, 100)
        #     ax.set_zlim(-3, 7)
        #     ax.view_init(73, -159)
        #     self.contador += 1
        #     plt.show()

    # ---------------------------- Onde o UAV ta ----------------------------------
    def callbackPosicao(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        if yaw < 0: yaw = math.pi + yaw
        
        # print("-------------------")
        # print(odom.pose)
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
                if ((abs(self.currentPosX - self.rotas["x"][-1]) < 0.2 and abs(self.currentPosY - self.rotas["y"][-1]) < 0.2 and abs(self.currentPosZ - self.rotas["z"][-1]) < 0.6) or(self.pos == len(self.rotas["x"]))) and self.unic["definirRota"]==1:
                # if self.pos == len(self.rotas["x"]):
                    if self.unic["bateria"]!= 4: self.unic["bateria"] = 2
                    if self.unic["bateriaGazebo"] == 1: self.unic["bateriaGazebo"] = 2

                    self.memoria["final"] = memory_usage()
                    self.cpu["final"] = psutil.cpu_percent()

                    print(self.rotas["x"])
                    print(self.rotas["y"])
                    print(self.rotas["z"])
                    print("Uso da bateria:")
                    print(self.bateria)
                    print("Uso da bateria Gazebo:")
                    self.bateriaGazebo["uso"] = self.bateriaGazebo["inicial"] - self.bateriaGazebo["atual"]
                    print(self.bateriaGazebo)
                    print("Comprimento: " + str(distancia_rota(self.rotas["x"], self.rotas["y"])))
                    try:
                        print("Media do tempo: " + str(stc.mean(self.variaveisLog["tt"])))
                        print("Variancia do tempo: " + str(stc.variance(self.variaveisLog["tt"])))
                        print("Desvio padrao do tempo: " + str(stc.stdev(self.variaveisLog["tt"])))
                    except:
                        pass
                    try:
                        print("Maior do tempo: " + str(max(self.variaveisLog["tt"])))
                        print("Menor do tempo: " + str(min(self.variaveisLog["tt"])))
                    except:
                        pass
                    print("Tempo de voo: " + str(time() - self.counts["total"]))

                    print("CPU:")
                    print(self.cpu)
                    print("Memoria:")
                    print(self.memoria)

                    print("CPU Real:")
                    self.cpu["inicial"] *= self.processadorDoPc / 100
                    self.cpu["final"] *= self.processadorDoPc / 100
                    self.cpu["uso"] *= self.cpu["final"] - self.cpu["inicial"]
                    print(self.cpu)
                    print("Memoria Real:")
                    self.memoria["inicial"] *=  self.memoriaDoPc / 100
                    self.memoria["final"] *=  self.memoriaDoPc / 100
                    self.memoria["uso"] *= self.memoria["final"] - self.memoria["inicial"]
                    print(self.memoria)

                    if self.log:
                        self.f.write("Uso da bateria:")
                        self.f.write(self.bateria)
                        self.f.write("Uso da bateria Gazebo:")
                        self.f.write(self.bateriaGazebo)
                        self.f.write("Comprimento: " + str(distancia_rota(self.rotas["x"], self.rotas["y"])))
                        self.f.write("Media do tempo: " + str(stc.mean(self.variaveisLog["tt"])))
                        try:
                            self.f.write("Variancia do tempo: " + str(stc.variance(self.variaveisLog["tt"])))
                            self.f.write("Desvio padrao do tempo: " + str(stc.stdev(self.variaveisLog["tt"])))
                        except:
                            pass
                        self.f.write("Maior do tempo: " + str(max(self.variaveisLog["tt"])))
                        self.f.write("Menor do tempo: " + str(min(self.variaveisLog["tt"])))
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
                        acrZ = 0#-0.6 if self.rotas["z"][self.pos-1] > self.rotas["z"][self.pos] else 0.8
                        self.tempo["wait"] = andarGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos] + acrZ, self.rotas["yaw"][self.pos], self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw, rotacao=False, MPC=self.controller)
                        self.velodyne = 1
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

                # checar se realmente foi para o ponto desejado, se nao tentar de novo
                # if self.unic["andar"] == 1:
                #     print(self.rotas["x"])
                #     if dist_euclidiana([self.currentPosX, self.currentPosY], [self.rotas["x"][self.pos], self.rotas["y"][self.pos]]) > 0.25:
                #         addRotaPonto(self.pos+1, self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["x"], self.rotas["y"], self.rotas["z"], self.rotas["yaw"], self.rotas["z"][self.pos], self.rotas["yaw"][self.pos])
                #     self.unic["andar"] = 0

                self.unic["busy"] = 0
                self.unic["idle"] = logStateMachine("Idle", self.unic["idle"])
                if time() - self.counts["parar"] > self.tempo["parar"]: 
                    # print(self.pos)
                    # print(len(self.rotas["x"]))
                    self.status = self.arrived
                    self.pos += 1            

def main():
    rospy.init_node("Planejador")
    globalPlanner()
    # plt.show()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    plt.show()

if __name__ == "__main__":
    main()
