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
from classic import rrt_connect_3d as alg

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
        self.land, self.takeoff, self.hover, self.sec, self.garraOn, self.esperarCamera, self.garraOff, self.calibrarTarget = 100, 200, 300, 400, 500, 600, 700, 800

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

        # Trajectory
        self.rotas = {}
        self.rotas["x"], self.rotas["y"], self.rotas["z"], self.rotas["yaw"] = np.array([]), np.array([]), np.array([]), np.array([])
        ax0 = [ 2.0, 2.1595752,   2.31915041,  2.47872561,  2.63830081,  2.79787602,  2.95745122,  3.11702642,  3.27660163,  3.43617683,  3.59575204,  3.75532724,  3.91490244,  4.13426649,  4.42194478,  4.70962306,  4.99730135,  5.28497964,  5.57265793,  5.86033622,  6.00417537,  6.29185366,  6.57953195,  6.86721024,  7.15488852,  7.44256681,  7.7302451,   8.01792339,  8.30560168,  8.59327997,  8.88095826,  9.0, 9.0, 9.0, 9.0, 9.0,  9.0, 9.0, 9.0, 9.0, 9.0, 9.0,  9.17082007,  9.45849836,  9.74617665, 10.03385494, 10.32153323, 10.60921152, 10.89688981, 11.1845681,  11.47224639, 11.75992468, 12.04760297, 12.33528126, 12.62295954, 12.91063783, 13.19831612, 13.48599441, 13.7736727,  14.06135099, 14.34902928, 14.63670757, 14.92438586, 15.0,         15.0,         15.0, 15.0,         15.0,         15.0,         15.0, 15.22581217, 15.51349046, 15.80116875, 16.08884704, 16.37652533, 16.66420362, 16.95188191, 17.2395602, 17.52723849, 17.81491678, 18.10259507, 18.39027336, 18.67795165, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9, 18.9]
        ay0 = [4.0, 4.23936281, 4.47872561, 4.71808842, 4.95745122, 5.19681403, 5.43617683, 5.67553964, 5.91490244, 6.15426525, 6.39362805, 6.63299086, 6.87235366, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.84001732, 6.56710173, 6.29418615, 6.02127056, 5.74835497, 5.47543938, 5.20252379, 4.9296082,  4.65669262, 4.38377703, 4.11086144, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.25330823, 4.54098651, 4.8286648, 5.11634309, 5.40402138, 5.69169967, 5.97937796, 6.26705625, 6.55473454, 6.84241283, 7.13009112, 7.41776941, 7.7054477,  7.99312599, 8.28080428, 8.56848257, 8.85616086]
        az0 = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 1.94667244, 1.85570058, 1.76472872, 1.67375685, 1.58278499, 1.49181313, 1.40084126, 1.3098694,  1.21889754, 1.12792568, 1.03695381, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.21206415, 1.49974244, 1.78742073, 2.07509902, 2.36277731, 2.6504556,  2.93813389, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
        ax1=[2.1, 2.3769350533478777, 2.753870106695755, 3.959428613344527, 6.252235032204958, 8.043866923745487, 9.91593552547417, 10.058998588483274, 11.01229099628721, 12.124465472058471, 13.07775787986241, 14.0, 14.0, 14.0, 14.0, 14.0, 14.0, 14.0, 14.0, 13.999999999999998, 14.0, 13.999999999999998, 14.0, 14.0, 14.0, 14.0, 14.108934402843776, 15.091565725212527, 16.074197047581286, 17.05682836995004, 18.039459692318793, 19.0, 19.0]
        ay1=[2.1, 4.261610320087264, 6.52322064017453, 8.0, 8.0, 7.9780665381272575, 7.042032237262916, 7.0, 7.0, 7.0, 7.000000000000002, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.999999999999999, 7.0, 6.999999999999999, 7.0, 6.838959564262494, 5.856328241893737, 4.873696919524981, 4.0, 4.0, 4.0, 4.0, 4.0, 5.4669859511741405, 7.715267618608777]
        az1=[2.0, 2.0, 2.0, 2.0, 2.0, 1.978066538127257, 1.042032237262915, 1.0147496471208182, 1.2530727490718023, 1.5311163680146174, 1.7694394699656024, 2.195777790999999, 2.359549678061458, 2.523321565122917, 2.687093452184376, 2.8508653392458356, 3.014637226307295, 3.1784091133687546, 3.34218100043021, 3.5059528874916692, 3.6697247745531287, 3.833496661614588, 3.9972685486760473, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 3.7066028097651724, 3.2569464762782445]
        ax2=[2.1, 2.0, 2.0, 2.849650475337115, 4.949653464304296, 7.049656453271476, 8.099657947755068, 9.149659442238656, 11.074662182125241, 12.124663676608833, 13.0, 13.0, 13.0, 13.0, 13.028539646397071, 14.082871062320471, 15.137202478243875, 16.015811991513377, 17.07014340743678, 18.124474823360185, 19.000000000000004, 19.000000000000004]
        ay2=[2.1, 4.284384734500703, 6.393047566347506, 7.999999999999999, 7.999999999999999, 8.0, 8.0, 7.999999999999999, 8.0, 8.0, 8.0, 6.958732698719235, 5.904401282795832, 4.850069866872429, 3.9999999999999996, 4.0, 4.0, 4.0, 4.0, 4.000000000000001, 5.553809959238892, 7.621523983695556]
        az2=[2.0, 2.0, 2.0, 1.9227590476966256, 1.7318496850632457, 1.5409403224298657, 1.4454856411131756, 1.3500309597964855, 1.1750307107158873, 1.0795760293991972, 2.756882566780065, 4.0, 4.0, 4.0, 3.9999999999999996, 4.0, 4.0, 4.0, 4.0, 4.000000000000001, 3.6892380081522225, 3.2756952032608897]
        self.rotas["x"] = ax1#(np.array(ax2)*5).tolist()
        self.rotas["y"] = ay1#(np.array(ay2)*5).tolist()
        self.rotas["z"] = az1#(np.array(az2)/0.8).tolist()
        self.rotas["yaw"] = [0] * len(self.rotas["x"])
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
        # _ = rospy.Subscriber("/uav1/velodyne/scan", PointCloud2, self.callbackBuildMap3D)
        # _ = rospy.Subscriber("/zed/cloud_map", PointCloud2, self.callbackBuildMapZed3D)
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
        if self.unic["SM"] == 1:# and self.unic["definirRota"] == 1:     
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

        # Criar um topico com amostragem dos obstaculos em X (0.5)
        # Restringir a visao para a direcao em q o drone esta indo
        # checagem a cada 2 tempos
        # Delimitar um range, como 5-8 metros
        # identificar se o obs eh dinamico ou n
        ########## COMO FAZER ##########
        ########## 2D ##########
        # checar se ha obs ond antes n tinha e se n tem ond antes tinha
        # isso ta melhor explicado no slide da quali
        ########## 3D ##########
        # checar se o obstaculo esta se aproximando em do UAV entre os dois tempos
        ########## END ##########
        # faz a suavizacao com: o ponto atual (0.5 a frente), dois ponto a frente (1 metros), o pseudo ponto de colisao e o ponto destino
        # ponto destino sera 4-6 (2 a 3 metros) pontos a frente do ponto de colisao
        # pegar a parte da trajetoria apos o ponto destino
        # substituir apenas essa parte da trajetoria

        # checar se tem uma colisao com obstaculo dinamico
        # atualziar trajetoria
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
        # checar se tem colisao com obstaculo estatico
        # atualziar trajetoria
        if ((time() - self.counts["total"]) > 5) and (self.unic["definirRota"] == 1) and (self.avoidStatic == 0 or self.avoidStatic + 6 < self.pos) and (self.pos > 0):
            if abs(self.currentPosX - self.p.xt) > 4 and abs(self.currentPosY - self.p.yt) > 3 and abs(self.currentPosZ - self.p.yt) > 1:
                posicao = self.pos
                # tamanho = len(self.rotas["x"])
                # print("VAI CHECAR COLISAO")
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
                    ax = plt.axes(projection = "3d")
                    ax.plot3D(self.a, self.b, self.c, 'k.') 
                    ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                    ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                    ax.set_xlim(0,20) 
                    ax.set_ylim(0,10) 
                    ax.set_zlim(0,6) 
                    plt.pause(0.01)
                    plt.show()
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

            # if self.currentPosX > 15 and self.currentPosY > 12 and self.currentPosZ > 1 and self.currentPosX < 20 and self.currentPosY < 15:
            #     ax = plt.axes(projection = "3d")
            #     ax.plot3D(self.a, self.b, self.c, 'k.') 
            #     ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
            #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
            #     ax.set_xlim(-10,100) 
            #     ax.set_ylim(-10,100) 
            #     ax.set_zlim(0,4) 
            #     plt.pause(0.01)
            #     plt.show()

            if self.unic["definirRota"] == 0:
                # print(self.a)
                # print(self.b)
                # print(self.c)
                _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
                rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                self.rotas["x"] = np.array(rx)
                self.rotas["y"] = np.array(ry)
                self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
                self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
                print("rota definida")
                self.variaveisLog["tt"].append(t)
                ax = plt.axes(projection = "3d")
                ax.plot3D(self.a, self.b, self.c, 'k.') 
                ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                # ax.set_xlim(0,20) 
                # ax.set_ylim(0,10) 
                ax.set_zlim(0,4) 
                plt.pause(0.01)
                plt.show()
                # print(self.rotas["x"])
                # print(self.rotas["y"])
                self.unic["definirRota"] = 1

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

            # if self.unic["definirRota"] == 0:
            #     # print(self.a)
            #     # print(self.b)
            #     # print(self.c)
            #     _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
            #     rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
            #     self.rotas["x"] = np.array(rx)
            #     self.rotas["y"] = np.array(ry)
            #     self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
            #     self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
            #     print("rota definida")
            #     # print(self.rotas["x"])
            #     # print(self.rotas["y"])
                # self.unic["definirRota"] = 1

            self.velodyne = 0

    # def callbackHokuyo(self, data):
    #     x, y, z = [], [], []
    #     for index, value in enumerate(data.ranges[120:600]):
    #         if value < 30:
    #             aux = np.linspace(180, 0, 600-120)
    #             theta = aux[index]
    #             x.append(np.cos(np.radians(theta)) * value)
    #             y.append(np.sin(np.radians(theta)) * value)
    #             z.append(self.currentPosZ)

    #     pl = rotationMatrix3D(self.currentPosYaw, x, y, z, "yaw")
    #     for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
    #         if [round(i1), round(i2), round(i3)] not in self.abc:
    #             self.a.append(round(i1))
    #             self.b.append(round(i2))
    #             self.c.append(round(i3))
    #             self.abc.append([round(i1), round(i2), round(i3)])

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


    def callbackHokuyo20(self, data):
        # header: 
        # seq: 193
        # stamp: 
        #     secs: 57
        #     nsecs:  24000000
        # frame_id: "uav1/fcu"
        # angle_min: -2.3561899662017822
        # angle_max: 2.3561899662017822
        # angle_increment: 0.006554075051099062
        # time_increment: 0.0
        # scan_time: 0.0
        # range_min: 0.10000000149011612
        # range_max: 30.0
        a, b, c = [], [], []
        for index, value in enumerate(data.ranges[120:600]):
            if value < 30:
                aux = np.linspace(180, 0, 600-120)
                theta = aux[index]
                a.append(np.cos(np.radians(theta)) * value)
                b.append(np.sin(np.radians(theta)) * value)
                c.append(self.currentPosZ)

        pl = rotationMatrix3D(self.currentPosYaw, a, b, c, "yaw")
        pl = rotationMatrix3D(math.radians(-20), pl[0], pl[1], pl[2], "pitch")
        for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
            if [round(i1), round(i2), round(i3)] not in self.abc:
                self.a.append(round(i1))
                self.b.append(round(i2))
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
                ax.set_zlim(0,4) 
                plt.pause(0.01)
                plt.show()

    def callbackBuildMapZed3D(self, data):
        gen = point_cloud2.read_points(data, skip_nans=True)
        int_data = list(gen)

        self.newA, self.newB, self.newC = [], [], []
    
        for x in int_data:
            if round(x[2] > 0):
                self.newA.append(round(x[0]))
                self.newB.append(round(-x[1]))
                self.newC.append(round(x[2]))

        pl = self.rotationMatrix(0, self.newA, self.newB, self.newC)

        self.newA, self.newB, self.newC = [], [], []

        for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
            dentroRange = 1


            if dentroRange:
                if [round(i1), round(i2), round(i3)] not in self.abc and round(i2)>1:
                    self.newA.append(round(i2))
                    self.newB.append(round(i1))
                    self.newC.append(round(i3+1))
                    self.a.append(round(i2))
                    self.b.append(round(i1))
                    self.c.append(round(i3+1))
                    self.abc.append([round(i1), round(i2), round(i3)])

        # if self.currentPosX > 15 and self.currentPosY > 12 and self.currentPosZ > 1 and self.currentPosX < 20 and self.currentPosY < 15:
        #     ax = plt.axes(projection = "3d")
        #     ax.plot3D(self.a, self.b, self.c, 'k.') 
        #     ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
        #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
        #     ax.set_xlim(-10,100) 
        #     ax.set_ylim(-10,100) 
        #     ax.set_zlim(0,4) 
        #     plt.pause(0.01)
        #     plt.show()

        if self.unic["definirRota"] == 0:
            print("Iniciando a definir rota")
            _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
            rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
            self.rotas["x"] = np.array(rx)
            self.rotas["y"] = np.array(ry)
            self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
            self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
            ax = plt.axes(projection = "3d")
            ax.plot3D(self.a, self.b, self.c, 'k.') 
            ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
            ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
            ax.set_xlim(0,20) 
            ax.set_ylim(0,10) 
            ax.set_zlim(0,6) 
            plt.pause(0.01)
            plt.show()
            # print(self.a)
            # print(self.b)
            # print(self.c)
            print("rota definida")                              
            self.unic["definirRota"] = 1


    def callbackBuildMap3D(self, data):
        if self.velodyne:
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
                    if [round(i1), round(i2), round(i3)] not in self.abc and round(i2)>1:
                        self.newA.append(round(i2+3))
                        self.newB.append(round(i1+3))
                        self.newC.append(round(i3+1))
                        self.a.append(round(i2+3))
                        self.b.append(round(i1+3))
                        self.c.append(round(i3+1))
                        self.abc.append([round(i1), round(i2), round(i3)])

            # self.a = self.p.xobs
            # self.b = self.p.yobs
            # self.c = self.p.zobs

            # if self.currentPosX > 15 and self.currentPosY > 12 and self.currentPosZ > 1 and self.currentPosX < 20 and self.currentPosY < 15:
            #     ax = plt.axes(projection = "3d")
            #     ax.plot3D(self.a, self.b, self.c, 'k.') 
            #     ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
            #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
            #     ax.set_xlim(-10,100) 
            #     ax.set_ylim(-10,100) 
            #     ax.set_zlim(0,4) 
            #     plt.pause(0.01)
            #     plt.show()

            if self.unic["definirRota"] == 0:
                print("Iniciando a definir rota")
                # ax = plt.axes(projection = "3d")
                # ax.plot3D(self.a, self.b, self.c, 'k.') 
                # ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                # ax.set_xlim(0,20) 
                # ax.set_ylim(0,10) 
                # ax.set_zlim(0,5) 
                # plt.pause(0.01)
                # plt.show()
                _, t, rx, ry, rz = alg.run(show=0, vmx=self.a, vmy=self.b, vmz=self.c, startx=self.currentPosX, starty=self.currentPosY, startz=self.currentPosZ, p1=self.p)
                rx, ry, rz = rotaToGazebo3D(rx, ry, rz, self.a, self.b, self.c, self.distNodes)
                self.rotas["x"] = np.array(rx)
                self.rotas["y"] = np.array(ry)
                self.rotas["z"] = np.array(rz) # [self.altura] * len(rx)
                self.rotas["yaw"] = np.array([self.anguloGeral] * len(rx))
                ax = plt.axes(projection = "3d")
                ax.plot3D(self.a, self.b, self.c, 'k.') 
                ax.plot3D(self.rotas["x"], self.rotas["y"], self.rotas["z"], 'y.') 
                ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                ax.set_xlim(0,20) 
                ax.set_ylim(0,10) 
                ax.set_zlim(0,6) 
                plt.pause(0.01)
                plt.show()
                # print(self.a)
                # print(self.b)
                # print(self.c)
                print("rota definida")                              
                self.unic["definirRota"] = 1

            # if 10<time()-self.counts["total"]< 15:
            #     ax = plt.axes(projection = "3d")
            #     ax.plot3D(self.a, self.b, self.c, 'y.') 
            #     ax.plot3D(a1, a2, a3, 'b.') 
            #     ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
            #     ax.set_xlim(-10,100) 
            #     ax.set_ylim(-10,100) 
            #     ax.set_zlim(0,3) 
            #     try:
            #         ax.set_title(str(self.pos) + " - " + str(math.degrees(self.currentPosYaw)) + " - " + str(math.degrees(math.atan2(self.rotas["y"][self.pos+1] - self.currentPosY, self.rotas["x"][self.pos+1] - self.currentPosX))))
            #     except:
            #         pass
            #     ax.set_xlabel("x (m)" + str(self.currentPosX))
            #     ax.set_ylabel("y (m)" + str(self.currentPosY))
            #     ax.set_zlabel("z (m)" + str(self.currentPosZ))
            #     ax.view_init(azim=-111, elev=77)
            #     # plt.savefig("new" + str(self.numeroImagem))

            #     # self.f.write("---------------------------------------------------------------\n")
            #     # self.f.write(str(self.numeroImagem) + "\n")
            #     # self.f.write(str(self.pos) + "\n")
            #     # self.f.write("Posicao atual: x-" + str(self.currentPosX) + " | y-" + str(self.currentPosY) + " | z-" + str(self.currentPosZ) + " | yaw-" + str(math.degrees(self.currentPosYaw)) + "\n")
            #     # self.f.write("Indo para: x-" + str(self.rotas["x"][self.pos]) + " | y-" + str(self.rotas["y"][self.pos]) + " | z-" + str(self.rotas["z"][self.pos]) + " | yaw-" + str(math.degrees(self.rotas["yaw"][self.pos])) + "\n")
            #     # self.f.write("Angulo entre posicao inicial e a proxima: " + str(math.degrees(math.atan2(self.rotas["y"][self.pos+1] - 7, self.rotas["x"][self.pos+1] - 7))) + "\n")
            #     # self.f.write("Angulo entre posicao atual e a proxima: " + str(math.degrees(math.atan2(self.rotas["y"][self.pos+1] - self.currentPosY, self.rotas["x"][self.pos+1] - self.currentPosX))) + "\n")
            #     # self.f.write("Angulo entre posicao anterior e a proxima: " + str(math.degrees(math.atan2(self.rotas["y"][self.pos+1] - self.rotas["y"][self.pos], self.rotas["x"][self.pos+1] - self.rotas["x"][self.pos]))) + "\n")
            #     # self.f.write("Angulo entre posicao anterior e a inicial: " + str(math.degrees(math.atan2(self.rotas["y"][self.pos] - 7, self.rotas["x"][self.pos] - 7))) + "\n")
            #     # self.f.write("Angulo entre posicao atual e a inicial: " + str(math.degrees(math.atan2(self.currentPosY - 7, self.currentPosX - 7))) + "\n")
            #     # self.f.write("\n")

            #     plt.pause(0.01)
            #     plt.show()

            # self.numeroImagem += 1

            self.velodyne = 0

    def callbackBuildMap(self, obs):
        buildMapX, buildMapY, buildMapZ = [], [], []
        # if round(time() - self.counts["total"])%5==0:
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
                # self.contador0 += 1
                # self.x = self.rotas["x"][self.contador0]
                # self.y = self.rotas["y"][self.contador0]
                # self.trocaYaw = 0

        # a,b = com capa | a1,b1 = sem capa
        # if self.trocaYaw:
        #     pl = rotationMatrix(self.currentPosYaw, buildMapX, buildMapY, buildMapZ)
        #     for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
        #         self.a.append(i1)
        #         self.b.append(i2)
        #         self.c.append(i3)
        #         self.trocaYaw = 0
        
        # print("-------------------------------")
        # self.a, self.b, self.c, self.a1, self.b1, self.c1, self.a1b1c1 =  mapping3D(buildMapX, buildMapY, buildMapZ, self.a, self.b, self.c, self.a1b1c1, tamCapa=0)
        # self.a, self.b, self.a1, self.b1, self.a1b1c1 =  laserROS(buildMapX, buildMapY, self.a, self.b, self.a1b1c1, tamCapa=0)
        # print(self.a)
        # print(round(time() - self.counts["total"]))
        print(time() - self.counts["total"])
        if round(time() - self.counts["total"])%1==0 and 24 > time() - self.counts["total"] > 15:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            zz = [2] * len(self.xx)
            ax.plot3D(self.xx, self.yy, zz, ".g")
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
            # ax.azim = -124
            ax.view_init(73, -159)
            # plt.savefig("orbsslam" + str(self.contador) + ".png")
            # plt.savefig("orbsslamNewView" + str(self.contador) + ".png")
            # plt.pause(0.01)
            self.contador += 1
            plt.show()
            # plt.plot(self.currentPosX, self.currentPosY, ".r")
            # plt.plot(self.rotas["x"], self.rotas["y"], "-r")
            # plt.plot(buildMapX, buildMapY, ".y")
            # plt.plot(self.a, self.b, ".k")
            # plt.show()
        
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
                # if self.rotas["x"][-1] == 36 and self.rotas["x"][-1] == 32:
                if ((abs(self.currentPosX - self.p.xt) < 0.2 and abs(self.currentPosY - self.p.yt) < 0.2 and abs(self.currentPosZ - self.p.yt) < 0.6) or(self.pos == len(self.rotas["x"]))) and self.unic["definirRota"]==1:
                # if self.pos == len(self.rotas["x"]):
                    if self.unic["bateria"]!= 4: self.unic["bateria"] = 2
                    if self.unic["bateriaGazebo"]!= 4: self.unic["bateriaGazebo"] = 2

                    self.memoria["final"] = memory_usage()
                    self.cpu["final"] = psutil.cpu_percent()

                    # if self.rotas["x"][-1] != self.land:
                    #     self.rotas["x"].append(25)
                    #     self.rotas["y"].append(18)
                    #     self.rotas["z"].append(self.altura)
                    #     self.rotas["yaw"].append(self.rotas["yaw"][-1])
                        # self.pos -= 1
                        # print(self.rotas["x"])
                        # print("Ate mais e obrigado pelos peixes")

                    print(self.rotas["x"])
                    print(self.rotas["y"])
                    print(self.rotas["z"])
                    print("Uso da bateria:")
                    print(self.bateria)
                    print("Uso da bateria Gazebo:")
                    print(self.bateriaGazebo)
                    print("Comprimento: " + str(distancia_rota(self.rotas["x"], self.rotas["y"])))
                    print("Media do tempo: " + str(stc.mean(self.variaveisLog["tt"])))
                    try:
                        print("Variancia do tempo: " + str(stc.variance(self.variaveisLog["tt"])))
                        print("Desvio padrao do tempo: " + str(stc.stdev(self.variaveisLog["tt"])))
                    except:
                        pass
                    print("Maior do tempo: " + str(max(self.variaveisLog["tt"])))
                    print("Menor do tempo: " + str(min(self.variaveisLog["tt"])))
                    print("Tempo de voo: " + str(time() - self.counts["total"]))

                    print("CPU:")
                    print(self.cpu)
                    print("Memoria:")
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
                        self.tempo["wait"] = andarGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos] + acrZ, self.rotas["yaw"][self.pos], self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw, rotacao=False)
                        self.velodyne = 1
                    #     plt.plot(self.rotas["x"], self.rotas["y"])
                    # plt.plot(self.a, self.b, ".k")
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
                    print(self.pos)
                    print(len(self.rotas["x"]))
                    # print(self.a)
                    # print(self.rotas["x"])
                    # print(self.pos)
                    # plt.plot(self.a, self.b, ".k")
                    # plt.plot(self.currentPosX, self.currentPosY, ".b")
                    # plt.plot(self.rotas["x"], self.rotas["y"], "-r")
                    # plt.xlim(0,50)
                    # plt.ylim(0,50)
                    # plt.show()
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
    # plt.show()

if __name__ == "__main__":
    main()