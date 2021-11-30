# -*- coding: utf-8 -*-
import sys
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt

from helper.utils import *
from helper.ambiente import Pontos
from classic import birrt as alg

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion

from std_msgs.msg import String, Int32
from sensor_msgs.msg import BatteryState

from datetime import datetime
import statistics as stc
from sys import exit
import psutil

from utilsUAV import *

class globalPlanner:
    def __init__(self):
        # Tags
        self.land, self.takeoff, self.hover, self.sec, self.garraOn, self.esperarCamera, self.garraOff, self.calibrarTarget = 100, 200, 300, 400, 500, 600, 700, 800

        self.memoriaDoPc = 16000 # MB
        self.processadorDoPc = 2800 # MHz

        # Flags
        self.log = 0
        self.pos = 0
        self.goToHome = 0
        self.letra = ""
        self.unic = {"SM": 0, "busy": 0, "print": 0, "hover": 0, "definirRota": 0, "sec": 0, "andar": 0, "bateria": 0, "bateriaGazebo": 0}
        self.rotina = {"normal": 0, "visao": 0}

        # Mapa
        self.p = Pontos()
        self.a, self.b, self.a1, self.b1, self.a1b1 = [], [], [], [], [] # a,b = com capa | a1,b1 = sem capa

        # Variable Values
        self.altura, self.distNodes = 3.2, 1
        self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw = 2, 2, 0, 0
        self.alturaLaser = 0
        self.bateria = {"inicial": 0, "final": 0, "uso": 0}
        self.bateriaGazebo = {"inicial": 0, "final": 0, "uso": 0}
        self.variaveisLog = {"tt": []}
        self.memoria = {"inicial": 0, "final": 0}
        self.cpu = {"inicial": 0, "final": 0, "uso": 0}
        self.knownEnvironment = 0
        self.controller = 1 # Use 1 to MPC and 0 to cmd_vel

        # Trajectory
        self.rotas = {}
        self.rotas["x"], self.rotas["y"], self.rotas["z"], self.rotas["yaw"] = [], [], [], []
        # self.rotas["x"], self.rotas["y"] = rotaToGazebo2(self.rotas["x"], self.rotas["y"])
        # self.rotas["z"] = [self.altura] * len(self.rotas["x"])
        # self.rotas["yaw"] = [0] * len(self.rotas["x"])
        self.xWrite, self.yWrite, self.zWrite, self.yawWrite = [], [], [], []
        
        # Values to be Changed by the User
        # self.rotas["x"] = [2] 
        # self.rotas["y"] = [2] 
        # self.rotas["z"] = [self.altura] * len(self.rotas["x"])
        # self.rotas["yaw"] = [0] * len(self.rotas["x"])

        # Generate Trajectory
        # _, t, rx, ry = alg.run(show=0, vmx=self.a, vmy=self.b, startx=self.currentPosX, starty=self.currentPosY, p1=self.p)
        
        # To Log
        fileName = "logPathPlanner/comUAV/" + "RRT" + str(datetime.now().day) + str(datetime.now().month) + str(datetime.now().hour) + str(datetime.now().minute) + str(datetime.now().second) + ".txt"
        if self.log: self.f = open(fileName, "a")
        # self.variaveisLog["tt"].append(t)

        # Adjust Trajectory
        # self.rotas["x"], self.rotas["y"] = rotaToGazebo(rx, ry, self.a, self.b, self.distNodes)
        # self.rotas["z"] = [self.altura] * len(self.rotas["x"])
        # self.rotas["yaw"] = [0] * len(self.rotas["x"])

        # Times
        self.counts = {"total": 0, "parar": 0, "tempo": 0, "calibrandoTarget": 0}
        self.tempo = {"parar": 0, "takeoff": 6, "land": 6, "wait": 0, "hover": 5, "sec": 2}

        # Start
        self.counts["total"] = time()
        sleeping(t=1)

        # State Machine
        self.status = 2
        self.busy, self.arrived, self.idle = 1, 2, 3

        # Subscribers
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackPosicao)
        _ = rospy.Subscriber("/uav1/odometry/odom_local", Odometry, self.callbackMain)
        _ = rospy.Subscriber("/uav1/mavros/battery", BatteryState, self.callbackBattery)
        _ = rospy.Subscriber("/battery/percent", Int32, self.callbackBatteryGazebo)
        # _ = rospy.Subscriber("/battery/status", BatteryState, self.callbackBatteryGazebo)
        _ = rospy.Subscriber("/uav1/odometry/odom_main_innovation", Odometry, self.callbackDynamic)
        _ = rospy.Subscriber("/uav1/odometry/gps_local_odom", Odometry, self.callbackStatic)
        _ = rospy.Subscriber("/uav1/garmin/range", Range, self.callbackLaser)
        _ = rospy.Subscriber("/build_map", PoseArray, self.callbackBuildMap)

        # decolagemInicial()
        print("Espere um momento, ja iremos comecar")

        if self.knownEnvironment:
            _, t, rx, ry = alg.run(show=0, vmx=self.a, vmy=self.b, startx=self.currentPosX, starty=self.currentPosY, p1=self.p)
            rx, ry = rotaToGazebo(rx, ry, self.a, self.b, self.distNodes)
            self.rotas["x"] = rx
            self.rotas["y"] = ry
            self.rotas["z"] = [self.altura] * len(rx)
            self.rotas["yaw"] = [0] * len(rx)
            print("rota definida")
            self.unic["definirRota"] = 1
            
        rospy.sleep(5)
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
        if self.unic["bateria"] == 0:
            self.bateria["inicial"] = bat.current
            self.unic["bateria"] = 1

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
        v1x, v1y, v1all, v2x, v2y, v2all = [], [], [], [], [], []

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

        # checar se tem colisao com obstaculo dinamico
        # atualziar trajetoria

    # ---------------------------- Obstaculo Estatico ----------------------------------
    def callbackStatic(self, odom):
        # checar se tem colisao com obstaculo estatico
        # atualziar trajetoria
        if self.counts["total"] > 5 and self.unic["definirRota"] == 1:
            if colidirTrajetoria(self.a, self.b, self.rotas["x"], self.rotas["y"], self.pos, value=0.5):
                print("Tem colisao na trajetoria")
                _, t, rx, ry = alg.run(show=0, vmx=self.a, vmy=self.b, startx=self.currentPosX, starty=self.currentPosY, p1=self.p)
                rx, ry = rotaToGazebo(rx, ry, self.a, self.b, self.distNodes)
                print("Tempo para recalcular foi de: " + str(t))

                self.rotas["x"][self.pos:] = rx
                self.rotas["y"][self.pos:] = ry
                self.rotas["z"][self.pos:] = [self.altura] * len(rx)
                self.rotas["yaw"][self.pos:] = [0] * len(rx)
                self.variaveisLog["tt"].append(t)

    # ---------------------------- Altura Laser ----------------------------------
    def callbackBuildMap(self, obs):
        if not self.knownEnvironment: 
            buildMapX, buildMapY = [], []
            for value in obs.poses:
                if abs(value.position.x - self.currentPosX) > 0.3 and abs(value.position.y - self.currentPosY) > 0.3: 
                    buildMapX.append(value.position.x)
                    buildMapY.append(value.position.y)

            # a,b = com capa | a1,b1 = sem capa
            self.a, self.b, _, _, self.a1b1 =  laserROS(buildMapX, buildMapY, self.a, self.b, self.a1b1, tamCapa=0)
        else:
            self.a = self.p.xobs
            self.b = self.p.xobs
            self.c = self.p.xobs
        
        if self.unic["definirRota"] == 0:
            print("defining trajectory")
            _, t, rx, ry = alg.run(show=1, vmx=self.a, vmy=self.b, startx=self.currentPosX, starty=self.currentPosY, p1=self.p)
            print("improving trajectory")
            rx, ry = rotaToGazebo(rx, ry, self.a, self.b, self.distNodes)
            self.rotas["x"] = rx
            self.rotas["y"] = ry
            self.rotas["z"] = [self.altura] * len(rx)
            self.rotas["yaw"] = [0] * len(rx)
            print("trajectory defined")
            self.unic["definirRota"] = 1

    # ---------------------------- Onde o UAV ta ----------------------------------
    def callbackPosicao(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        if yaw < 0: yaw = math.pi + yaw
        
        self.currentPosX = odom.pose.pose.position.x 
        self.currentPosY = odom.pose.pose.position.y 
        self.currentPosZ = odom.pose.pose.position.z 
        self.currentPosYaw = yaw

        self.xWrite.append(self.currentPosX)
        self.yWrite.append(self.currentPosY)
        self.zWrite.append(self.currentPosZ)
        self.yawWrite.append(self.currentPosYaw)

    # ---------------------------- State Machine ----------------------------------
    def rotinaNormal(self):
        if self.unic["SM"] == 1: # Se tiver funcionando
            # ---------------- Decidindo sua vida ---------------------
            if self.status == self.arrived:
                self.unic["idle"] = 0

                # ---------------- Cabou ---------------------
                # if self.rotas["x"][-1] == 36 and self.rotas["x"][-1] == 32:
                if abs(self.currentPosX - self.rotas["x"][-1]) < 1 and abs(self.currentPosY - self.rotas["y"][-1]) < 1:
                # if self.pos == len(self.rotas["x"]):
                    if self.unic["bateria"] != 4: self.unic["bateria"] = 2
                    if self.unic["bateriaGazebo"] != 4: self.unic["bateriaGazebo"] = 2

                    self.memoria["final"] = memory_usage()
                    self.cpu["final"] = psutil.cpu_percent()

                    print("Uso da bateria:")
                    print(self.bateria)
                    print("Uso da bateria Gazebo:")
                    print(self.bateriaGazebo)
                    print("Comprimento: " + str(distancia_rota(self.rotas["x"], self.rotas["y"])))
                    try:
                        print("Media do tempo: " + str(stc.mean(self.variaveisLog["tt"])))
                        print("Variancia do tempo: " + str(stc.variance(self.variaveisLog["tt"])))
                        print("Desvio padrao do tempo: " + str(stc.stdev(self.variaveisLog["tt"])))
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
                    self.memoria["uso"] *=  self.memoria["final"] - self.memoria["inicial"]
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

                        self.f.write("Realmente seguido")
                        self.f.write("Trajetoria X")
                        for value in self.xWrite:
                            self.f.write(str(value))
                            self.f.write(", ")
                        self.f.write("\n\n")
                        print("Trajetoria Y")
                        for value in self.yWrite:
                            self.f.write(str(value))
                            self.f.write(", ")
                        self.f.write("\n\n")

                        self.f.write("Comprimento da rota realmente seguida: " + str(distancia_rota(self.xWrite, self.yWrite)))

                    print("Acabou a missao")
                    self.unic["SM"] = 0
                    exit()

                # ---------------- Flag Start/End ---------------------
                elif self.rotas["x"][self.pos] == self.land:
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
                    print("Indo para")
                    print(str(self.rotas["x"][self.pos]) + " - " + str(self.rotas["y"][self.pos]) + " - " + str(self.rotas["z"][self.pos]))
                    self.unic["print"], self.unic["andar"] = logStateMachine("Walking", self.unic["print"], self.unic["andar"])
                    self.tempo["wait"] = andarGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos], self.rotas["yaw"][self.pos], self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw, MPC=self.controller)

                if self.unic["SM"] == 1:
                    self.counts["tempo"] = time()
                    self.status = self.busy
            
            # ---------------- UAV ocupado ---------------------
            if self.status == self.busy:
                self.unic["print"] = 0
                self.unic["busy"] = logStateMachine("I am busy", self.unic["busy"])
                if time() - self.counts["tempo"] > self.tempo["wait"]: 
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
                    self.status = self.arrived
                    self.pos += 1            

def main():
    rospy.init_node("Planejador")
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # plt.show()

if __name__ == "__main__":
    main()