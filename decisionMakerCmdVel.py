#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from geometry_msgs.msg import PoseStamped, PoseArray, Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, UInt8, Int32
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from datetime import datetime
import os

from helper.utils import *
from utilsUAV import velocity

class globalPlanner:
    def __init__(self):
        # Constants
        self.tipoPouso = {"pid": 0, "xy": 1}
        self.tipoOrientacao = {"vicon": 0, "orbslam": 1, "imu": 2, "gps": 3, "tempOrbs": 0}
        self.cameraSettings = {"frontal": 0, "baixo": 1, "toLand": 2, "toCenter": 3}
        self.te = {"busy": 20, "cheguei": 21, "abortei": 22, "indoBusy": 23, "orbs1": 10, "orbs2": 11, "v1": 30, "v2": 31, "v3": 32, "pa1": 40, "pa2": 41, "p1": 50, "p2": 51, "f1": 60, "h1": 70}
        self.mv = {"xy": 0, "xyz": 1, "subir": 2, "xysubir": 3, "xyvarrer": 4, "xypousarAlto": 5, "xypousar": 6, "flip": 7, "rotacionar": 8, "xyrotacionar": 9, "fim": 10, "xyfim": 11, "hover": 12}
        self.rotas = dict()

        # Default Values
        # self.alturaDrone, self.safeHeigh, self.alturaDefault = 8, 7.5, 8
        self.alturaDrone, self.safeHeigh, self.alturaDefault = 14, 14, 14
        self.LIMIAR_PRECISAO_ALVO = {"alvo": 0.8, "erro": 0.3}
        self.ajustes = {"tempo": 2, "px": 0, "py": 0, "pz": 0}
        self.cameraAngle = -80
        self.pixelPouso = {"x": 20, "y": 20}
        self.focoCamera = {"x": 595.035494, "y": 594.674498} 
        self.pixelPID = 10

        # Variable Values
        self.curr_pos = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.alvosBloqueados = [[0, 0, 0]]
        self.calibraP0 = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.calibraGPS0 = {"x": 0, "y": 0, "z": None}
        self.targetPixel = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.visionValues = {"u": [], "v": [], "area": [], "yaw": [], "m": [], "flag": []}

        # SARC variables
        self.trajectoriesX = []
        self.trajectoriesY = []
        self.nextPoint = {"x": None, "y": None}
        self.currentPosXP, self.currentPosYP, self.currentPosZP, self.currentPosYawP = 0, 0, 0, 0
        self.tx, self.ty = [], []
        self.statusTrajectory = 0
        self.free = 0
        self.inTrajectory = 0
        self.posInicial, self.posFinal = 0, 0
        self.distanciaTotal = 0
        self.distanciaTotalAtual = 0
        self.distanciaConcluida = 0
        self.distanciaVolta = 0
        self.verAlga = 0
        self.mudarZ, self.trocarAltura = 0, 1
        self.maiorDistX, self.maiorDistY = 0, 0
        self.pares = []
        self.maxDistD1, self.maxDistD2 = 0, 0

        # Flags
        self.pos = 0
        self.unic = {"vicon": 0, "SM": 0, "odom": 0, "base": 0, "gps": 0, "altitude": -1, "follow": 0}
        self.rotina = {"normal": 0, "visao": 0, "orbs": 0, "perdeuOrbs": 0}
        self.estado = 0

        # Types of Routines
        self.tr = dict()
        self.tr["idle"] = 0
        self.tr["visao"] = {"startPID": 10, "pouso": 20, "startXY": 30, "aproxPID": 40, "nothing": 50, "lookTarget": 60}
        self.tr["orbs"] = {"start": 1, "p1": 10, "p2": 20, "p3": 30, "p4": 40, "end": 90}
        self.tr["perdeuOrbs"] = {"start": 10}
        self.tr["normal"] = {"start": 1, "subirOne": 5, "checkMove": 10, "xyvarrer": 25, "xypousarAlto": 30, "xypousar": 35, "flip": 40, "hover": 45, "fim": 50}

        # ---------------------------------------------------------------------------------------------
        # Values to be Changed by the User
        aPouso = 1.91
        self.rotas["y"] = [45.12,45.12,45.12,25.56,25.56,25.56,10]
        self.rotas["x"] = [11.19,11.19,11.19,11.19,11.19,11.19,11.19]
        self.rotas["z"] = [self.alturaDrone,aPouso,self.alturaDrone,self.alturaDrone,aPouso,self.alturaDrone,self.alturaDrone]

        self.rotas["x"].insert(0, 14)
        self.rotas["y"].insert(0, 61) #1166
        self.rotas["z"].insert(0, self.alturaDrone)

        # self.rotas["x"].insert(len(self.rotas["x"]), 0)
        # self.rotas["y"].insert(len(self.rotas["x"]), 0)
        # self.rotas["z"].insert(len(self.rotas["x"]), 0)
    
        self.rotas["yaw"] = [0] * len(self.rotas["x"])

        # Flags Options: xy | xyz | subir | xysubir | xyvarrer | xypousarAlto | xypousar | flip | rotacionar | xyrotacionar | fim | xyfim | hover
        # self.rotas["flag"] = [self.mv["subir"], self.mv["xy"], self.mv["xy"], self.mv["fim"]]
        self.rotas["flag"] = [self.mv["xyz"]] * len(self.rotas["x"])
        # self.rotas["flag"][0] = self.mv["subir"]
        # self.rotas["flag"][-1] = self.mv["fim"]

        # Landing Options: pid | xy
        self.pouso = self.tipoPouso["xy"]

        # Orientation Options: orbslam | vicon | imu
        self.orientacao = self.tipoOrientacao["imu"]

        # Camera Options: baixo | frontal
        self.camera = self.cameraSettings["frontal"]

        # Camera Functions Options: toLand | toCenter
        self.cameraFunction = self.cameraSettings["toLand"]

        # Use Rotation Matrix
        self.rotationMatrixUse = False
        
        # Times
        self.counts = {"mudarZ": 0, "verAlga": 0, "update": 0, "follow": 0, "total": 0, "parar": 0, "takeoff": 0, "land": 0, "perderOrbs": 0, "flip": 0, "varredura": 0, "camera": 0, "base": 0, "ignoraVisao": 0, "pid": 0, "hover": 0}
        self.tempo = {"mudarZ": 0.2, "verAlga": 1.5, "update": 0.5, "follow": 5, "parar": 2, "takeoff": 0.6, "land": 6, "perderOrbs": 6, "flip": 8, "varredura": 6, "camera": 7, "base": 50, "ignoraVisao": 5, "pid": 1.5, "hover": 2}
        # ---------------------------------------------------------------------------------------------

        # Flags Sent to Control
        self.idle = 0
        self.vai = 10
        self.land = 30
        self.perdeuOrbs = 44
        self.perdeuOrbsSempre = 47
        self.desFinal = 77
        self.calibraAlvo = 88
        self.abort = 100
        self.gambs = 850
        self.status = self.idle

        # Flags Received by the Control
        self.idle = 0
        self.busy = 15
        self.cheguei = 10
        self.abortei = 5
        self.ackParam = 3
        self.iniciouOrb = 0
        self.perdeu = 1
        self.semEscala = 2
        self.comEscala = 3

        # Publishers
        self.cmd = rospy.Publisher("planejamento1", PoseStamped, queue_size=1)
        # self.iniciaOrb = rospy.Publisher("iniciaOrbSlam", UInt8, queue_size=1)
        # self.iniciaLog = rospy.Publisher("iniciaLog", UInt8, queue_size=1)
        # self.flip = rospy.Publisher("/bebop/flip", UInt8, queue_size=1)
        # self.myCamera   = rospy.Publisher("/bebop/camera_control", Twist, queue_size = 1)
        self.sobe = rospy.Publisher("/drone1/takeoff", Empty, queue_size=1)
        self.desce = rospy.Publisher("/drone1/land", Empty, queue_size=1)
        self.pub_drone = rospy.Publisher('/drone1/trajectory_status', Pose, queue_size=10)
        self.dist_drone = rospy.Publisher('/arts/max_distance', Pose, queue_size=10)
        self.cmd_vel = rospy.Publisher('/drone1/cmd_vel', Twist, queue_size=1)

        # Initial Node
        rospy.init_node('global_planner_drone_one', anonymous=True)

        # Subscribers
        _ = rospy.Subscriber("/drone1/ground_truth/state", Odometry, self.callbackOdom)
        _ = rospy.Subscriber("/drone1/ground_truth/state", Odometry, self.callbackPosicao) # _ = rospy.Subscriber("statusPlanning1", UInt8, self.callbackPosicao)
        # _ = rospy.Subscriber("/scale/log", Odometry, self.callbackLog)
        # _ = rospy.Subscriber("/perception/target", PoseArray, self.callbackVisao)
        # _ = rospy.Subscriber("/perception/tubo", UInt8, self.callbackTubo)
        # _ = rospy.Subscriber("/perception/qrcode", String, self.callbackQrCode)
        # _ = rospy.Subscriber("/perception/setesegmentos", String, self.callbackSeteSegmentos)
        # _ = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, self.callbackVicon)
        # self.gps_sub = rospy.Subscriber("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", NavSatFix, self.callbackGPS)
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackPosicaoUAVPrincipal)
        # _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackFollow)
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackUpdate)
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackHelpAbort)
        _ = rospy.Subscriber("/drone1/mission_planner", Pose, self.callbackDadosMissao)
        # _ = rospy.Subscriber("/safe_heigh", Int32, self.callbackSafeHeigh)
        _ = rospy.Subscriber('/arts/max_distance', Pose, self.callbackRangeMax)
        

        # Start
        print("Start")
        if self.orientacao == self.tipoOrientacao["orbslam"]:
            self.tipoOrientacao["tempOrbs"] = 1
            self.orientacao = self.tipoOrientacao["imu"]
        self.counts["total"] = self.time()
        self.sleeping(t=1)
        self.unic["SM"] = 1
        self.rotina["normal"] = self.tr["normal"]["start"]

    # --------------------------------------------- Auxiliary Functions -------------------------------------------
    # Function to Wait
    def sleeping(self, t=5):
        rospy.sleep(t)

    # Function to Counting Time (with nanosecs)
    def time(self):
        return rospy.get_rostime().secs + (rospy.get_rostime().nsecs/1e9)

    # Function to Define Time to Control 
    def setTime(self, objx, objy, objz):
        euc = math.sqrt(pow(objx, 2) + pow(objy, 2) + pow(objz, 2))
        return abs(euc * self.ajustes["tempo"])

    def rotationMatrix(self, psi0, x1, y1, z1):
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
        return pos_local

    def haversine(self, lat1, lon1, lat2, lon2, km=False):
        R = 6373.0

        dlon = lon2 - lon1
        dlat = lat2 - lat1
        
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        distance = R * c

        if lon1-lon2 == 0:
            # angle = math.atan((lat1 - lat2)/(lon1 - lon2))
            angle = math.atan2((lat1 - lat2), (lon1 - lon2))
            
            # true in km
            # false in meters
            return distance, angle if km else distance * 1000, angle
        else:
            return distance*1000, None

    def addRota(self, x, y, vX, vY, vZ, vYaw, vMv, z=-100, yaw=0, mv=1):
        # mv = 1 = xyz
        vX.append(x)
        vY.append(y)
        vZ.append(z)
        vYaw.append(yaw)
        vMv.append(mv)
        return vX, vY, vZ, vYaw, vMv

    def addPouse(self, x, y, z, yaw, flag, fim):
        x.append(-100)
        y.append(-100)
        z.append(-100)
        yaw.append(0)
        flag.append(fim)

        return x, y, z, yaw, flag

    # --------------------------------------------- Functions to Control -------------------------------------------
    # Function to Send Position with Local Target
    def goToLocal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
        # print("LOCAL")
        if flag == self.vai:
            self.estado = self.te["indoBusy"]

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        
        # It's not allowed to make curves in this function
        pose.pose.orientation.x = 0
        
        # Define Time that can't be less than 3
        pose.pose.orientation.y = math.ceil(self.setTime(x, y, z)) if t == 0 else t
        if pose.pose.orientation.y < 4:
            pose.pose.orientation.y = 6
        
        pose.pose.orientation.z = self.vai if flag == 10 else flag

        if flag == self.vai:
            print("x: " + str(pose.pose.position.x) + " y: " + str(pose.pose.position.y) + " z: " + str(pose.pose.position.z))
            print("time: " + str(pose.pose.orientation.y))

        self.cmd.publish(pose)

    # Function to Send Position with Global Target
    def goToGlobal(self, x=-100, y=-100, z=-100, yaw=-100, t=0, flag=10):

        print("Posicao Atual - x: " + str(self.curr_pos["x"]) + " y: " + str(self.curr_pos["y"]) + " z: " + str(self.curr_pos["z"]))
        print("Indo - x: " + str(self.rotas["x"][self.pos]) + " y: " + str(self.rotas["y"][self.pos]) + " z: " + str(self.rotas["z"][self.pos]))
        print(self.pos)
        t_msg = Twist()
        t_msg.linear.x, t_msg.linear.y, t_msg.linear.z = velocity(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos], self.curr_pos["x"], self.curr_pos["y"],  self.curr_pos["z"], vel=0.1)
        t_msg.linear.x *= -1
        t_msg.linear.y *= -1
        print(t_msg.linear.x)
        print(t_msg.linear.y)
        print("publicando")
        self.status = self.busy
        self.cmd_vel.publish(t_msg)
        self.pos += 1


    # Function to Rotate UAV
    def rotateUAV(self, angle=-400, t=7, turn=-1):
        self.estado = self.te["indoBusy"]

        # If the angle is passed in degree converts to radian
        if abs(angle) > math.pi and abs(angle) - int(abs(angle)) == 0 and abs(angle) <= 360:
            angle = math.radians(angle)

        pose = PoseStamped()
        
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

        # Angle
        # Default turn to right
        pose.pose.orientation.x = math.pi * turn if angle == angle else angle - self.curr_pos["yaw"]

        if abs(angle) - (math.pi/2) < 0.1:
            pose.pose.orientation.y = t
        else:
             pose.pose.orientation.y = t * math.degrees(angle) / 90

        if pose.pose.orientation.y < 5:
            pose.pose.orientation.y = 5

        pose.pose.orientation.z = self.vai
        
        print("Rotate for " + str(pose.pose.orientation.y) + " seconds to " + str(math.degrees(pose.pose.orientation.x)) + " degrees")
        
        self.cmd.publish(pose)

    # Function to Up UAV
    def upUAV(self, z=-100, t=0):
        self.goToGlobal()

    # --------------------------------------------- Callbacks -------------------------------------------
    def callbackHelpAbort(self, seila):
        if self.verAlga:
            if self.time() - self.counts["verAlga"] > self.tempo["verAlga"]:
                self.goToGlobal(flag=self.vai)
                self.verAlga = 0

    def callbackPosicaoUAVPrincipal(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        if yaw < 0: yaw = math.pi + yaw
        
        self.currentPosXP = odom.pose.pose.position.x 
        self.currentPosYP = odom.pose.pose.position.y 
        self.currentPosZP = odom.pose.pose.position.z 
        self.currentPosYawP = yaw

    # --------------------------------------------- Position -------------------------------------------
    # These callbacks return the global position even when the initial orientation isn't zero (excpet log, because need reset in orbs)
    def callbackLog(self, log):
        if self.orientacao == self.tipoOrientacao["orbslam"]:
            self.curr_pos["x"] = log.pose.pose.position.x #+ self.ajustes["px"]
            self.curr_pos["y"] = log.pose.pose.position.y #+ self.ajustes["py"]
            self.curr_pos["z"] = log.pose.pose.position.z #+ self.ajustes["pz"]
            print("z atual: " + str(self.curr_pos["z"]))
            roll, pitch, yaw = euler_from_quaternion([log.pose.pose.orientation.x, log.pose.pose.orientation.y, log.pose.pose.orientation.z, log.pose.pose.orientation.w])
            self.curr_pos["yaw"] = yaw

    def callbackOdom(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        if yaw < 0: yaw = math.pi + yaw
        
        self.curr_pos["x"] = odom.pose.pose.position.x 
        self.curr_pos["y"] = odom.pose.pose.position.y 
        self.curr_pos["z"] = odom.pose.pose.position.z 
        self.curr_pos["yaw"] = yaw

        # provavelmente n eh necessario, mas falta debugar - MELHORIA
        if len(self.pares) == 0:
            self.pares = [[self.curr_pos["x"], self.curr_pos["y"]]]

    def callbackGPS(self, gps):
        if self.orientacao == self.tipoOrientacao["gps"]:
            lat = gps.latitude
            longi = gps.longitude
            alt = gps.altitude
            if gps.altitude != -1: # altitude return -1 if not available
                self.unic["altitude"] = 0
            else:
                self.unic["altitude"] = 1
            
            if self.calibraGPS0["z"] == None:
                self.calibraGPS0["z"] = alt

            if self.unic["gps"] == 0:
                self.calibraGPS0["x"] = lat
                self.calibraGPS0["y"] = longi
                self.unic["gps"] = 1
            else:
                # Encontra posicao atual
                if self.unic["altitude"] == 0:
                    self.curr_pos["z"] = alt
                
                hip, angle = self.haversine(self.calibraGPS0["x"], self.calibraGPS0["y"], lat, longi)
                
                inversoX = 1 if lat > self.calibraGPS0["x"] else -1
                inversoY = 1 if longi > self.calibraGPS0["y"] else -1

                if angle != None:
                    self.curr_pos["x"] = hip * math.sin(angle) * inversoX
                    self.curr_pos["y"] = hip * math.cos(angle) * inversoY
                else:
                    rospy.logwarn("Latitude and Longitude arent Updating")

    # --------------------------------------------- Main -------------------------------------------
    def callbackPosicao(self, data):
        if self.status == self.busy:
            if self.estado != self.te["busy"]:
                self.estado = self.te["busy"]
                print("Busy")
            # print("distancias")
            # print("X: " + str(self.curr_pos["x"]) + " - " + str(self.rotas["x"][self.pos-1]) + " is: " + str(self.curr_pos["x"] - self.rotas["x"][self.pos-1]))
            # print("X: " + str(self.curr_pos["y"]) + " - " + str(self.rotas["y"][self.pos-1]) + " is: " + str(self.curr_pos["y"] - self.rotas["y"][self.pos-1]))
            # print("X: " + str(self.curr_pos["z"]) + " - " + str(self.rotas["z"][self.pos-1]) + " is: " + str(self.curr_pos["z"] - self.rotas["z"][self.pos-1]))
            if abs(self.curr_pos["x"] - self.rotas["x"][self.pos-1]) < 2 and  abs(self.curr_pos["y"] - self.rotas["y"][self.pos-1]) < 2 and abs(self.curr_pos["z"] - self.rotas["z"][self.pos-1]) < 1.2: 
                print("cabo busy")
                t_msg = Twist()
                t_msg.linear.x = 0
                t_msg.linear.y = 0
                t_msg.linear.z = 0
                self.cmd_vel.publish(t_msg)
                
                self.status = self.cheguei
                self.counts["parar"] = self.time()
                self.unic["ouvirVisao"] = 0
        elif self.status == self.cheguei:
            if self.estado != self.te["cheguei"]:
                self.estado = self.te["cheguei"]
                self.status = self.idle
                # self.goToGlobal(flag=self.idle)
                self.counts["parar"] = self.time()
                print("Arrived")
        elif self.status == self.abortei:
            if self.estado != self.te["abortei"]:
                self.estado = self.te["abortei"]
                self.status = self.idle
                # self.goToGlobal(flag=self.idle)
                print("Aborted")
        elif self.status == self.ackParam:
            self.rotina["visao"] = self.tr["visao"]["aproxPID"]  

        # Main Routine
        if self.unic["SM"] == 1:
            self.tempo["parar"] = 8 if self.rotas["z"][self.pos-1] < 20 else 0.2
            if self.status == self.idle and (self.time() - self.counts["parar"] > self.tempo["parar"]) and self.estado != self.te["indoBusy"]:
                if self.rotina["visao"] + self.rotina["orbs"] + self.rotina["perdeuOrbs"] == 0:
                    self.normalRoutine()

    # --------------------------------------------- Routines -------------------------------------------
    def normalRoutine(self):
        if self.rotina["normal"] == self.tr["normal"]["start"]:
            self.sobe.publish(Empty())
            self.counts["takeoff"] = self.time()
            self.rotina["normal"] = self.tr["normal"]["subirOne"] 
            print("Going up - Take off")
        
        elif self.rotina["normal"] == self.tr["normal"]["subirOne"]:
            if(self.time() - self.counts["takeoff"] > self.tempo["takeoff"]):
                self.upUAV(z=self.rotas["z"][self.pos])
                # self.pos += 1
                if self.tipoOrientacao["tempOrbs"] == 1:
                    self.rotina["orbs"] = self.tr["orbs"]["start"]
                else:
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]

                self.unic["follow"] = 1

        elif self.estado == self.te["cheguei"]:
            # Do an action for each flag on rotes
            if self.rotina["normal"] == self.tr["normal"]["checkMove"]:  
                if self.pos >= len(self.rotas["flag"]):
                    self.toFollow()

                if self.rotas["flag"][self.pos] == self.mv["xy"]:
                    if self.pos == self.posInicial: print("Iniciando Coverage XY") ############## AINDA NAO TESTADO
                    if self.pos == self.posFinal: print("Terminando Coverage XY")

                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    # self.pos += 1
                
                elif self.rotas["flag"][self.pos] == self.mv["xyz"]:
                    # print("XYZ")
                    # print(self.rotas["z"])
                    if self.pos == self.posInicial: print("Iniciando Coverage XYZ") ############## AINDA NAO TESTADO
                    if self.pos == self.posFinal: print("Terminando Coverage XYZ")
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos], z=self.rotas["z"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    # self.pos += 1

                elif self.rotas["flag"][self.pos] == self.mv["subir"]:
                    self.upUAV(z=self.rotas["z"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    # self.pos += 1

                elif self.rotas["flag"][self.pos] == self.mv["rotacionar"]:
                    self.rotateUAV(angle=self.rotas["yaw"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    # self.pos += 1

                elif self.rotas["flag"][self.pos] == self.mv["xysubir"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.rotas["flag"][self.pos] = self.mv["subir"]
                
                elif self.rotas["flag"][self.pos] == self.mv["xyvarrer"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["xyvarrer"]
                
                elif self.rotas["flag"][self.pos] == self.mv["xypousarAlto"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["xypousarAlto"]
                
                elif self.rotas["flag"][self.pos] == self.mv["xypousar"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["xypousar"]
                
                elif self.rotas["flag"][self.pos] == self.mv["flip"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["flip"]

                elif self.rotas["flag"][self.pos] == self.mv["hover"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["hover"]

                elif self.rotas["flag"][self.pos] == self.mv["xyrotacionar"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.rotas["flag"][self.pos] = self.mv["rotacionar"]
                
                elif self.rotas["flag"][self.pos] == self.mv["xyfim"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.rotas["flag"][self.pos] = self.mv["fim"]

                elif self.rotas["flag"][self.pos] == self.mv["fim"]:
                    print("Landing")
                    self.desce.publish(Empty())
                    self.unic["SM"] = 0
                    print("End")

            # Starting Sub Routing
            elif self.rotina["normal"] == self.tr["normal"]["xyvarrer"]:
                twist = Twist()
                twist.angular.y = self.cameraAngle
                self.myCamera.publish(twist)
                print("Turn camera to: " + str(twist.angular.y) + " degrees")
                self.counts["camera"] = self.time()
                self.estado = self.te["v1"]

            elif self.rotina["normal"] == self.tr["normal"]["xypousarAlto"]:
                print("Landing high")
                self.desce.publish(Empty())
                self.counts["land"] = self.time()
                self.estado = self.te["pa1"]
            
            elif self.rotina["normal"] == self.tr["normal"]["xypousar"]:
                print("Landing low")
                self.desce.publish(Empty())
                self.counts["land"] = self.time()
                self.estado = self.te["p1"]
            
            elif self.rotina["normal"] == self.tr["normal"]["flip"]:
                print("Flip")
                self.flip.publish(0) # 0 - frontflip | 1 - backflip | 2 - rightflip | 3 - leftflip
                self.counts["flip"] = self.time()
                self.estado = self.te["f1"]

            elif self.rotina["normal"] == self.tr["normal"]["hover"]:
                print("Hover")
                self.counts["hover"] = self.time()
                self.estado = self.te["h1"]

        # Sub Routines
        elif self.estado == self.te["v1"]:
            if self.time() - self.counts["camera"] > self.tempo["camera"]:
                self.counts["varredura"] = self.time()
                self.estado = self.te["v2"]
                print("Start scan")
        elif self.estado == self.te["v2"]:
            if self.time() - self.counts["varredura"] > self.tempo["varredura"]:
                twist = Twist()
                twist.angular.y = 0
                self.myCamera.publish(twist)
                print("Turn camera to: " + str(twist.angular.y) + " degree")
                self.estado = self.te["v3"]
                self.counts["camera"] = self.time()
        elif self.estado == self.te["v3"]:
            if self.time() - self.counts["camera"] > self.tempo["camera"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                # self.pos += 1
                self.estado = self.te["cheguei"]


        elif self.estado == self.te["pa1"]:
            if self.time() - self.counts["land"] > self.tempo["land"]:
                self.sobe.publish(Empty())
                self.counts["takeoff"] = self.time()
                self.estado = self.te["pa2"]
                print("Take off")

        elif self.estado == self.te["pa2"]:
            if self.time() - self.counts["takeoff"] > self.tempo["takeoff"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                # self.pos += 1
                self.estado = self.te["cheguei"]
        

        elif self.estado == self.te["p1"]:
            if self.time() - self.counts["land"] > self.tempo["land"]:
                self.sobe.publish(Empty())
                self.counts["takeoff"] = self.time()
                self.estado = self.te["p2"]
                print("Take off")

        elif self.estado == self.te["p2"]:
            if self.time() - self.counts["takeoff"] > self.tempo["takeoff"]:
                self.upUAV(z=self.rotas["z"][self.pos])
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                # self.pos += 1


        elif self.estado == self.te["f1"]:
            if self.time() - self.counts["flip"] > self.tempo["flip"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                # self.pos += 1
                self.estado = self.te["cheguei"] 

        elif self.estado == self.te["h1"]:
            if self.time() - self.counts["hover"] > self.tempo["hover"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                # self.pos += 1
                self.estado = self.te["cheguei"] 


def main():
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()