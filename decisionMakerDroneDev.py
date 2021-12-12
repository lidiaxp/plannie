#!/usr/bin/env python3

import math
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty, UInt8
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from datetime import datetime 
import os

class globalPlanner:
    def __init__(self):
        dirName = "logPlanner"
        try:
            os.mkdir(dirName)
        except:
            pass
        nameFile = "GlobalPlannerLog" + str(datetime.now().year) + str(datetime.now().month) + str(datetime.now().day) + str(datetime.now().hour) + str(datetime.now().minute) + str(datetime.now().second)
        self.file = open(dirName + "/" + nameFile + ".txt","w")
        
        # Constants
        self.tipoPouso = {"pid": 0, "xy": 1}
        self.tipoOrientacao = {"vicon": 0, "orbslam": 1, "imu": 2, "gps": 3, "tempOrbs": 0}
        self.cameraSettings = {"frontal": 0, "baixo": 1, "toLand": 2, "toCenter": 3}
        self.te = {"busy": 20, "cheguei": 21, "abortei": 22, "indoBusy": 23, "orbs1": 10, "orbs2": 11, "v1": 30, "v2": 31, "v3": 32, "pa1": 40, "pa2": 41, "p1": 50, "p2": 51, "f1": 60}
        self.mv = {"xy": 0, "xyz": 1, "subir": 2, "xysubir": 3, "xyvarrer": 4, "xypousarAlto": 5, "xypousar": 6, "flip": 7, "rotacionar": 8, "xyrotacionar": 9, "fim": 10, "xyfim": 11}
        self.rotas = dict()

        # Default Values
        self.alturaDrone = 2.5
        self.LIMIAR_PRECISAO_ALVO = {"alvo": 0.8, "erro": 0.3}
        self.ajustes = {"tempo": 5, "px": 0, "py": 0, "pz": 0}
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

        # Flags
        self.pos = 0
        self.unic = {"vicon": 0, "SM": 0, "odom": 0, "base": 0, "gps": 0, "altitude": -1}
        self.rotina = {"normal": 0, "visao": 0, "orbs": 0, "perdeuOrbs": 0}
        self.estado = 0

        # Types of Routines
        self.tr = dict()
        self.tr["idle"] = 0
        self.tr["visao"] = {"startPID": 10, "pouso": 20, "startXY": 30, "aproxPID": 40, "nothing": 50, "lookTarget": 60}
        self.tr["orbs"] = {"start": 1, "p1": 10, "p2": 20, "p3": 30, "p4": 40, "end": 90}
        self.tr["perdeuOrbs"] = {"start": 10}
        self.tr["normal"] = {"start": 1, "subirOne": 5, "checkMove": 10, "xyvarrer": 25, "xypousarAlto": 30, "xypousar": 35, "flip": 40, "fim": 50}

        # ---------------------------------------------------------------------------------------------
        # Values to be Changed by the User
        self.rotas["x"] = [0, 4, 5, 0]
        self.rotas["y"] = [0, 5, -6, 0]
        self.rotas["z"] = [self.alturaDrone, 0, 0, 0]
        self.rotas["yaw"] = [0, -0, 0, 0]

        # Flags Options: xy | xyz | subir | xysubir | xyvarrer | xypousarAlto | xypousar | flip | rotacionar | xyrotacionar | fim | xyfim
        self.rotas["flag"] = [self.mv["subir"], self.mv["xy"], self.mv["xy"], self.mv["fim"]]

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
        self.counts = {"total": 0, "parar": 0, "takeoff": 0, "land": 0, "perderOrbs": 0, "flip": 0, "varredura": 0, "camera": 0, "base": 0, "ignoraVisao": 0, "pid": 0}
        self.tempo = {"parar": 0, "takeoff": 1, "land": 6, "perderOrbs": 6, "flip": 8, "varredura": 6, "camera": 7, "base": 50, "ignoraVisao": 5, "pid": 1.5}
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
        self.iniciaOrb = rospy.Publisher("iniciaOrbSlam", UInt8, queue_size=1)
        self.iniciaLog = rospy.Publisher("iniciaLog", UInt8, queue_size=1)
        self.flip = rospy.Publisher("/bebop/flip", UInt8, queue_size=1)
        self.myCamera   = rospy.Publisher("/bebop/camera_control", Twist, queue_size = 1)
        self.sobe = rospy.Publisher("/bebop/takeoff", Empty, queue_size=1)
        self.desce = rospy.Publisher("/bebop/land", Empty, queue_size=1)

        # Subscribers
        self.statusOdometry_sub = rospy.Subscriber("/drone1/ground_truth/state", Odometry, self.callbackOdom)
        self.statusLog_sub = rospy.Subscriber("/scale/log", Odometry, self.callbackLog)
        self.statusPlanning_sub = rospy.Subscriber("statusPlanning1", UInt8, self.callbackPosicao)
        self.statusVisao_sub = rospy.Subscriber("/perception/target", PoseArray, self.callbackVisao)
        self.statusTubo_sub = rospy.Subscriber("/perception/tubo", UInt8, self.callbackTubo)
        self.statusQrCode_sub = rospy.Subscriber("/perception/qrcode", String, self.callbackQrCode)
        self.statusSeteSegmentos_sub = rospy.Subscriber("/perception/setesegmentos", String, self.callbackSeteSegmentos)
        self.vicon_sub = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, self.callbackVicon)
        # self.gps_sub = rospy.Subscriber("/bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", NavSatFix, self.callbackGPS)
    
        # Initial Node
        rospy.init_node('Global_Planner', anonymous=True)

        # Start
        self.file.write("Start\n")
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

    # --------------------------------------------- Functions to Control -------------------------------------------
    # Function to Send Position with Local Target
    def goToLocal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
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
        if pose.pose.orientation.y < 3:
            pose.pose.orientation.y = 3
        
        pose.pose.orientation.z = self.vai if flag == 10 else flag

        if flag == self.vai:
            self.file.write("x: " + str(pose.pose.position.x) + " y: " + str(pose.pose.position.y) + " z: " + str(pose.pose.position.z) + "\n")
            self.file.write("time: " + str(pose.pose.orientation.y) + "\n")
            print("x: " + str(pose.pose.position.x) + " y: " + str(pose.pose.position.y) + " z: " + str(pose.pose.position.z))
            print("time: " + str(pose.pose.orientation.y))

        self.cmd.publish(pose)

    # Function to Send Position with Global Target
    def goToGlobal(self, x=-100, y=-100, z=-100, yaw=-100, t=0, flag=10):
        if flag == self.vai:
            self.estado = self.te["indoBusy"]

        if x != -100:
            self.file.write("Posicao Atual - x: " + str(self.curr_pos["x"]) + " y: " + str(self.curr_pos["y"]) + " z: " + str(self.curr_pos["z"]) + "\n")
            self.file.write("Global - x: " + str(x) + " y: " + str(y) + " z: " + str(z) + "\n")
            print("Posicao Atual - x: " + str(self.curr_pos["x"]) + " y: " + str(self.curr_pos["y"]) + " z: " + str(self.curr_pos["z"]))
            print("Global - x: " + str(x) + " y: " + str(y) + " z: " + str(z))

        # If don't sent an axis as param won't move on the axis
        x1 = x - self.curr_pos["x"] if x != -100 else 0
        y1 = y - self.curr_pos["y"] if y != -100 else 0
        z1 = z - self.curr_pos["z"] if z != -100 else 0

        # It's not allowed to do curves in this function
        # yaw1 = yaw - self.curr_pos["yaw"] if yaw != -100 else 0

        if self.rotationMatrixUse == True:
            pos_local = self.rotationMatrix(self.curr_pos["yaw"], x1, y1, z1)
            self.goToLocal(pos_local[0], pos_local[1], pos_local[2], 0, t, flag)
        else:
            self.goToLocal(x1, y1, z1, 0, t, flag)
        

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
        
        self.file.write("Rotate for " + str(pose.pose.orientation.y) + " seconds to " + str(math.degrees(pose.pose.orientation.x)) + " degrees" + "\n")
        print("Rotate for " + str(pose.pose.orientation.y) + " seconds to " + str(math.degrees(pose.pose.orientation.x)) + " degrees")
        
        self.cmd.publish(pose)

    # Function to Up UAV
    def upUAV(self, z=-100, t=0):
        self.estado = self.te["indoBusy"]
        
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.alturaDrone if z == -100 else z - self.curr_pos["z"]
        pose.pose.orientation.x = 0
        
        # Define Time that can't be less than 3
        pose.pose.orientation.y = math.ceil(self.setTime(0, 0, pose.pose.position.z)) if t == 0 else t
        if pose.pose.orientation.y < 5:
            pose.pose.orientation.y = 5
        
        pose.pose.orientation.z = self.vai
        
        self.file.write("Going up in " + str(pose.pose.orientation.y) + " seconds to " + str(pose.pose.position.z) + " meters" + "\n")
        print("Going up in " + str(pose.pose.orientation.y) + " seconds to " + str(pose.pose.position.z) + " meters")

        self.cmd.publish(pose)

    # --------------------------------------------- Functions to Help Landing-------------------------------------------
    # Function that checks if target has already been visited
    def checkTarget(self, destino):
        bloqueado = 0
        a2 = self.setTime(destino[0], destino[1], 0) / self.ajustes["tempo"]
        
        for i in range(len(self.alvosBloqueados)):
            a1 = self.setTime(self.alvosBloqueados[i][0], self.alvosBloqueados[i][1], 0) / self.ajustes["tempo"]
                        
        if abs(a1 - a2) < self.LIMIAR_PRECISAO_ALVO["alvo"]:
            bloqueado += 1

        # Target wans't visited
        if bloqueado == 0:
            return True

        # Target was visited
        else:
            return False 

    # --------------------------------------------- Callbacks -------------------------------------------
    # --------------------------------------------- Position -------------------------------------------
    # These callbacks return the global position even when the initial orientation isn't zero (excpet log, because need reset in orbs)
    def callbackLog(self, log):
        if self.orientacao == self.tipoOrientacao["orbslam"]:
            self.curr_pos["x"] = log.pose.pose.position.x + self.ajustes["px"]
            self.curr_pos["y"] = log.pose.pose.position.y + self.ajustes["py"]
            self.curr_pos["z"] = log.pose.pose.position.z + self.ajustes["pz"]
            roll, pitch, yaw = euler_from_quaternion([log.pose.pose.orientation.x, log.pose.pose.orientation.y, log.pose.pose.orientation.z, log.pose.pose.orientation.w])
            self.curr_pos["yaw"] = yaw

    def callbackOdom(self, odom):
        roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        
        if self.unic["odom"] == 0:
            self.calibraP0["yaw"] = yaw
            self.calibraP0["x"] = odom.pose.pose.position.x
            self.calibraP0["y"] = odom.pose.pose.position.y
            self.calibraP0["z"] = odom.pose.pose.position.z
            self.unic["odom"] = 1
        else:
            if self.orientacao == self.tipoOrientacao["imu"]:
                self.curr_pos["x"] = odom.pose.pose.position.x + self.ajustes["px"]
                self.curr_pos["y"] = odom.pose.pose.position.y + self.ajustes["py"]
                self.curr_pos["z"] = odom.pose.pose.position.z + self.ajustes["pz"]
                self.curr_pos["yaw"] = yaw - self.calibraP0["yaw"]

        if self.orientacao == self.tipoOrientacao["gps"]:
            self.curr_pos["yaw"] = yaw - self.calibraP0["yaw"]
        
        if self.unic["altitude"] == 1:
            self.curr_pos["z"] = odom.pose.pose.position.z + self.ajustes["pz"]

    def callbackVicon(self, vicon):
        if self.orientacao == self.tipoOrientacao["vicon"]:
            pv = vicon.transform.translation
            ov = vicon.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([ov.x, ov.y, ov.z, ov.w])        
            if self.unic["vicon"] == 0:
                self.calibraP0["yaw"] = yaw
                self.calibraP0["x"] = pv.x
                self.calibraP0["y"] = pv.y
                self.calibraP0["z"] = pv.z
                self.unic["vicon"] = 1
            else:
                self.curr_pos["x"] = pv.x + self.ajustes["px"]
                self.curr_pos["y"] = pv.y + self.ajustes["py"]
                self.curr_pos["z"] = pv.z + self.ajustes["pz"]
                self.curr_pos["yaw"] = yaw - self.calibraP0["yaw"]

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
                
                if angle != None:
                    self.curr_pos["x"] = hip * math.sin(angle)
                    self.curr_pos["y"] = hip * math.cos(angle)
                else:
                    rospy.logwarn("Latitude and Longitude arent Updating")
                
    # --------------------------------------------- Vision -------------------------------------------
    def callbackQrCode(self, qrcode):
        letra = qrcode.data

    def callbackTubo(self, tubo):
        cor = tubo.data # 0: red | 1: green

    def callbackSeteSegmentos(self, sete):
        seg = sete.data 

    def callbackVisao(self, target):
        # u - v - area - pitch - yaw - m - flag
        if self.unic["SM"] == 1 and self.checkTarget([self.curr_pos["x"], self.curr_pos["y"]]) and self.time() - self.counts["ignoraVisao"] > self.tempo["ignoraVisao"]:
            self.file.write("alvo ainda n foi visitado, pode ver" + "\n")
            print("alvo ainda n foi visitado, pode ver")
            u, v, area, yaw, m, flag = [], [], [], [], [], []
            for pose in target.poses:
                u.append(pose.position.x)
                v.append(pose.position.y)
                area.append(pose.position.z)
                yaw.append(pose.orientation.y)
                m.append(pose.orientation.z)
                flag.append(pose.orientation.w)

            # Flag to warn that vision routine was entered by camera
            self.tr["visao"]["vision"] = True

            # Routine with PID
            if self.pouso == self.tipoPouso["pid"]:
                self.visionValues["u"] = u[0]
                self.visionValues["v"] = v[0]
                self.visionValues["area"] = area[0]
                self.visionValues["yaw"] = yaw[0]
                self.visionValues["m"] = m[0]
                self.visionValues["flag"] = flag[0] 

                if self.rotina == self.tr["idle"]:
                    self.goToLocal(flag=self.abort)
                    self.rotina["visao"] = self.tr["visao"]["startPID"]
            
            elif self.pouso == self.tipoPouso["xy"]:
            # Routine without PID
                if self.unic["base"] == 0:
                    self.visionValues["u"] = []
                    self.visionValues["v"] = []
                    self.visionValues["area"] = []
                    self.visionValues["yaw"] = []
                    self.visionValues["m"] = []
                    self.visionValues["flag"] = []
                    self.counts["base"] = 0
                    self.unic["base"] = 1
                    self.rotina["visao"] = self.tr["visao"]["startXY"]
                    self.goToLocal(flag=self.abort)
                    self.file.write("rotina de aproximcao inicial" + "\n")
                    print("rotina de aproximcao inicial")
                
                if self.counts["base"] < self.tempo["base"]:
                        self.counts["base"] += 1
                        self.visionValues["u"].append(u[0])
                        self.visionValues["v"].append(v[0])
                        self.visionValues["area"].append(area[0])
                        self.visionValues["yaw"].append(yaw[0])
                        self.visionValues["m"].append(m[0])
                        self.visionValues["flag"].append(flag[0])

    # --------------------------------------------- Main -------------------------------------------
    def callbackPosicao(self, data):
        orbs = (0b11000000 & data.data) >> 6
        status = (0b00001111 & data.data)

        if orbs == self.perdeu: 
            if self.counts["perderOrbs"] == 0:
                self.counts["perderOrbs"] = self.time()
                self.file.write("perdeu orbs" + "\n")
                print("perdeu orbs")
                self.orientacao = self.tipoOrientacao["imu"]
                self.rotina["perdeuOrbs"] = self.tr["perdeuOrbs"]["start"]
                self.goToGlobal(flag=self.abort)
        elif orbs == self.comEscala:
            self.rotina["perdeuOrbs"] = self.tr["idle"]
            self.orientacao = self.tipoOrientacao["orbslam"]

        # Global States
        if status == self.busy:
            if self.estado != self.te["busy"]:
                self.estado = self.te["busy"]
                self.file.write("Busy" + "\n")
                print("Busy")
        elif status == self.cheguei:
            if self.estado != self.te["cheguei"]:
                self.estado = self.te["cheguei"]
                self.goToGlobal(flag=self.idle)
                self.counts["parar"] = self.time()
                self.file.write("Arrived" + "\n")
                print("Arrived")
        elif status == self.abortei:
            if self.estado != self.te["abortei"]:
                self.estado = self.te["abortei"]
                self.goToGlobal(flag=self.idle)
                self.file.write("Aborted" + "\n")
                print("Aborted")
        elif status == self.ackParam:
            self.rotina["visao"] = self.tr["visao"]["aproxPID"]  


        # Main Routine
        if self.unic["SM"] == 1:
            if status == self.idle and (self.time() - self.counts["parar"] > self.tempo["parar"]) and self.estado != self.te["indoBusy"]:
                if self.rotina["visao"] + self.rotina["orbs"] + self.rotina["perdeuOrbs"] == 0:
                    self.normalRoutine()
                else: 
                    print("algo deu errado")
                # elif self.rotina["visao"] > 0 and self.rotina["orbs"] + self.rotina["perdeuOrbs"] == 0:
                #     self.visionRoutine()
                # elif self.rotina["orbs"] > 0 and self.rotina["visao"] + self.rotina["perdeuOrbs"] == 0:
                #     self.orbsRoutine(orbs)
                # elif self.rotina["perdeuOrbs"] > 0 and self.rotina["orbs"] + self.rotina["visao"] == 0:
                #     self.lostOrbsRoutine()

    # --------------------------------------------- Routines -------------------------------------------
    def normalRoutine(self):
        if self.rotina["normal"] == self.tr["normal"]["start"]:
            self.sobe.publish(Empty())
            self.counts["takeoff"] = self.time()
            self.rotina["normal"] = self.tr["normal"]["subirOne"] 
            self.file.write("Going up - Take off" + "\n")
            print("Going up - Take off")
        
        elif self.rotina["normal"] == self.tr["normal"]["subirOne"]:
            if(self.time() - self.counts["takeoff"] > self.tempo["takeoff"]):
                self.upUAV(z=self.rotas["z"][self.pos])
                self.pos += 1
                if self.tipoOrientacao["tempOrbs"] == 1:
                    self.rotina["orbs"] = self.tr["orbs"]["start"]
                else:
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]

        elif self.estado == self.te["cheguei"]:
            # Do an action for each flag on rotes
            if self.rotina["normal"] == self.tr["normal"]["checkMove"]:  
                if self.rotas["flag"][self.pos] == self.mv["xy"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.pos += 1
                
                elif self.rotas["flag"][self.pos] == self.mv["xyz"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos], z=self.rotas["z"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.pos += 1

                elif self.rotas["flag"][self.pos] == self.mv["subir"]:
                    self.upUAV(z=self.rotas["z"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.pos += 1

                elif self.rotas["flag"][self.pos] == self.mv["rotacionar"]:
                    self.rotateUAV(angle=self.rotas["yaw"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.pos += 1

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

                elif self.rotas["flag"][self.pos] == self.mv["xyrotacionar"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.rotas["flag"][self.pos] = self.mv["rotacionar"]
                
                elif self.rotas["flag"][self.pos] == self.mv["xyfim"]:
                    self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.rotas["flag"][self.pos] = self.mv["fim"]

                elif self.rotas["flag"][self.pos] == self.mv["fim"]:
                    self.file.write("Landing" + "\n")
                    print("Landing")
                    self.desce.publish(Empty())
                    self.unic["SM"] = 0
                    self.file.write("End" + "\n")
                    print("End")
                    self.file.close()

            # Starting Sub Routing
            elif self.rotina["normal"] == self.tr["normal"]["xyvarrer"]:
                twist = Twist()
                twist.angular.y = self.cameraAngle
                self.myCamera.publish(twist)
                self.file.write("Turn camera to: " + str(twist.angular.y) + " degrees" + "\n")
                print("Turn camera to: " + str(twist.angular.y) + " degrees")
                self.counts["camera"] = self.time()
                self.estado = self.te["v1"]

            elif self.rotina["normal"] == self.tr["normal"]["xypousarAlto"]:
                self.file.write("Landing high" + "\n")
                print("Landing high")
                self.desce.publish(Empty())
                self.counts["land"] = self.time()
                self.estado = self.te["pa1"]
            
            elif self.rotina["normal"] == self.tr["normal"]["xypousar"]:
                self.file.write("Landing low" + "\n")
                print("Landing low")
                self.desce.publish(Empty())
                self.counts["land"] = self.time()
                self.estado = self.te["p1"]
            
            elif self.rotina["normal"] == self.tr["normal"]["flip"]:
                self.file.write("Flip" + "\n")
                print("Flip")
                self.flip.publish(0) # 0 - frontflip | 1 - backflip | 2 - rightflip | 3 - leftflip
                self.counts["flip"] = self.time()
                self.estado = self.te["f1"]


        # Sub Routines
        elif self.estado == self.te["v1"]:
            if self.time() - self.counts["camera"] > self.tempo["camera"]:
                self.counts["varredura"] = self.time()
                self.estado = self.te["v2"]
                self.file.write("Start scan" + "\n")
                print("Start scan")
        elif self.estado == self.te["v2"]:
            if self.time() - self.counts["varredura"] > self.tempo["varredura"]:
                twist = Twist()
                twist.angular.y = 0
                self.myCamera.publish(twist)
                self.file.write("Turn camera to: " + str(twist.angular.y) + " degree" + "\n")
                print("Turn camera to: " + str(twist.angular.y) + " degree")
                self.estado = self.te["v3"]
                self.counts["camera"] = self.time()
        elif self.estado == self.te["v3"]:
            if self.time() - self.counts["camera"] > self.tempo["camera"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                self.pos += 1
                self.estado = self.te["cheguei"]


        elif self.estado == self.te["pa1"]:
            if self.time() - self.counts["land"] > self.tempo["land"]:
                self.sobe.publish(Empty())
                self.counts["takeoff"] = self.time()
                self.estado = self.te["pa2"]
                self.file.write("Take off" + "\n")
                print("Take off")

        elif self.estado == self.te["pa2"]:
            if self.time() - self.counts["takeoff"] > self.tempo["takeoff"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                self.pos += 1
                self.estado = self.te["cheguei"]
        

        elif self.estado == self.te["p1"]:
            if self.time() - self.counts["land"] > self.tempo["land"]:
                self.sobe.publish(Empty())
                self.counts["takeoff"] = self.time()
                self.estado = self.te["p2"]
                self.file.write("Take off" + "\n")
                print("Take off")

        elif self.estado == self.te["p2"]:
            if self.time() - self.counts["takeoff"] > self.tempo["takeoff"]:
                self.upUAV(z=self.rotas["z"][self.pos])
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                self.pos += 1


        elif self.estado == self.te["f1"]:
            if self.time() - self.counts["flip"] > self.tempo["flip"]:
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                self.pos += 1
                self.estado = self.te["cheguei"] 
        
    # Routine to do something when see an object ----------------------------------------------------------------------
    def visionRoutine(self):
        if self.rotina["visao"] == self.tr["visao"]["startPID"]:
            self.file.write("Define final value to PID" + "\n")
            print("Define final value to PID")
            self.goToLocal(x=np.mean(self.visionValues["u"]), y=np.mean(self.visionValues["v"]), flag=self.desFinal)
            self.rotina["visao"] = self.tr["visao"]["nothing"]
        
        elif self.rotina["visao"] == self.tr["visao"]["aproxPID"]:
            if abs(np.mean(self.visionValues["u"])) > self.pixelPouso["x"] or abs(np.mean(self.visionValues["v"])) > self.pixelPouso["y"]:
                # Need implement a function to simulate pixels if vision dont see nothing
                if self.time() - self.counts["pid"] > self.tempo["pid"]:
                    if self.targetPixel["x"] > self.targetPixel["y"]:
                        escala = self.targetPixel["x"] / self.targetPixel["y"]
                        self.targetPixel["x"] -= self.pixelPID * escala
                        self.targetPixel["y"] -= self.pixelPID 
                    else: 
                        escala = self.targetPixel["y"] / self.targetPixel["x"]
                        self.targetPixel["x"] -= self.pixelPID
                        self.targetPixel["y"] -= self.pixelPID * escala
                    
                else:    
                    self.goToLocal(x=np.mean(self.visionValues["u"]), y=np.mean(self.visionValues["v"]), flag=self.calibraAlvo)
                    self.counts["pid"] = self.time()
            else:
                self.file.write("Above the target, time to land" + "\n")
                print("Above the target, time to land")
                if self.cameraFunction == self.cameraSettings["toLand"]:
                    self.rotina["visao"] = self.tr["visao"]["pouso"]
                elif self.cameraFunction == self.cameraSettings["toCenter"]:
                    self.rotina["visao"] = self.tr["visao"]["lookTarget"]

        elif self.rotina["visao"] == self.tr["visao"]["lookTarget"]:
            self.counts["ignoraVisao"] = self.time()
            self.rotina["visao"] = self.tr["visao"]["lookTarget2"]

        elif self.rotina["visao"] == self.tr["visao"]["lookTarget2"]:
            if self.time() - self.counts["ignoraVisao"] > self.tempo["ignoraVisao"]:
                self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos])
                self.counts["ignoraVisao"] = self.time()
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                self.unic["base"] = 0
                self.rotina["visao"] = self.tr["idle"]

        elif self.rotina["visao"] == self.tr["visao"]["startXY"]:
            if self.counts["base"] == self.tempo["base"]:
                self.counts["base"] += 1
                if self.camera == self.cameraSettings["frontal"]:
                    self.targetPixel["x"] = (np.mean(self.visionValues["u"]) * (self.curr_pos["z"]) / self.focoCamera["x"]) + 0.3
                elif self.camera == self.cameraSettings["baixo"]:
                    self.targetPixel["x"] = (np.mean(self.visionValues["u"]) * (self.curr_pos["z"]) / self.focoCamera["x"])
                self.targetPixel["y"] = np.mean(self.visionValues["v"]) * (self.curr_pos["z"]) / self.focoCamera["y"]
                self.file.write("See the target" + "\n")
                print("See the target")

                if self.checkTarget([self.targetPixel["x"], self.targetPixel["y"]]):
                    self.file.write("Approaching Target" + "\n")
                    print("Approaching Target")
                    self.goToLocal(self.targetPixel["x"], self.targetPixel["y"], t=6)
                    if self.cameraFunction == self.cameraSettings["toLand"]:
                        self.rotina["visao"] = self.tr["visao"]["pouso"]
                    elif self.cameraFunction == self.cameraSettings["toCenter"]:
                        self.rotina["visao"] = self.tr["visao"]["lookTarget"]
                else:
                    self.file.write("Target has already been visited" + "\n")
                    print("Target has already been visited")
                    self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos])
                    self.counts["ignoraVisao"] = self.time()
                    self.rotina["normal"] = self.tr["normal"]["checkMove"]
                    self.unic["base"] = 0
                    self.rotina["visao"] = self.tr["idle"]
                
        elif self.rotina["visao"] == self.tr["visao"]["pouso"]:
            self.desce.publish(Empty())
            self.counts["land"] = self.time
            self.file.write("Landing by the vision" + "\n")
            print("Landing by the vision")
            self.rotina["visao"] == self.tr["visao"]["pouso1"]

        elif self.rotina["visao"] == self.tr["visao"]["pouso1"]:
            if self.time() - self.counts["land"] > self.tempo["land"]:
                self.sobe.publish(Empty())
                self.counts["takeoff"] = self.time
                self.file.write("Take off by the vision" + "\n")
                print("Take off by the vision")
                self.rotina["visao"] == self.tr["visao"]["pouso2"]

        elif self.rotina["visao"] == self.tr["visao"]["pouso2"]:
            if self.time() - self.counts["takeoff"] > self.tempo["takeoff"]:
                self.upUAV(z=self.rotas["z"][self.pos])
                self.rotina["normal"] = self.tr["normal"]["checkMove"]
                self.unic["base"] = 0
                self.rotina["visao"] = self.tr["idle"]
                # Talvez precise modificiar a posicao do vetor de rotas para self.pos -= 1

    # Do a square -----------------------------------------------------------------------------------------------------
    def orbsRoutine(self, orbs):
        if self.estado == self.te["cheguei"]:
            if self.rotina["orbs"] == self.tr["orbs"]["start"]:
                self.iniciaOrb.publish(0xFF)
                self.file.write("direita" + "\n")
                print("direita")
                self.goToLocal(y=-0.3, t=3)
                self.rotina["orbs"] = self.tr["orbs"]["p1"]
            elif self.rotina["orbs"] == self.tr["orbs"]["p1"]:
                self.file.write("frente" + "\n")
                print("frente")
                self.goToLocal(x=0.3, t=3)
                self.rotina["orbs"] = self.tr["orbs"]["p2"]
            elif self.rotina["orbs"] == self.tr["orbs"]["p2"]:
                self.file.write("tras" + "\n")
                print("esquerda")
                self.goToLocal(y=0.3, t=3)
                self.rotina["orbs"] = self.tr["orbs"]["p3"]
            elif self.rotina["orbs"] == self.tr["orbs"]["p3"]:
                self.file.write("tras" + "\n")
                print("tras")
                self.goToLocal(x=-0.3, t=3)
                self.rotina["orbs"] = self.tr["orbs"]["p4"]
            elif self.rotina["orbs"] == self.tr["orbs"]["p4"]:
                self.file.write("frente" + "\n")
                print("frente")
                self.iniciaLog.publish(0XFF)
                self.goToLocal(x=0.3, t=3)
                self.rotina["orbs"] = self.tr["orbs"]["end"]

        if orbs == self.comEscala:
            self.file.write("Com escala" + "\n")
            print("Com escala")
            self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos]) # seguir a vida
            self.pos += 1
            self.rotina["orbs"] = 0
            self.rotina["normal"] = self.tr["normal"]["checkMove"]
            self.orientacao = self.tipoOrientacao["orbslam"]

    # What happen if orbslam die ---------------------------------------------------------------------------------------
    def lostOrbsRoutine(self):
        if self.time() - self.counts["perderOrbs"] < self.tempo["perderOrbs"]:
            if self.rotina["perdeuOrbs"] == self.tr["perdeuOrbs"]["start"]:
                self.file.write("tras recuperando orbs" + "\n")
                print("tras recuperando orbs")
                self.goToLocal(x=-0.2, t=3)
                self.rotina["perdeuOrbs"] = self.tr["perdeuOrbs"]["s1"]
            
            if self.estado == self.te["cheguei"]:
                if self.rotina["perdeuOrbs"] == self.tr["perdeuOrbs"]["s1"]:
                    self.file.write("frente recuperando orbs" + "\n")
                    print("frente recuperando orbs")
                    self.goToLocal(x=0.2, t=3)
                    self.rotina["perdeuOrbs"] = self.tr["perdeuOrbs"]["s1"]
        else:
            self.file.write("Lost orbslam Forever" + "\n")
            print("Lost orbslam Forever")
            self.goToLocal(flag=self.perdeuOrbsSempre)
            self.goToGlobal(x=self.rotas["x"][len(self.rotas["x"]) - 1], y=self.rotas["y"][len(self.rotas["y"]) - 1])

def main():
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()