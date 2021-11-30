import sys
from std_srvs.srv import Trigger, SetBool, SetBoolRequest
from mrs_msgs.srv import ReferenceStampedSrv, ReferenceStampedSrvResponse, ReferenceStampedSrvRequest, StringRequest, String, Float64Srv, Float64SrvRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import rospy
import math
from dynamic_reconfigure.srv import ReconfigureRequest, Reconfigure
from dynamic_reconfigure.parameter_generator import *
from geometry_msgs.msg import Twist

# from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from gazebo_msgs.srv import GetModelState
from helper.utils import rotacionar

from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
import pygame

sample_rate = 48000
pygame.mixer.pre_init(sample_rate, -16, 1, 1024)
pygame.init()

class colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def diegoVoice(audio):
    # print("Falando")
    pygame.mixer.music.load(str(audio) + '_mod.mp3')
    pygame.mixer.music.play()

def decolagemInicial():
    rospy.wait_for_service("/uav1/mavros/cmd/arming")
    try:
        ola = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
        req = CommandBoolRequest()
        req.value = 1
        resp = ola(req)
        # rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)
    
    rospy.wait_for_service("/uav1/mavros/set_mode")
    try:
        ola = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)
        req = SetModeRequest()
        req.base_mode = 0
        req.custom_mode = "offboard"
        resp = ola(req)
        # rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

    rospy.wait_for_service("/uav1/control_manager/use_safety_area")
    try:
        ola = rospy.ServiceProxy("/uav1/control_manager/use_safety_area", SetBool)
        req = SetBoolRequest()
        req.data = False
        resp = ola(req)
        # rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

    rospy.wait_for_service("/uav1/control_manager/set_min_height")
    try:
        ola = rospy.ServiceProxy("/uav1/control_manager/set_min_height", Float64Srv)
        req = Float64SrvRequest()
        req.value = 0
        resp = ola(req)
        rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

def set_vio(estimador="BARO"):
    rospy.wait_for_service("/uav1/odometry/change_alt_estimator_type_string")
    try:
        ola = rospy.ServiceProxy("/uav1/odometry/change_alt_estimator_type_string", String)
        req = StringRequest()
        req.value = estimador
        resp = ola(req)
        # rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

def euclidiana(x, y, z=0, currentPosX=0, currentPosY=0, currentPosZ=0, currentPosYaw=0, yaw=0):
    vel = math.sqrt(math.pow(currentPosX - x, 2) + math.pow(currentPosY - y, 2) + math.pow(currentPosZ - z, 2))
    # if vel < 2:
    #     vel = 2
    return vel

class parametroDouble: 
    def __init__(self, name, value):
        self.name = name
        self.value = value

def setDoubleParameter(option=["kiwxy"], value=[0.1]):
    # option can be kiwxy, kibxy, km
    rospy.wait_for_service("/uav1/control_manager/mpc_controller/set_parameters")
    try:
        ola = rospy.ServiceProxy("/uav1/control_manager/mpc_controller/set_parameters", Reconfigure)
        req = ReconfigureRequest()
        req.config.doubles = []
        for a, b in zip(option, value):
            parameter = parametroDouble(a, b)
            req.config.doubles.append(parameter)
        resp = ola(req)
        # rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

def addRotaPonto(ponto, vx, vy, rotax, rotay, rotaz, rotayaw, vz=3.2, vyaw=0):
    rotax.insert(ponto, vx)
    rotay.insert(ponto, vy)
    rotaz.insert(ponto, vz)
    rotayaw.insert(ponto, vyaw)
    return rotax, rotay, rotaz, rotayaw

def dist_euclidiana(v1, v2):
	dim, soma = len(v1), 0
	for i in range(dim):
		soma += math.pow(v1[i] - v2[i], 2)
	return math.sqrt(soma)

def addRota(vx, vy, rotax, rotay, rotaz, rotayaw, vz=3.2, vyaw=0):
    rotax.append(vx)
    rotay.append(vy)
    rotaz.append(vz)
    rotayaw.append(vyaw)
    return rotax, rotay, rotaz, rotayaw

def addTag(tag, rotax, rotay, rotaz, rotayaw):
    rotax.append(tag)
    rotay.append(500)
    rotaz.append(500)
    rotayaw.append(500)
    return rotax, rotay, rotaz, rotayaw

def addTagPonto(ponto, tag, rotax, rotay, rotaz, rotayaw):
    rotax.insert(ponto, tag)
    rotay.insert(ponto, 500)
    rotaz.insert(ponto, 500)
    rotayaw.insert(ponto, 500)
    return rotax, rotay, rotaz, rotayaw

def logStateMachine(frase, condicao, extra=None):
    if condicao == 0 and len(frase) > 0: print(frase)
    return 1 if extra == None else 1, 1
    
def melhorRota(x, y, posX, posY):
    camFinalX = [posX]
    camFinalY = [posY]
    distancias = []

    while len(x) >= 1:
        d = float("inf")
        coord, index = [], 0
        for i in range(len(x)):
            dist = dist_euclidiana([camFinalX[-1], camFinalY[-1]], [x[i], y[i]])
            if dist < d:
                index = i
                coord = [x[i], y[i]]
                d = dist
        del x[index]
        del y[index]

        distancias.append(d)
        camFinalX.append(coord[0])
        camFinalY.append(coord[1])
    
    # print("Caminho")
    # print(camFinalX[1:])
    # print(camFinalY[1:])
    # if len(camFinalX[1:]) == 3: print("Melhor caminho definido")
    # else: print("Nao foi encontrado todas as bases")

    return camFinalX[1:], camFinalY[1:]

def takeoff():
    rospy.wait_for_service("/uav1/uav_manager/takeoff")
    try:
        ola = rospy.ServiceProxy("/uav1/uav_manager/takeoff", Trigger)
        ola()
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

def land():
    rospy.wait_for_service("/uav1/uav_manager/land")
    try:
        ola = rospy.ServiceProxy("/uav1/uav_manager/land", Trigger)
        ola()
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)

def andarGlobal(x, y, z, rand, currentPosX, currentPosY, currentPosZ, currentPosYaw, vel=0.5, rotacionar=False, MPC=1):
    if MPC:
        rospy.wait_for_service("/uav1/control_manager/reference")
        if rotacionar:
            rand = rotacionar(currentPosX, currentPosY, x, y)
        # rand = 1.57 #rotacionar(currentPosX, currentPosY, x, y)
        
        try:
            ola = rospy.ServiceProxy("/uav1/control_manager/reference", ReferenceStampedSrv)
            req = ReferenceStampedSrvRequest()
            req.reference.position.x = x 
            req.reference.position.y = y 
            req.reference.position.z = z
            # print(z)
            # print("------------------------")
            req.reference.heading = rand
            resp = ola(req)
            print("x: " + str(req.reference.position.x))
            print("y: " + str(req.reference.position.y))
            print("z: " + str(req.reference.position.z))
            # print("rand: " + str(req.reference.heading))
            # rospy.loginfo(resp)
        except rospy.ServiceException as e:
            print("Falha na chamada de servico: %s"%e)

        return euclidiana(x, y, z, currentPosX, currentPosY, currentPosZ, currentPosYaw) 
    else:
        control_cmd(x, y, z, rand, currentPosX, currentPosY, currentPosZ, currentPosYaw, vel=vel)

def andarLocal(x, y, z, rand, currentPosX, currentPosY, currentPosZ, currentPosYaw):
    rospy.wait_for_service("/uav1/control_manager/reference")
    try:
        ola = rospy.ServiceProxy("/uav1/control_manager/reference", ReferenceStampedSrv)
        req = ReferenceStampedSrvRequest()
        req.reference.position.x = currentPosX - x
        req.reference.position.y = y + currentPosY
        req.reference.position.z = z + currentPosZ
        req.reference.heading = rand + currentPosYaw
        resp = ola(req)
        # rospy.loginfo(resp)
    except rospy.ServiceException as e:
        print("Falha na chamada de servico: %s"%e)
    
    return euclidiana(x, y, z, currentPosX, currentPosY, currentPosZ, currentPosYaw) * 0.7

def zerarTwist(valorTwist):
    valorTwist.angular.x = 0
    valorTwist.angular.y = 0
    valorTwist.angular.z = 0
    valorTwist.linear.x = 0
    valorTwist.linear.y = 0
    valorTwist.linear.z = 0

    return valorTwist

def criar_reta(x1, y1, x2, y2):
    delta_y = y2 - y1
    delta_x = x2 - x1
    
    if delta_x == 0:
        m = 0
    else:
        m = delta_y / delta_x # equivalente a a
        
    angulo = math.atan2(delta_x, delta_y)
    n = y2 - (m * x2)     # equivalente a c
    # b sempre vai ser -1

    return m, n, angulo

def sleeping(t=5):
    rospy.sleep(t)

def time():
    return rospy.get_rostime().secs + (rospy.get_rostime().nsecs/1e9)

def attach(letra_da_base, bool_detach = True):
    """Inputs:
        letra_da_base: str = letra da base indicada no bloco (eg: 'A', 'B', etc)
        bool_detach: bool =     True, para fazer detach
                                False, para fazer attach""" 
    letra_da_base = letra_da_base.upper()
    
    uav, uav_link = "uav1", "base_link"
    equipment, equipment_link = "equipment" + letra_da_base, "link_" + letra_da_base
    reference = 'ground_plane'

    if bool_detach:
        attach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        done_str = (colors.HEADER + 'Pacote entregue\n' + colors.ENDC)
    else:
        get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        uav_coord = get_coordinates(uav, reference)
        equipment_coord = get_coordinates(equipment, reference)

        dx = abs(uav_coord.pose.position.x - equipment_coord.pose.position.x)
        dy = abs(uav_coord.pose.position.y - equipment_coord.pose.position.y)
        dz = abs(uav_coord.pose.position.z - equipment_coord.pose.position.z)

        if dx > 0.2 or dy > 0.2 or dz > 1.0:
            print('Not close enough to perform attach. Exiting attach.')
            return False

        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        done_str = (colors.HEADER + "Pacote acoplado" + colors.ENDC)
        # done_str = (colors.HEADER + "Pacote " + str(letra_da_base) + " acoplado" + colors.ENDC)
        setDoubleParameter(option=["kqxy", "kiwxy_lim"], value=[8.0, 9.0])

    attach_srv.wait_for_service()
    req = AttachRequest()

    req.model_name_1 = uav
    req.link_name_1 = uav_link
    req.model_name_2 = equipment
    req.link_name_2 = equipment_link

    try:
        attach_srv.call(req)
        print(done_str)
        return True
    except Exception as e:
        print(e)
        print('Something went wrong. Exiting attach.')
        return False

def velocity(x_goal, y_goal, z_goal, currentX, currentY, currentZ, vel=0.5):
    x = abs(currentX - x_goal)
    y = abs(currentY - y_goal)
    z = abs(currentZ - z_goal)
    sinalX, sinalY, sinalZ = 1, 1, 1
    if currentX > x_goal:
        sinalX = -1
    if currentY > y_goal:
        sinalY = -1
    if currentZ > z_goal:
        sinalZ = -1
    if x >= y and x >= z:
        percent1 = x / y
        percent2 = x / z
        # print("X")
        return vel*sinalX, vel/percent1*sinalY, vel/percent2*sinalZ
    elif y >= x and y >= z:
        percent1 = y / x
        percent2 = y / z
        # print("Y")
        return vel/percent1*sinalX, vel*sinalY, vel/percent2*sinalZ
    else:
        percent1 = z / x
        percent2 = z / y
        # print("Z")
        return vel/percent1*sinalX, vel/percent2*sinalY, vel*sinalZ

def control_cmd(x_goal, y_goal, z_goal, yaw_goal, currentX, currentY, currentZ, currentYaw, vel=0.5, topicPub="/cmd_vel"):
    cmd_vel = rospy.Publisher(topicPub, Twist, queue_size=1)
    t_msg = Twist()
    t_msg.linear.x, b, c = velocity(x_goal, y_goal, z_goal, currentX, currentY, currentZ, vel=0.5)

    cmd_vel.publish(t_msg)