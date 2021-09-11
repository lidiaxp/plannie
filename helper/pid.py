import rospy
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion
import time
import math

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt

class MovePID:
    def __init__(self):
        rospy.init_node('pid_controller_initial', anonymous=True)

        self.velocity_publisher = rospy.Publisher('/drone1/cmd_vel', Twist, queue_size=10)
 
        self.pose = Odometry()
        self.rate = rospy.Rate(1)
        self.tolerancia = 0.25

        self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw = 0, 0, 0, 0
        
        # self.count = 0
        # self.unic = 0
        # self.pub = rospy.Publisher('/build_map3D', PoseArray, queue_size=1)
        # self.all = []
        # self.obsX, self.obsY, self.obsZ = [], [], []
        # self.t = time.time()

        print("Start")
        
        _ = rospy.Subscriber("/pid/cmd_vel", Pose, self.callbackMove)
        _ = rospy.Subscriber('/drone1/ground_truth/state', Odometry, self.callbackPosicao)

    def callbackPosicao(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        
        self.currentPosX = odom.pose.pose.position.x
        self.currentPosY = odom.pose.pose.position.y
        self.currentPosZ = odom.pose.pose.position.z
        self.currentPosYaw = yaw

        # print(self.currentPosX)
        # print(self.currentPosY)

    def rotationMatrix(self, psi0, x1, y1, z1):
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
        return pos_local

    def callbackMove(self, data):
        print("LISTEN")
        self.move2goal(data.position.x, data.position.y)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.currentPosX), 2) + pow((goal_pose.y - self.currentPosY), 2))
   
    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
   
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.currentPosY, goal_pose.x - self.currentPosX)
   
    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.currentPosYaw)

    def velocity(self, goal_pose, vel=0.5):
        x = abs(self.currentPosX - goal_pose.x)
        y = abs(self.currentPosY - goal_pose.y)
        sinalX, sinalY = 1, 1
        percent = x/y if x > y else y/x        
        if x > y:
            percent = x / y
            # print("X")
            # print(vel/percent)
            if self.currentPosX > goal_pose.x:
                sinalX = -1
            if self.currentPosY > goal_pose.y:
                sinalY = -1
            return vel*sinalX, vel/percent*sinalY
        else:
            percent = y / x
            # print("Y")
            # print(vel/percent)
            if self.currentPosX > goal_pose.x:
                sinalX = -1
            if self.currentPosY > goal_pose.y:
                sinalY = -1
            return vel/percent*sinalX, vel*sinalY
   
    def move2goal(self, x, y):
        goal_pose = Pose().position
        # print("Indo para " + str(goal_pose))
   
        goal_pose.x = x
        goal_pose.y = y
   
        vel_msg = Twist()
   
        while self.euclidean_distance(goal_pose) >= self.tolerancia:
            vel_msg.linear.x = self.linear_vel(goal_pose)/10 # self.velocity(goal_pose)[0] #self.linear_vel(goal_pose)/8
            vel_msg.linear.y = 0 #self.velocity(goal_pose)[1]
            vel_msg.linear.z = 0
   
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)/10
   
            self.velocity_publisher.publish(vel_msg)
  
            # self.rate.sleep()
   
        # print("SAIU")
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
  
        # rospy.spin()

def main():
    MovePID()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()