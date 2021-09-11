import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import euler_from_quaternion
import time
import math
import struct
import ctypes
from scipy import ndimage

import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class identifyObstacle3D:
    def __init__(self):
        self.currentPosX, self.currentPosY, self.currentPosZ, self.currentPosYaw = 2, 2, 2, 0
        self.count = 0
        self.unic = 0
        self.pub = rospy.Publisher('/build_map3D', PoseArray, queue_size=1)
        self.all = []
        self.obsX, self.obsY, self.obsZ = [], [], []
        self.t = time.time()

        self.number_of_sampling = 30

        rospy.init_node("obstacle3D")
        print("Start")
        
        _ = rospy.Subscriber("/uav1/velodyne/scan", PointCloud2, self.callbackObstacle)
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackPosicao)

    def callbackPosicao(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        
        if self.count == 0:
            self.lastYaw = yaw

        self.currentPosX = odom.pose.pose.position.x
        self.currentPosY = odom.pose.pose.position.y
        self.currentPosY = odom.pose.pose.position.z
        self.currentPosYaw = yaw

        self.count += 1

    def rotationMatrix(self, psi0, x1, y1, z1):
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
        return pos_local

    def callbackObstacle(self, data):
        print(time.time()-self.t)
        if self.count > 0:
            a4, a5, a6 = [], [], []
            a1, a2, a3 = [], [], []
            x, y, z = [], [], []
            matriz = np.zeros((101, 101))
            xyz = np.array([[0,0,0]])   
            gen = point_cloud2.read_points(data, skip_nans=True)
            int_data = list(gen)

            for x in int_data:
                if round(x[2]) + 1 > 0:
                    a4.append(round(x[0]))
                    a5.append(round(-x[1]))
                    a6.append(round(x[2]+1))

            pl = self.rotationMatrix(-0.71, a4, a5, a6)
            
            for i1, i2, i3 in zip(pl[0], pl[1], pl[2]):
                a1.append(i2)
                a2.append(i1)
                a3.append(i3)
                xyz = np.append(xyz,[[i2, i1, i3]], axis = 0)

            self.count += 1

            if 8<time.time()-self.t<13:
                ax = plt.axes(projection = "3d")
                ax.plot3D(a1, a2, a3, 'y.') 
                ax.plot3D([self.currentPosX], [self.currentPosY], [self.currentPosZ], ".r")
                ax.set_xlim(0,100) 
                ax.set_ylim(0,100) 
                ax.set_zlim(0,5) 
                ax.set_xlabel("x (m)" + str(self.currentPosX))
                ax.set_ylabel("y (m)" + str(self.currentPosY))
                ax.set_zlabel("z (m)" + str(self.currentPosZ))
                ax.view_init(50, -137)
                plt.pause(0.01)
                plt.show()


def main():
    identifyObstacle3D()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()