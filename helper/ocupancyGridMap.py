# !/usr/bin/env python
# rosrun gmapping slam_gmapping scan:=/uav1/rplidar/scan _odom_frame:=uav1/gps_origin _base_frame:=uav1/fcu _map_frame:=map
import rospy
import sys
import time
import os
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int8MultiArray
import matplotlib.pyplot as plt
import numpy as np

def callback(mapmsg):
    map = mapmsg.data
    print(mapmsg.header)
    row,col,res = mapmsg.info.width, mapmsg.info.height, mapmsg.info.resolution
    map = np.array(map)
    map = map.reshape((row, col))

    a, b = [], []

    for i in range(len(map)):
        for j in range(len(map[i])):
            if map[i][j] == 100:
                a.append(round(i*res))
                b.append(round(j*res))

    plt.plot(a, b, ".k")
    plt.show()

def somethingCool():
    global mapdata
    mapdata = Int8MultiArray()
    rospy.init_node('test_name', anonymous=False)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    rospy.loginfo(mapdata)
    rospy.spin()


if __name__ == '__main__':
    try:
        somethingCool()
    except rospy.ROSInterruptException:
        pass