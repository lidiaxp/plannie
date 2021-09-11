# -*- coding: utf-8 -*-
import rospy
import math
from gps_common.msg import GPSFix
import matplotlib.pyplot as plt
from haversine import haversine
import time

class gpsRTK:
    def __init__(self):
        self.unic = 0

        self.unic = 0
        self.posicao0 = {"x": 0, "y": 0}
        self.currentPosX, self.currentPosY = 0, 0

        self.x = []
        self.y = []
        self.tempo = time.time()

        rospy.init_node("GPS_RTK")

        _ = rospy.Subscriber("/emlid/extended_fix", GPSFix, self.callbackGPSRTK)

    def callbackGPSRTK(self, data):
        if math.isnan(data.latitude) == False and data.latitude != 0:
            if self.unic == 0:
                self.posicao0["x"] = data.latitude
                self.posicao0["y"] = data.longitude
                self.unic = 1

            elif self.unic == 1:
                self.currentPosY = haversine([self.posicao0["x"], self.posicao0["y"]], [data.latitude, self.posicao0["y"]])*1000
                self.currentPosX = haversine([self.posicao0["x"], self.posicao0["y"]], [self.posicao0["x"], data.longitude])*1000
                
                if data.latitude < self.posicao0["x"]: self.currentPosX *= -1
                if data.longitude < self.posicao0["y"]: self.currentPosY *= -1

                self.x.append(self.currentPosX)
                self.y.append(self.currentPosY)
            
                if 60 > time.time() - self.tempo > 50:
                    plt.plot(self.x, self.y, ".r")
                    plt.show()

def main():
    gpsRTK()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()