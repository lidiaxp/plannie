import rospy
import math
import numpy as np
from obstacle_detector.msg import Obstacles
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

class identifyObstacle:
    def __init__(self):
        self.currentPosX, self.currentPosY = 1, 1
        # self.pix, self.piy = 1, 1
        self.posX, self.posY = [], []
        self.unic = 0

        self.pub = rospy.Publisher('/build_map', PoseArray, queue_size=1)

        rospy.init_node("obstacle")

        _ = rospy.Subscriber("/raw_obstacles", Obstacles, self.callbackObstacle)
        _ = rospy.Subscriber("/uav1/odometry/odom_main", Odometry, self.callbackPosicao)

    def callbackObstacle(self, data):
        self.posX, self.posY = [], []
        fp, lp = 0, 0
        for value in data.segments:
            fp = value.first_point
            lp = value.last_point
            
            thirdValueX = abs(fp.x - lp.x) 
            thirdValueY = abs(fp.y - lp.y) 
            if thirdValueX > thirdValueY: 
                # Horizontal, ou seja, preciso salvar a variacao em X
                vetorX = [fp.x, lp.x]
                new_vetorX = np.linspace(min(vetorX), max(vetorX), round(max([thirdValueX, thirdValueY])) + 1)
                [self.posX.append(round(v)) for v in new_vetorX]
                [self.posY.append(round((fp.y+lp.y)/2)) for v in new_vetorX]
            else: 
                # Vertical, ou seja, preciso salvar a variacao em Y
                vetorY = [fp.y, lp.y]
                new_vetorY = np.linspace(min(vetorY), max(vetorY), round(max([thirdValueX, thirdValueY])) + 1)
                [self.posY.append(round(v)) for v in new_vetorY]
                [self.posX.append(round((fp.x+lp.x)/2)) for v in new_vetorY]
            
        posx, posy = np.asarray(self.posX) + self.currentPosX, np.asarray(self.posY) + self.currentPosY
        AAA = PoseArray()
      
        for (v1, v2) in zip(posx, posy):
            A = Pose()
            A.position.x = v1
            A.position.y = v2
            AAA.poses.append(A)
        
        self.pub.publish(AAA)

    def callbackPosicao(self, odom):
        # Somar com a posicao X e Y do yaml
        self.currentPosX = round(odom.pose.pose.position.x) + 1
        self.currentPosY = round(odom.pose.pose.position.y) + 1


def main():
    identifyObstacle()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()