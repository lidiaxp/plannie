# -*- coding: utf-8 -*-
import rospy
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


class OctomapChecker:
    def __init__(self):
        self.obs_x, self.obs_y, self.obs_z, self.obs_xyz = [], [], [], []
        self.curr_pos_x, self.curr_pos_y, self.curr_pos_z, self.curr_pos_yaw = [], [], [], []
        self.path_x, self.path_y, self.path_z, self.r_speeds = [], [], [], []

        # Subscribers
        rospy.Timer(rospy.Duration(0.1), self.tick)
        _ = rospy.Subscriber(f"/uav1/odometry/odom_main", Odometry, self.callbackPosicao)
        _ = rospy.Subscriber(f"/occupied_cells_vis_array", MarkerArray, self.callbackBuildMap3D_octomap)
        _ = rospy.Subscriber(f"/lrs/last_path", MultiDOFJointTrajectory, self.callbackLastPaths)

        self.new_path_pub = rospy.Publisher("//lrs/new_paths", MultiDOFJointTrajectory, queue_size=1)


    # ---------------------------- Loop :3 ----------------------------------
    def tick(self, data):
        # TODO check collision and send pub
        is_collision = check_collision()

        if is_collision:
            positions, _, reference_speeds = prepare_trajectory(
                define_yaml=False,
                options=['rain_forest', 24],
                plot_graph=0
            )
            pos_x = positions[0, :]
            pos_y = positions[1, :]
            pos_z = positions[2, :]
            new_path_x, new_path_y, new_path_z, new_r_speed = lrs_reduce_nodes(pos_x, pos_y, pos_z, reference_speeds)

            # TODO just get from the current node

            traj_msg = MultiDOFJointTrajectory()
            for i in range(new_path_x):
                point = MultiDOFJointTrajectoryPoint()

                trans = Transform()
                speed = Twist()

                trans.translation.x = new_path_x[i]
                trans.translation.y = new_path_y[i]
                trans.translation.z = new_path_z[i]

                speed.linear.x = new_r_speed[i]

                point.transforms.append(trans)
                point.velocities.append(speed)

            self.new_path_pub.publish(traj_msg)


    def callbackPosicao(self, odom):
        _, _, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        
        self.curr_pos_x = odom.pose.pose.position.x
        self.curr_pos_y = odom.pose.pose.position.y 
        self.curr_pos_z = odom.pose.pose.position.z 
        self.curr_pos_yaw = yaw


    def callbackBuildMap3D_octomap(self, data):
        for marker in data.markers:
            positions = marker.points
            for position in positions:
                round_pos_x = round(position.x)
                round_pos_y = round(position.y)
                round_pos_z = round(position.z)
                pos_to_add = [round_pos_x, round_pos_y, round_pos_z]

                if pos_to_add not in self.abc:
                    self.obs_x.append(round_pos_x)
                    self.obs_y.append(round_pos_y)
                    self.obs_z.append(round_pos_z)
                    self.obs_xyz.append(pos_to_add)

    
    def callbackLastPaths(self, data):
        self.path_x, self.path_y, self.path_z, self.r_speeds = [], [], [], []
        for point in data.points:
            for transform in point.transforms:
                self.path_x.append(transform.translation.x)
                self.path_y.append(transform.translation.y)
                self.path_z.append(transform.translation.z)

            for transform in point.velocities:
                self.r_speeds.append(transform.linear.x)


def main():
    rospy.init_node("check_octomap_collision")
    OctomapChecker()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    plt.show()


if __name__ == "__main__":
    main()
