#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import *


pub_odomLeftCam = rospy.Publisher('/leftCam_odometry', Odometry, queue_size=60);
pub_odomRightCam = rospy.Publisher('/rightCam_odometry', Odometry, queue_size=60);

BaseLine  = 0.247279067378212 #x, y, z, w

Q_Lidar2Cam = [0.5034721, 0.5020589, 0.501271, 0.4931329] #x, y, z, w
T_Lidar2Imu = np.array([0.011356, 0.002352, 0.08105])

Q_Imu2LeftCam = [-0.49276446989981, 0.500896743794452, -0.501684436262218, 0.503096384820381];
#Q_Imu2LeftCam = [0, 0, 0, 1];
T_Imu2LeftCam = np.array([-0.0870551272380779, +0.107604788194452, -0.0180391607070435]) #x, y, z// See VIO config file (posi_I_LC - T_I_Lidar ) 
T_Imu2RightCam = np.array([-0.0870551272380779, +0.107604788194452 - BaseLine, -0.0180391607070435]) #x, y, z// See VIO config file (posi_I_LC - T_I_Lidar ) 


def callback(odom):

    odomCam_left = Odometry();
    odomCam_left.header.frame_id = "map"
    odomCam_left.child_frame_id = "left_camera_usr"

    odomCam_right= Odometry();
    odomCam_right.header.frame_id = "map"
    odomCam_right.child_frame_id = "right_camera_usr"

    t_x = odom.pose.pose.position.x
    t_y = odom.pose.pose.position.y
    t_z = odom.pose.pose.position.z

    q_x = odom.pose.pose.orientation.x
    q_y = odom.pose.pose.orientation.y
    q_z = odom.pose.pose.orientation.z
    q_w = odom.pose.pose.orientation.w

    Q_lidar = [q_x, q_y, q_z, q_w]
    #Q_imu = quaternion_multiply(Q_lidar, Q_Lidar2Imu)
    Q_leftCam = quaternion_multiply(Q_lidar, Q_Lidar2Cam)
    #Q_leftCam = quaternion_multiply(Q_Imu2LeftCam, Q_imu)

    T_imu = np.array([t_x, t_y, t_z]) + T_Lidar2Imu
    T_leftCam = T_imu + T_Imu2LeftCam
    T_rightCam = T_imu + T_Imu2RightCam

    #print("left camera");
    #print(T_leftCam);

    #print("right camera");
    #print(T_rightCam);

    odomCam_left.pose.pose.position.x = T_leftCam[0]
    odomCam_left.pose.pose.position.y = T_leftCam[1]
    odomCam_left.pose.pose.position.z = T_leftCam[2]

    odomCam_left.pose.pose.orientation.x = Q_leftCam[0]
    odomCam_left.pose.pose.orientation.y = Q_leftCam[1]
    odomCam_left.pose.pose.orientation.z = Q_leftCam[2]
    odomCam_left.pose.pose.orientation.w = Q_leftCam[3]
    pub_odomLeftCam.publish(odomCam_left);

    odomCam_right.pose.pose.position.x = T_rightCam[0]
    odomCam_right.pose.pose.position.y = T_rightCam[1]
    odomCam_right.pose.pose.position.z = T_rightCam[2]

    odomCam_right.pose.pose.orientation.x = Q_leftCam[0]
    odomCam_right.pose.pose.orientation.y = Q_leftCam[1]
    odomCam_right.pose.pose.orientation.z = Q_leftCam[2]
    odomCam_right.pose.pose.orientation.w = Q_leftCam[3]
    pub_odomRightCam.publish(odomCam_right);

def lidar2CameraPose():
    rospy.init_node('lidar2CameraPose', anonymous=True)

    rospy.Subscriber('/odom_lidar',Odometry, callback)

    print("\033[1;36m\n>>> Lidar 2 Camera Odometry Started<<<\033[0m")

    rospy.spin()

if __name__ == '__main__':
    lidar2CameraPose()
