#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

pub_rtkOdom = rospy.Publisher('/rtk_odometry', Odometry, queue_size=60);

def callback(odom):

    odom_x = odom.pose.pose.position.x + 0
    odom_y = odom.pose.pose.position.y + 0
    odom_z = odom.pose.pose.position.z + 0.0

    odom.pose.pose.position.x = odom_x
    odom.pose.pose.position.y = odom_y
    odom.pose.pose.position.z = odom_z

    pub_rtkOdom.publish(odom)


def rtk_simulator():
    rospy.init_node("rtk_simulator", anonymous=True)
    print("\033[1;36m\n>>> RTK simulator started <<<\033[0m")

    rospy.Subscriber('/gps_odometry', Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    rtk_simulator()
