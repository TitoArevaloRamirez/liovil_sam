#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import pandas as pd
from datetime import datetime
import os

filename = "~/odom.txt"#rospy.get_param('~filename')#'~/rawGPS.txt'

def callback(odom):

    rawData = []

    odom_timeStamp = odom.header.stamp
    odom_x = odom.pose.pose.position.x
    odom_y = odom.pose.pose.position.y
    odom_z = odom.pose.pose.position.z

    q_x = odom.pose.pose.orientation.x
    q_y = odom.pose.pose.orientation.y
    q_z = odom.pose.pose.orientation.z
    q_w = odom.pose.pose.orientation.w

    rawData.append([odom_x, odom_y, odom_z, q_x, q_y, q_z, q_w, odom_timeStamp])

    #print(np.shape(rawData))
    df = pd.DataFrame(rawData)
    df.to_csv(filename, mode = 'a', header=False, index=False)


def odom_listener():
    rospy.init_node('odom_listener', anonymous=True)
    global filename
    nodeName = rospy.get_name();
    filename = rospy.get_param(nodeName+"/filename")
    print("\033[1;36m\n>>> " + nodeName + " started <<<\033[0m")

    now_datetime = datetime.now();
    dt_string = now_datetime.strftime("%Y-%m-%d-%H-%M")

    filename = filename + dt_string 
    isExist = os.path.exists(filename) 
    if not isExist:
        os.makedirs(filename)
        print("\033[0;36m\tDirectory created\033[0m")
        filename = filename + "/odom.txt"

    print("\033[0;36m\tFilename: " + filename + "\033[0m")

    rospy.Subscriber('/odom_save',Odometry, callback)

    rospy.spin()

if __name__ == '__main__':
    odom_listener()
