#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import NavSatFix

pub_gps = rospy.Publisher('/gps/fix', NavSatFix, queue_size=60);

def callback(gps):
    gps.header.frame_id = "gps_link"
    pub_gps.publish(gps)


def gps_listener():
    rospy.init_node("gps_listener", anonymous=True)
    nodeName = rospy.get_name();
    print("\033[1;36m\n>>> GPS rename frame_id started <<<\033[0m")

    rospy.Subscriber('/gps_save',NavSatFix, callback)

    rospy.spin()

if __name__ == '__main__':
    gps_listener()
