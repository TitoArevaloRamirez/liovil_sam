#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
#from camera_info_manager import CameraInfoManager

new_height = 1032
new_width = 1384

pub_leftInfo = rospy.Publisher('mapping/left/camera_info_usr', CameraInfo, queue_size=60);

pub_rightInfo = rospy.Publisher('mapping/right/camera_info_usr', CameraInfo, queue_size=60);

def callback_left(camInfo):
    camInfo.roi.x_offset = 0
    camInfo.roi.y_offset = 0
    camInfo.roi.height = 0
    camInfo.roi.width = 0
    camInfo.height = new_height
    camInfo.width = new_width
    pub_leftInfo.publish(camInfo)

def callback_right(camInfo):
    camInfo.roi.x_offset = 0
    camInfo.roi.y_offset = 0
    camInfo.roi.height = 0
    camInfo.roi.width = 0
    camInfo.height = new_height
    camInfo.width = new_width
    pub_rightInfo.publish(camInfo)


def camInfo_edit():
    rospy.init_node("camInfo_edit", anonymous=True)

    rospy.Subscriber('left/camera_info', CameraInfo, callback_left)
    rospy.Subscriber('right/camera_info', CameraInfo, callback_right)

    print("\033[1;36m>>> Camera Info Edit Started <<<\033[0m")

    rospy.spin()

if __name__ == '__main__':
    camInfo_edit()
