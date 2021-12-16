#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import NavSatFix
import pandas as pd

def callback(gps):

    rawData = []

    filename = '~/rawGPS.txt'
    gps_timeStamp = gps.header.stamp
    gps_latitude = gps.latitude
    gps_longitude = gps.longitude
    gps_altitude = gps.altitude

    rawData.append([gps_latitude, gps_longitude, gps_altitude, gps_timeStamp])

    #print(np.shape(rawData))
    df = pd.DataFrame(rawData)
    df.to_csv(filename, mode = 'a', header=False, index=False)


    print(gps.header)



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/vinebot_0/gps/fix',NavSatFix, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
