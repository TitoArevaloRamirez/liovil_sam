#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pandas as pd
from datetime import datetime
import os

filename = '~/map_'

def callback(data):

    rawData = []


    rate= rospy.Rate(5.0)

    for points in pc2.read_points(data, field_names = ('x', 'y', 'z', 'intensity'), skip_nans=True):
        x = points[0]
        y = points[1]
        z = points[2]
        intensity = points[3]
        timeStamp = data.header.stamp
        #print('x = %f;  y = %f; z = %f; intensity = %f'%(x, y, z, intensity))

        rawData.append([x, y, z, intensity, timeStamp])

    #print(np.shape(np.asarray(rawData).T))
    df = pd.DataFrame(np.asarray(rawData))
    df.to_csv((filename + str(data.header.seq) + '.txt'), mode = 'w', header=False, index=False)
    #print(filename + str(data.header.seq) + '.txt')



def ptCloud_listener():
    rospy.init_node('ptCloud_listener', anonymous=True)
    global filename
    nodeName = rospy.get_name();
    filename = rospy.get_param(nodeName+"/filename")
    print("\033[1;36m\n>>> Save Point Cloud Map started <<<\033[0m")

    now_datetime = datetime.now();
    dt_string = now_datetime.strftime("%Y-%m-%d-%H-%M")

    filename = filename + dt_string 
    isExist = os.path.exists(filename) 
    if not isExist:
        os.makedirs(filename)
        print("\033[0;36m\tDirectory created\033[0m")
        filename = filename + "/map_"

    print("\033[0;36m\tFilename: " + filename + "\033[0m")

    rospy.Subscriber('/ptCloud_save', PointCloud2, callback)
    #rospy.Subscriber('/laser_cloud_map', PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    ptCloud_listener()
