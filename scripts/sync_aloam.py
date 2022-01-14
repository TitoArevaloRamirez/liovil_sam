#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


pub_lidarOdom = rospy.Publisher('/odom_lidar_usr_sync', Odometry, queue_size=100);
pub_lidarPath = rospy.Publisher('/laser_odom_path_sync', Path, queue_size=100);

pub_cloudCorner = rospy.Publisher('/laser_cloud_corner_last_sync', PointCloud2, queue_size=100);
pub_cloudSurf = rospy.Publisher('/laser_cloud_surf_last_sync', PointCloud2, queue_size=100);
pub_cloudFull = rospy.Publisher('/velodyne_cloud_3_sync', PointCloud2, queue_size=100);



def syncCallback(lidarOdom_msg, cloudCorner_msg, cloudSurf_msg, cloudFull_msg, lidarPath_msg):
    cloudCorner_msg.header.stamp = lidarOdom_msg.header.stamp;
    cloudSurf_msg.header.stamp = lidarOdom_msg.header.stamp;
    cloudFull_msg.header.stamp = lidarOdom_msg.header.stamp;
    lidarPath_msg.header.stamp = lidarOdom_msg.header.stamp;

    pub_lidarOdom.publish(lidarOdom_msg);
    pub_cloudCorner.publish(cloudCorner_msg);
    pub_cloudSurf.publish(cloudSurf_msg);
    pub_cloudFull.publish(cloudFull_msg);
    pub_lidarPath.publish(lidarPath_msg);



def sync_aloam():
    rospy.init_node("sync_aloam", anonymous=True)

    lidarOdom = message_filters.Subscriber('/odom_lidar_usr', Odometry)
    lidarPath = message_filters.Subscriber('/laser_odom_path', Path)

    cloudCorner = message_filters.Subscriber('/laser_cloud_corner_last', PointCloud2)
    cloudSurf = message_filters.Subscriber('/laser_cloud_surf_last', PointCloud2)
    cloudFull = message_filters.Subscriber('/velodyne_cloud_3', PointCloud2)

    ts = message_filters.ApproximateTimeSynchronizer([lidarOdom, cloudCorner, cloudSurf, cloudFull, lidarPath], 10, 0.1, allow_headerless=True)
    ts.registerCallback(syncCallback)
    rospy.spin()

if __name__ == '__main__':
    sync_aloam()
