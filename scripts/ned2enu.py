#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from tf.transformations import *
from geometry_msgs.msg import Quaternion

pub_imu = rospy.Publisher('/imu_enu', Imu, queue_size=60);

R = np.array([[0, 1, 0],
               [1, 0, 0],
               [0, 0, -1]]);
R_q = Quaternion(0.7071068, 0.7071068, 0, 0)

def callback(imu):

    # ned
    x = imu.orientation.x
    y = imu.orientation.y
    z = imu.orientation.z
    w = imu.orientation.w

    linAcc_ned = np.array([0, 0, 0]);
    angVel_ned = np.array([0, 0, 0]);

    q_ned = Quaternion(x, y, z, w);


    angVel_ned[0] = imu.angular_velocity.x
    angVel_ned[1] = imu.angular_velocity.y
    angVel_ned[2] = imu.angular_velocity.z

    linAcc_ned[0] = imu.linear_acceleration.x
    linAcc_ned[1] = imu.linear_acceleration.y
    linAcc_ned[2] = imu.linear_acceleration.z


    # transform

    linAcc_enu = R.dot(linAcc_ned);
    angVel_enu = R.dot(angVel_ned);

    q_enu = quaternion_multiply([0.7071068, 0.7071068, 0, 0], [x, y, z, w]);

    # enu

    imu.orientation.x = q_enu[1]
    imu.orientation.y = q_enu[2]
    imu.orientation.z = q_enu[3]
    imu.orientation.w = q_enu[2]
    imu.angular_velocity.x = angVel_enu[0]
    imu.angular_velocity.y = angVel_enu[1]
    imu.angular_velocity.z = angVel_enu[2]
    imu.linear_acceleration.x = linAcc_enu[0]
    imu.linear_acceleration.y = linAcc_enu[1]
    imu.linear_acceleration.z = linAcc_enu[2]

    pub_imu.publish(imu)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/vinebot_0/imu_rion', Imu, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
