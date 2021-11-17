#include <iostream>
#include <vector>
#include <mutex>
#include <queue>
#include <deque>
#include <string>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

using namespace std;

std::string imuTopic_sub;
std::string imuTopic_pub;

std::mutex mtx;
std::deque<sensor_msgs::Imu> imuQueue_enu;
    
sensor_msgs::Imu imu2ENU(const sensor_msgs::Imu& imu_in){
    //vector<double> extRotV{0, 1, 0,
    //                       1, 0, 0,
    //                       0, 0, -1};
    //Eigen::Matrix3d extRot;
    //Eigen::Quaterniond extQRPY;
    //extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
    //extQRPY = Eigen::Quaterniond(extRot);
    //
    //sensor_msgs::Imu imu_out = imu_in;
    //// rotate acceleration
    //Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    //acc = extRot * acc;
    //imu_out.linear_acceleration.x = acc.x();
    //imu_out.linear_acceleration.y = acc.y();
    //imu_out.linear_acceleration.z = acc.z();
    //// rotate gyroscope
    //Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    //gyr = extRot * gyr;
    //imu_out.angular_velocity.x = gyr.x();
    //imu_out.angular_velocity.y = gyr.y();
    //imu_out.angular_velocity.z = gyr.z();
    //// rotate roll pitch yaw
    //Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    //Eigen::Quaterniond q_final = q_from * extQRPY;
    //imu_out.orientation.x = q_final.x();
    //imu_out.orientation.y = q_final.y();
    //imu_out.orientation.z = q_final.z();
    //imu_out.orientation.w = q_final.w();
    
    sensor_msgs::Imu imu_out = imu_in;

    imu_out.orientation.x =  imu_in.orientation.y;
    imu_out.orientation.y =  imu_in.orientation.x;
    imu_out.orientation.z = -imu_in.orientation.z;
    imu_out.orientation.w =  imu_in.orientation.w;
    
    imu_out.linear_acceleration.x =  imu_in.linear_acceleration.y;
    imu_out.linear_acceleration.y =  imu_in.linear_acceleration.x;
    imu_out.linear_acceleration.z = -imu_in.linear_acceleration.z;

    imu_out.angular_velocity.x =  imu_in.angular_velocity.y;
    imu_out.angular_velocity.y =  imu_in.angular_velocity.x;
    imu_out.angular_velocity.z = -imu_in.angular_velocity.z;

    return imu_out;
    };

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_ned) {
    std::lock_guard<std::mutex> lock(mtx);
    sensor_msgs::Imu imu_enu = imu2ENU(*imu_ned);
    imuQueue_enu.push_back(imu_enu);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "ned2enu");
    ros::NodeHandle nh;

    nh.param<std::string>("ned2enu/imuSubs", imuTopic_sub, "/vinebot_0/imu_rion"); 
    //nh.param<std::string>("ned2enu/imuSubs", imuTopic_sub, "/imu/data"); 
    nh.param<std::string>("ned2enu/imuPubl", imuTopic_pub, "/imu_enu"); 

    ros::Subscriber imu_ned = nh.subscribe(imuTopic_sub, 100, imu_callback);
    ros::Publisher imu_pub= nh.advertise<sensor_msgs::Imu>(imuTopic_pub,100);
    ROS_INFO("\033[1;32m----> NED to ENU started.\033[0m");
    while(ros::ok()){
        if(!imuQueue_enu.empty()){
            sensor_msgs::Imu *imu2pub = &imuQueue_enu.front();
            imu_pub.publish(*imu2pub);
            imuQueue_enu.pop_front();
        }

        ros::spinOnce();
    }

    
    return 0;
}
