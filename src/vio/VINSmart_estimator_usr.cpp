#include <ros/ros.h>
#include <map>
#include <cmath>
#include <vector>
#include <fstream>
#include <string>
#include <iostream>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include <vio/FixedLagSmoother.h>
#include <vio/BatchFixedLagSmoother.h>
#include <vio/time_based_retriever.h>
#include <liovil_sam/StereoFeatureMatches.h>
#include <liovil_sam/landmarks.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "vio/CameraPoseVisualization.h"

#include <thread>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

typedef gtsam::SmartStereoProjectionPoseFactor SmartStereoFactor; // (cam_noise_model, params, body_P_sensor)

// global variable
// file output setup
std::string res_lidar_Path = "test_lidar.txt";
std::string res_rt_Path = "test_rt.txt";
std::string res_rt_imuRate_Path = "test_rtIMURATE.txt";
std::string res_bias = "bias.txt";
std::string kf_Path = "keyframes.txt";
std::string time_Path = "time.txt";
std::ofstream outFile_result_lidar(res_lidar_Path);
std::ofstream outFile_result_rt(res_rt_Path);
std::ofstream outFile_result_rt_imuRate(res_rt_imuRate_Path);
std::ofstream outFile_result_bias(res_bias);
std::ofstream outFile_kf(kf_Path);
std::ofstream outFile_time(time_Path);

// Uncomment line below to use the CombinedIMUFactor as opposed to the standard ImuFactor.
//#define USE_COMBINED

// candi_feat_info
struct candi_feat_info {
    candi_feat_info() {}
    candi_feat_info(uint64_t numFrameIn, uint64_t recentFrameIDIn) :
                    numFrame(numFrameIn), recentFrameID(recentFrameIDIn) {}
    uint64_t numFrame;
    uint64_t recentFrameID;
};
// landmark
struct landmark {
    uint64_t Lid;
    std::vector<uint64_t> Win_KidSet;
    std::vector<int> Win_KindexSet; // index to the measurement
    int Ksize;// size of Win_KidSet
    gtsam::Point3 Position;
    bool Win_added;
    SmartStereoFactor::shared_ptr smartfactor;
};
// keyframe
uint64_t kid_counter = 0;
struct keyframe {
    uint64_t Kid;
    ros::Time timestamp;
    // robust landmarks
    std::vector< Eigen::Vector2d > landLeft, landRight; // Eigen::aligned_allocator<Eigen::Vector2d>
    std::vector<uint64_t> LidSet;
    std::vector<bool> Lvalid;
    uint32_t landmarkSize; // equal to the number of truth in Lvalid
    // method
    keyframe(): Kid(kid_counter++) {}
    ~keyframe() {}
    keyframe(const keyframe &kf) {
        Kid = kf.Kid;
        timestamp = kf.timestamp;
        landLeft = kf.landLeft;
        landRight = kf.landRight;
        LidSet = kf.LidSet;
        Lvalid = kf.Lvalid; 
        landmarkSize = kf.landmarkSize;
    }
    keyframe& operator=(const keyframe& kf) {
        // if constructed just before assignment, the global counter is incremened which should not happen
        if (&kf != this)
            kid_counter--;
        Kid = kf.Kid;
        timestamp = kf.timestamp;
        landLeft = kf.landLeft;
        landRight = kf.landRight;
        LidSet = kf.LidSet;
        Lvalid = kf.Lvalid;
        landmarkSize = kf.landmarkSize;
        return *this;
    }
};

// For keyframe output
std::ostream &operator<<(std::ostream &os, keyframe const &m) {
    // to file
    std::ostringstream osFile;
    osFile << std::setprecision(16) << m.timestamp.toNSec()/1.0e9 << "\n";
    std::string str = osFile.str();
    outFile_kf << str;
    // landmarks
    for (uint32_t i=0;i<m.landmarkSize;i++)
        // to file
        outFile_kf << m.landLeft[i](0) << ", " << m.landLeft[i](1) << ", " << m.landRight[i](0) << ", " << m.landRight[i](1) << "\n";
    outFile_kf.close();
    outFile_kf.open(kf_Path, std::ios_base::app);
    return os;
}
// state_info
struct state_info {
    uint64_t Kid;
    ros::Time msgtime;
    //nav state information
    gtsam::Pose3 pose;                        //3x3, vec3/ or 4x4 matrix
    gtsam::noiseModel::Diagonal::shared_ptr pose_noise;     //6x6  xyz+rpy
    gtsam::Vector3 velocity;                  // vec 3
    gtsam::noiseModel::Diagonal::shared_ptr velocity_noise; //3x3  vx,vy,vz
    gtsam::imuBias::ConstantBias imu_bias;
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise;     //6x6; acc + gyro
};
// For eigen output
Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
std::string sep = "\n----------------------------------------\n";


class KeyFrames_selector{

protected:
    // ros
    ros::Subscriber imuOdom_sub;
    ros::Subscriber f_sub;
    // publisher
    ros::Publisher odom_imu_pub;

};



// main
int main(int argc, char **argv) {
    ros::init(argc, argv, "smart_smoother");
    ros::NodeHandle nh("~");

    ros::MultiThreadedSpinner spinner(4);

    ROS_INFO("\033[1;36m \n >>> Visual Inertial Odometry (VIO) <<< \033[0m");
    VINSmart_estimator smartS_estimator(nh); // = new VINSmart_estimator(nh);


    //spinner.start();
    std::thread loopthread(&VINSmart_estimator::optimizeLoop, &smartS_estimator);

    spinner.spin();


    loopthread.join();


    //smartS_estimator.optimizeLoop();
    return 0;
}


