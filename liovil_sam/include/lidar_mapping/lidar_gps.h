#ifndef MODULES_LIDAR_GPS_H_
#define MODULES_LIDAR_GPS_H_
#include <time.h>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam_unstable/slam/BiasedGPSFactor.h>

#include "lidar_mapping/geodetic_conv.hpp"
#include "lidar_mapping/time_based_retriever.h"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace beyond
{
namespace lidar_mapping
{
class lidarGPS
{
    typedef Eigen::Matrix<double, 3, 1> GPS_MSG;
  public:
    lidarGPS(ros::NodeHandle &nh);
    void odom_lidar_callback(const nav_msgs::Odometry::ConstPtr &input_odometry);
    void odom_gps_callback(const nav_msgs::Odometry::ConstPtr &input_odometry);

  private:
    bool optimize = false;
    int lidar_scan_id;
    int loop_constriant_optimized_id;
    float scanPeriod;
    Eigen::Matrix4f imu2lidar;
    std::vector<float> imu2lidar1;
    gtsam::NonlinearFactorGraph *graph;
    TimeBasedRetriever<Eigen::Matrix4f> gps_msg_buffer;
    gtsam::ISAM2Params paramISAM2;
    gtsam::ISAM2 isam2;
    gtsam::Values initial_values;
    gtsam::Values post_values;
    gtsam::Pose3 last_lidar_pose;

    // error model
    gtsam::noiseModel::Diagonal::shared_ptr first_scan_noise;
    gtsam::noiseModel::Diagonal::shared_ptr pose_between_noise;
    gtsam::noiseModel::Diagonal::shared_ptr loop_constraint_noise;
    gtsam::noiseModel::Diagonal::shared_ptr gps_noise_model;

    void add_gps_factor(int frame_id, const GPS_MSG &position, bool provided);
};
} // namespace lidar_mapping
} // namespace beyond

#endif // MODULES_LIDAR_GPS_H_
