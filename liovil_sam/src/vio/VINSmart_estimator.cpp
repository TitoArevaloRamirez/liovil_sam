/*v1*************************************************************************************/
/* This is developed at Carnegie Mellon University in collaboration with Autel Robotics */
/*                                                                                      */
/* PI:                                                                                  */
/* George Kantor                                                                        */
/*                                                                                      */
/* Authors:                                                                             */ 
/* Weizhao Shao                                                                         */
/* Cong Li                                                                              */
/* Srinivasan Vijayarangan                                                              */
/*                                                                                      */
/* Please refer to the contract document for details on license/copyright information.  */
/****************************************************************************************/
/*
    iSAM2 relinearization takes too long
    Sliding Window with smart factors is working. No keyframes but only recent 'lag' frames.
    input: FRONTEND: frame ID should start from 1.
*/
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
// estimator class
class VINSmart_estimator {
protected:
    // ros
    ros::Subscriber imu_sub;
    ros::Subscriber f_sub;
    ros::Subscriber lidar_sub;
    ros::Subscriber lc_sub;
    // publisher
    ros::Publisher odom_imu_pub;
    ros::Publisher odom_camera_pub;
    ros::Publisher camera_pose_visual_pub;
    ros::Publisher camera_path_pub;
    CameraPoseVisualization cameraposevisual;
    nav_msgs::Path path;
    ros::Publisher landmark_pub;
    // state
    state_info last_optimized_state;
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;
    state_info imu_propogation_state;
    // keeep data
    std::map<uint64_t, keyframe> Win_keyFrame; // Kid to keyframe
    std::map<uint64_t, landmark> landMarks; // Lid to landmark
    std::map<uint64_t, candi_feat_info> candi_feature_pool; // feature id to candi_feat_info
    // smart factor
    gtsam::Pose3 body_P_sensor; // left camera to IMU
    gtsam::Cal3_S2Stereo::shared_ptr Kstereo;
    gtsam::noiseModel::Isotropic::shared_ptr cam_noise_model;
    gtsam::SmartStereoProjectionParams paramsSF;
    // imu factor
    double gravity;
    // state Format is (E,N,U,qX,qY,qZ,qW,velE,velN,velU, b?)
    gtsam::PreintegratedImuMeasurements *imu_preintegrated_; // PreintegratedImuMeasurements (for ImuFactor) or
                                                             // PreintegratedCombinedMeasurements (for CombinedImuFactor).
    gtsam::PreintegratedImuMeasurements *imu_propagation_;
    // fixed lag optimzer
    vioFixedLagSmoother::KeyTimestampMap keyStateMap;
    BatchFixedLagSmoother smootherBatch;
    double lag;
    gtsam::LevenbergMarquardtParams paramsLM;
    bool bound_smoother;
    // isam2
    bool useISAM2;
    gtsam::ISAM2Params paramISAM2;
    gtsam::ISAM2 isam2;
    // optimize (graph)
    //int correction_count; // graph time stamp, should match Kid at this time.
    gtsam::NonlinearFactorGraph *graph;
    gtsam::Values initial_values;
    // interface with front end
    bool initd;
    bool newKeyframe;
    std::queue<keyframe> keyframeBuffer;
    TimeBasedRetriever< Eigen::Matrix<double,11,1> > imuMsg;
    ros::Time current_frame_time;
    ros::Time last_frame_time;
    ros::Time last_imu_time;
    // robustness
    double featRatio;
    uint64_t N_thru;
    gtsam::TriangulationParameters triangulationParam;
    // initialization
    bool firstIMU;
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma2, gyro_noise_sigma2;
    gtsam::Matrix33 accel_bias_cov, gyro_bias_cov, accel_noise_cov, gyro_noise_cov;
    // initialize the preintegration parameters
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p;
    // lidar feedback: extrinsic lidar IMU
    Eigen::Matrix4d T_L_I;
    gtsam::noiseModel::Diagonal::shared_ptr lidar_cb_poseNoise;
    bool lcfb;
    double diff_;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform aftMappedTrans;

public:
    /* Constructor */
    VINSmart_estimator(ros::NodeHandle& nh) : cameraposevisual(1,0,0,1), gravity(9.81), initd(false),
                newKeyframe(false), imuMsg(false),  last_frame_time(0.0), last_imu_time(0.0), firstIMU(true) {
        // subscribers and publishers 
        std::string imuTopic, frontendTopic, lidarTopic, lcTopic, odomIMU, odomCamera, cameraPoseVisual, cameraPath, landmarkPub; 
        nh.getParam("vio/imu_topic", imuTopic);
        nh.getParam("vio/frontend_topic", frontendTopic);
        nh.getParam("vio/lidar_topic", lidarTopic);
        nh.getParam("vio/loop_closure_topic", lcTopic);
        nh.getParam("vio/odom_imu_rate_topic", odomIMU);
        nh.getParam("vio/odom_camera_rate_topic", odomCamera);
        nh.getParam("vio/camera_pose_visual_topic", cameraPoseVisual);
        nh.getParam("vio/camera_path_publish_topic", cameraPath);
        nh.getParam("vio/landmark_topic", landmarkPub);
        // subscribers
        std::cout << imuTopic << " " << frontendTopic << " " << lidarTopic << std::endl;
        imu_sub = nh.subscribe(imuTopic, 100, &VINSmart_estimator::imu_callback, this);
        f_sub = nh.subscribe(frontendTopic, 100, &VINSmart_estimator::frame_callback, this);
        lidar_sub = nh.subscribe(lidarTopic, 100, &VINSmart_estimator::lidar_callback, this);
        lc_sub = nh.subscribe(lcTopic,100, &VINSmart_estimator::lc_callback, this);
        // publishers
        odom_imu_pub = nh.advertise<nav_msgs::Odometry>(odomIMU, 100);
        odom_camera_pub = nh.advertise<nav_msgs::Odometry>(odomCamera, 100);
        camera_pose_visual_pub = nh.advertise<visualization_msgs::MarkerArray>(cameraPoseVisual, 100);
        camera_path_pub = nh.advertise<nav_msgs::Path>(cameraPath, 1000);
        landmark_pub = nh.advertise<liovil_sam::landmarks>(landmarkPub, 100);
        // camera and lidar
        double stereo_fx_, stereo_fy_, stereo_cx_, stereo_cy_, stereo_baseline_; 
        nh.getParam("vio/fx", stereo_fx_);
        nh.getParam("vio/fy", stereo_fy_);
        nh.getParam("vio/cx", stereo_cx_);
        nh.getParam("vio/cy", stereo_cy_);
        nh.getParam("vio/baseline", stereo_baseline_);
        // set camera intrinsic
        Kstereo = gtsam::Cal3_S2Stereo::shared_ptr(new gtsam::Cal3_S2Stereo(stereo_fx_, stereo_fy_, 0, stereo_cx_, stereo_cy_, stereo_baseline_)); 
        // set extrinsics between IMU and left camera
        std::vector<double> quaternion_I_LC;
        std::vector<double> position_I_LC;
        nh.getParam("vio/quat_I_LC", quaternion_I_LC);
        nh.getParam("vio/posi_I_LC", position_I_LC);

        gtsam::Rot3 cam_rot = gtsam::Rot3::Quaternion(quaternion_I_LC[0], 
                                                      quaternion_I_LC[1],
                                                      quaternion_I_LC[2],
                                                      quaternion_I_LC[3]);
        gtsam::Point3 cam_trans = gtsam::Point3(position_I_LC[0], 
                                                position_I_LC[1], 
                                                position_I_LC[2]);  

        body_P_sensor = gtsam::Pose3(cam_rot, cam_trans);
        // set extrinsics between IMU and Lidar
        std::vector<double> transformation_I_Lidar;
        nh.getParam("vio/T_I_Lidar", transformation_I_Lidar);
        for (int i=0;i<4;i++)
            for (int j=0;j<4;j++)
                T_L_I(i,j) = transformation_I_Lidar[i*4+j];
        // set if use lidar callback and time interval restricted
        nh.getParam("vio/lidar_feedback", lcfb);
        nh.getParam("vio/time_interval_to_use_lidar_feedback", diff_);
        // set camera noise
        double camera_noise_sigma;
        nh.getParam("vio/camera_sigma", camera_noise_sigma);
        cam_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, camera_noise_sigma);
        // set state noise
        double imu_gyro_bias_noise_sigma, imu_acce_bias_noise_sigma;
        nh.getParam("vio/imu_gyro_bias_noise_sigma", imu_gyro_bias_noise_sigma);
        nh.getParam("vio/imu_acce_bias_noise_sigma", imu_acce_bias_noise_sigma);
        bias_noise_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 
                        imu_acce_bias_noise_sigma, imu_acce_bias_noise_sigma, imu_acce_bias_noise_sigma, 
                        imu_gyro_bias_noise_sigma, imu_gyro_bias_noise_sigma, imu_gyro_bias_noise_sigma).finished());
        // set lidar feedback noise
        double lcfb_noise_transSigma, lcfb_noise_orienSigma;
        nh.getParam("vio/lidar_feedback_sigma_trans", lcfb_noise_transSigma);
        nh.getParam("vio/lidar_feedback_sigma_orien", lcfb_noise_orienSigma);
        lidar_cb_poseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 
                        lcfb_noise_orienSigma, lcfb_noise_orienSigma, lcfb_noise_orienSigma,
                        lcfb_noise_transSigma, lcfb_noise_transSigma, lcfb_noise_transSigma).finished()); // rad rad rad m m m
        // set optimizer paramters
        nh.getParam("vio/lag_to_use", lag);
        nh.getParam("vio/bound_optimizer", bound_smoother);
        nh.getParam("vio/use_isam2", useISAM2);
        // set parameter for Smart Factor
        double distThresh, outlierRejThresh;
        nh.getParam("vio/landmark_distance_threshold", distThresh);
        nh.getParam("vio/outlier_rejection_threshold", outlierRejThresh);        
        paramsSF.setLinearizationMode(gtsam::LinearizationMode::JACOBIAN_SVD); // HESSIAN not implemented?
        paramsSF.setLandmarkDistanceThreshold(distThresh);
        paramsSF.setDynamicOutlierRejectionThreshold(outlierRejThresh);
        //paramsSF.setEnableEPI(true);
        //paramsSF.setRankTolerance(0);
        paramsSF.print();
        // set parameter for ISAM2 optimizer
        paramISAM2.optimizationParams = gtsam::ISAM2GaussNewtonParams();// gtsam::ISAM2DoglegParams();
        //paramISAM2.relinearizeThreshold = 0.1;
        //paramISAM2.relinearizeSkip = 10;
        isam2 = gtsam::ISAM2(paramISAM2);
        // set parameter for LM optimzer
        bool ceresDef;
        nh.getParam("vio/use_ceres_default", ceresDef);
        if (ceresDef)
            LevenbergMarquardtParams::SetCeresDefaults(&paramsLM);
        else {
            LevenbergMarquardtParams::SetLegacyDefaults(&paramsLM);
            double maxIter;
            nh.getParam("vio/max_iteration", maxIter);
            paramsLM.maxIterations = maxIter;
            nh.getParam("vio/absolute_error_toleration", paramsLM.absoluteErrorTol);
            nh.getParam("vio/relative_error_toleration", paramsLM.absoluteErrorTol);
            nh.getParam("vio/lambda_upper_bound", paramsLM.lambdaUpperBound);
        }
        //paramsLM.errorTol = 0;
        paramsLM.setLinearSolverType("MULTIFRONTAL_CHOLESKY");
        paramsLM.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;
        paramsLM.print();
        // create graph and smoother
        graph = new gtsam::NonlinearFactorGraph();
        smootherBatch = BatchFixedLagSmoother(lag, bound_smoother, paramsLM);
        // set frontend screening parameter: for robustness
        nh.getParam("vio/robust_feature_ratio", featRatio);
        double nThru_;
        nh.getParam("vio/number_frame_tracked", nThru_);
        N_thru = nThru_;
        nh.getParam("vio/triangulation_landmark_distance_threshold", triangulationParam.landmarkDistanceThreshold);
        nh.getParam("vio/triangulation_outlier_rejection_threshold", triangulationParam.dynamicOutlierRejectionThreshold);
        // initialize last_optimized_state to identity::origin
        last_optimized_state.pose = gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(0.0,0.0,0.0));
        // Assemble noise model
        double statePoseNoiseSigmaTrans, statePoseNoiseSigmaOrien, stateVelNoiseSigma; 
        nh.getParam("vio/state_pose_trans_noise_sigma", statePoseNoiseSigmaTrans);
        nh.getParam("vio/state_pose_orien_noise_sigma", statePoseNoiseSigmaOrien);
        nh.getParam("vio/state_velocity_noise_sigma", stateVelNoiseSigma);
        last_optimized_state.pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << statePoseNoiseSigmaOrien, statePoseNoiseSigmaOrien, statePoseNoiseSigmaOrien,
                        statePoseNoiseSigmaTrans, statePoseNoiseSigmaTrans, statePoseNoiseSigmaTrans).finished()); // rad,rad,rad,m, m, m
        last_optimized_state.velocity_noise = gtsam::noiseModel::Isotropic::Sigma(3,stateVelNoiseSigma); // m/s
        last_optimized_state.bias_noise = bias_noise_model;
        // Use the sensor specs to build the noise model for the IMU factor.
        std::vector<double> accelBiasCov, gyroBiasCov, accelNoiseCov, gyroNoiseCov;
        nh.getParam("vio/accelerameter_noise_cov", accelNoiseCov);
        nh.getParam("vio/gyroscope_noise_cov", gyroNoiseCov);
        gyro_noise_cov << gyroNoiseCov[0], gyroNoiseCov[1], gyroNoiseCov[2],
                          gyroNoiseCov[3], gyroNoiseCov[4], gyroNoiseCov[5],
                          gyroNoiseCov[6], gyroNoiseCov[7], gyroNoiseCov[8];
        accel_noise_cov << accelNoiseCov[0], accelNoiseCov[1], accelNoiseCov[2],
                          accelNoiseCov[3], accelNoiseCov[4], accelNoiseCov[5],
                          accelNoiseCov[6], accelNoiseCov[7], accelNoiseCov[8];
        nh.getParam("vio/accelerameter_bias_cov", accelBiasCov);
        nh.getParam("vio/gyroscope_bias_cov", gyroBiasCov);
        accel_bias_cov << accelBiasCov[0], accelBiasCov[1], accelBiasCov[2],
                          accelBiasCov[3], accelBiasCov[4], accelBiasCov[5],
                          accelBiasCov[6], accelBiasCov[7], accelBiasCov[8];
        gyro_bias_cov << gyroBiasCov[0], gyroBiasCov[1], gyroBiasCov[2],
                          gyroBiasCov[3], gyroBiasCov[4], gyroBiasCov[5],
                          gyroBiasCov[6], gyroBiasCov[7], gyroBiasCov[8];
        // set pre-intergraion parameter
        double intgrationCov, biaAccOmega;
        nh.getParam("vio/integration_cov_sigma2", intgrationCov);
        nh.getParam("vio/bias_accelermeter_intgration_sigma2", biaAccOmega);
        std::cout << "************Here***********" << std::endl;
        p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(gravity);
        // acc white noise in continuous
        p->accelerometerCovariance = accel_noise_cov;
        // integration uncertainty continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3,3) * intgrationCov;
        // gyro white noise in continuous
        p->gyroscopeCovariance = gyro_noise_cov;
        // acc bias in continuous
        p->biasAccCovariance = accel_bias_cov;
        // gyro bias in continuous
        p->biasOmegaCovariance = gyro_bias_cov;
        // error in the bias used for preintegration
        p->biasAccOmegaInt = gtsam::Matrix::Identity(6,6) * biaAccOmega;
        // initialize the pre integration object
        #ifdef USE_COMBINED
            imu_preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(p, gtsam::imuBias::ConstantBias(Eigen::Matrix<double,3,1>::Zero(), Eigen::Matrix<double,3,1>::Zero()));
        #else
            imu_preintegrated_ = new gtsam::PreintegratedImuMeasurements(p, gtsam::imuBias::ConstantBias(Eigen::Matrix<double,3,1>::Zero(), Eigen::Matrix<double,3,1>::Zero()));
        #endif
        imu_propagation_ = new gtsam::PreintegratedImuMeasurements(p, gtsam::imuBias::ConstantBias(Eigen::Matrix<double,3,1>::Zero(), Eigen::Matrix<double,3,1>::Zero()));
    	
    }
    // system initialization
    void sysInit(const keyframe& first_keyframe) {
        // set frame time
        current_frame_time = first_keyframe.timestamp;
        // set frame time
        last_frame_time = current_frame_time;
        // check if there is imu measurement
        bool imuSuccess = false;
        Eigen::Matrix<double,11,1> initIMUmeasurement = imuMsg.GetClosestEntry(current_frame_time, imuSuccess);
        // cannot find a closest imu measurement
        if (imuSuccess == false)
            return;
        // a closest imu measurement is found
        Eigen::Quaterniond initialQuat(initIMUmeasurement(7),initIMUmeasurement(8),initIMUmeasurement(9),initIMUmeasurement(10));
        // set as initial orientation
        gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(initialQuat.w(),initialQuat.x(),initialQuat.y(),initialQuat.z());
        // set as global origin
        gtsam::Point3 prior_point(0.0,0.0,0.0);
        // set initial pose
        gtsam::Pose3 prior_pose(prior_rotation,prior_point);
        last_optimized_state.pose = prior_pose;
        last_optimized_state.Kid = 0;
        // set initial velocity
        last_optimized_state.velocity = gtsam::Vector3(0.0,0.0,0.0);
        // set initial bias to zero
        last_optimized_state.imu_bias = gtsam::imuBias::ConstantBias(Eigen::Matrix<double,3,1>::Zero(), Eigen::Matrix<double,3,1>::Zero());
        // set time
        last_optimized_state.msgtime = current_frame_time;
        // output result to file publish camera rate pose
        publish_camera_rate_pose(last_optimized_state);
        // publish imu rate pose
        imu_propogation_state.msgtime = current_frame_time;
        imu_propogation_state.pose = last_optimized_state.pose;
        imu_propogation_state.velocity = last_optimized_state.velocity;
        imu_propogation_state.imu_bias = last_optimized_state.imu_bias;
        publish_imu_rate_pose(last_optimized_state);
        // set as initial state
        initial_values.insert(X(first_keyframe.Kid), last_optimized_state.pose);
        initial_values.insert(V(first_keyframe.Kid), last_optimized_state.velocity);
        initial_values.insert(B(first_keyframe.Kid), last_optimized_state.imu_bias);
        // initialize graph
        // Add all prior factors (pose, velocity, bias) to the graph.
        graph->add(gtsam::PriorFactor<Pose3>(X(first_keyframe.Kid), last_optimized_state.pose, last_optimized_state.pose_noise));
        graph->add(gtsam::PriorFactor<Vector3>(V(first_keyframe.Kid), last_optimized_state.velocity, last_optimized_state.velocity_noise));
        graph->add(gtsam::PriorFactor<imuBias::ConstantBias>(B(first_keyframe.Kid), last_optimized_state.imu_bias, last_optimized_state.bias_noise));
        // Constrain the first pose such that it cannot change from its original value during optimization
        // NOTE: NonlinearEquality forces the optimizer to use QR rather than Cholesky
        // QR is much slower than Cholesky, but numerically more stable
        //graph->push_back(gtsam::NonlinearEquality<Pose3>(X(first_keyframe.Kid),last_optimized_state.pose));
        // add in visual factors
        addSmartStereoFactor(first_keyframe);
        // add in optimizer
        if (useISAM2) {
            // not implemented.
        }
        else {
            // for batch smoother
            keyStateMap[X(first_keyframe.Kid)] = (double)first_keyframe.Kid;
            keyStateMap[V(first_keyframe.Kid)] = (double)first_keyframe.Kid;
            keyStateMap[B(first_keyframe.Kid)] = (double)first_keyframe.Kid;   
            smootherBatch.update(*graph, initial_values, keyStateMap);
            graph->resize(0);
            initial_values.clear();
            imu_preintegrated_->resetIntegrationAndSetBias(last_optimized_state.imu_bias);
        }
        imuMsg.DeletePrevious(current_frame_time);
        // set initd
        std::cout << "system initialization done. " << std::endl;
        initd = true;
        return;
    }
    // imu callback function
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
        if (firstIMU) {
            last_frame_time = msg->header.stamp;
            firstIMU = false;
        }
        // initialize a, w, ahrs
        gtsam::Vector3 linear_acc, angular_vel;
        Eigen::Quaterniond ahrs;
        // get linear acceleration
        linear_acc(0) = msg->linear_acceleration.x;
        linear_acc(1) = msg->linear_acceleration.y;
        linear_acc(2) = msg->linear_acceleration.z;
        // get angular velocity
        angular_vel(0) = msg->angular_velocity.x;
        angular_vel(1) = msg->angular_velocity.y;
        angular_vel(2) = msg->angular_velocity.z;
        // get quaternion
        ahrs.x() = msg->orientation.x;
        ahrs.y() = msg->orientation.y;
        ahrs.z() = msg->orientation.z;
        ahrs.w() = msg->orientation.w;
        // add to buffer
        Eigen::Matrix<double,11,1> imuMeasurement;
        // initially last_imu_time = 0
        double delta_t = ((double)msg->header.stamp.toNSec() - (double)last_imu_time.toNSec())/1.0e9;
        //std::cout << std::setprecision(18) << "Last IMU time: " << last_imu_time.toNSec() << " this IMU time: " << msg->header.stamp.toNSec() << "IMU message delta_t: " << delta_t << std::endl;
        last_imu_time = msg->header.stamp;
        // buffer sequence as this
        imuMeasurement << delta_t, linear_acc , angular_vel, ahrs.w(), ahrs.x(), ahrs.y(), ahrs.z();
        // add add
        imuMsg.AddEntry(msg->header.stamp,imuMeasurement);
        // imu propogation
        if (initd) {
            /*
            // get the delta_t for the imu_propogation
            double ddt_p = (msg->header.stamp.toNSec() - imu_propogation_state.msgtime.toNSec())/1.0e9;
            // get transformation
            Eigen::Matrix3d R_wb = ahrs.normalized().toRotationMatrix();
            // update acce with bias
            gtsam::Vector3 tempAcce = linear_acc - imu_propogation_state.imu_bias.accelerometer();
            // transformation to ENU coordinate
            tempAcce = R_wb*tempAcce;
            tempAcce(2) = tempAcce(2)-gravity;
            // get updated position
            gtsam::Vector3 position = imu_propogation_state.pose.translation() +
                                      imu_propogation_state.velocity * ddt_p +
                                      0.5 * tempAcce * ddt_p * ddt_p;

            // get updated velocity
            imu_propogation_state.velocity = imu_propogation_state.velocity + tempAcce * ddt_p;
            // update gyro with bias
            gtsam::Vector3 tempVel = angular_vel - imu_propogation_state.imu_bias.gyroscope();
            // transformation to ENU coordinate
            tempVel = R_wb * tempVel;
            // integrate the orientation
            Eigen::Matrix3d w_hat;
            w_hat << 0, -tempVel(2), tempVel(1),
                     tempVel(2), 0, -tempVel(0),
                    -tempVel(1), tempVel(0), 0;
            w_hat = Eigen::Matrix3d::Identity() + w_hat * ddt_p + 0.5 * w_hat * ddt_p * w_hat * ddt_p;
            w_hat = w_hat.colwise().normalized();
            // get updated orientation
            gtsam::Rot3 tQ(imu_propogation_state.pose.rotation().matrix() * w_hat);
            // set the result
            imu_propogation_state.msgtime = msg->header.stamp;
            imu_propogation_state.pose = gtsam::Pose3(tQ, position);
            */
            // use gtsam object to do integration
            imu_propagation_->integrateMeasurement(Vector3(imuMeasurement[1],imuMeasurement[2],imuMeasurement[3]),
                                        Vector3(imuMeasurement[4],imuMeasurement[5],imuMeasurement[6]),
                                        delta_t );
            // predict current imu pose
            gtsam::NavState pred_cur_pose = imu_propagation_->
            predict(gtsam::NavState(last_optimized_state.pose, last_optimized_state.velocity),
                             last_optimized_state.imu_bias);
            // set in the object
            imu_propogation_state.msgtime = msg->header.stamp;
            imu_propogation_state.pose = pred_cur_pose.pose();
            imu_propogation_state.velocity = pred_cur_pose.v();
            // publish the result
            publish_imu_rate_pose(imu_propogation_state);
        }
        return;
    }
    // frame callback function
    void frame_callback(const liovil_sam::StereoFeatureMatches::ConstPtr& msg) {
        // get time
        //clock_t start(clock());
        //std::cout << "Frame callback: [" << (int)msg->header.seq << "]\n";
        // map Fid to "if it is robust"
        std::map<uint64_t, bool> whether_robust_feat;
        // set triangulation camera
        gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2> > monoCameras;
        //std::vector<gtsam::Point2> monoMeasured;
        gtsam::PinholeBase::MeasurementVector monoMeasured;
        //std::cout << "HERE CTM 2" << std::endl;
        // system not initialized
        if (!initd) {
            uint32_t num_feat_thru_N_frame = 0;
            uint32_t totalCnt = msg->num_matches;
            // check robustness score on features
            gtsam::NavState pred_cur_pose = gtsam::NavState(last_optimized_state.pose, last_optimized_state.velocity);
            // only on stereo
            const gtsam::Pose3 leftPose = pred_cur_pose.pose().compose(body_P_sensor);
            const gtsam::Cal3_S2 monoCal = Kstereo->calibration();
            const gtsam::PinholeCamera<gtsam::Cal3_S2> leftCamera_i(leftPose, monoCal);
            const gtsam::Pose3 lcam_P_rcam = gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(Kstereo->baseline(),0.0,0.0));
            const gtsam::Pose3 rightPose = leftPose.compose( lcam_P_rcam );
            const gtsam::PinholeCamera<gtsam::Cal3_S2> rightCamera_i(rightPose, monoCal);
            monoCameras.push_back(leftCamera_i);
            monoCameras.push_back(rightCamera_i);
            // for each feature
            for (uint32_t i=0; i<totalCnt; i++) {
                // check robustness score on features, tracked frame and triangulation
                whether_robust_feat[msg->feature_ids[i]] = false;
                // initializing or increasing count maintainance
                if (candi_feature_pool.empty() || candi_feature_pool.find(msg->feature_ids[i])==candi_feature_pool.end())
                    candi_feature_pool[msg->feature_ids[i]] = candi_feat_info(1,msg->frame_id-1);
                else {
                    candi_feature_pool[msg->feature_ids[i]].numFrame++;
                    candi_feature_pool[msg->feature_ids[i]].recentFrameID = msg->frame_id-1;
                }
                // check if this feature is robust enough:: Robust 1
                if (candi_feature_pool[msg->feature_ids[i]].numFrame >= N_thru) {
                    // stereo triangulation
                    monoMeasured.push_back(gtsam::Point2(msg->left_xs[i],msg->left_ys[i]));
                    monoMeasured.push_back(gtsam::Point2(msg->right_xs[i],msg->right_ys[i]));
                    // check stereo triangulation
                    gtsam::TriangulationResult result =
                    gtsam::triangulateSafe(monoCameras, monoMeasured, triangulationParam);
                    std::cout << result;
                    if (result) {
                        num_feat_thru_N_frame++;
                        whether_robust_feat[msg->feature_ids[i]] = true;
                    }
                    monoMeasured.clear();
                }
            }
            std::cout << "INIT: Total feature of " << totalCnt << " with robust of " << num_feat_thru_N_frame << std::endl;
            //std::cout << "HERE CTM 1" << std::endl; 
            // initialize if there are enough robust features
            if ( double(num_feat_thru_N_frame) > featRatio * double(totalCnt) ) {  
                //std::cout << "HERE CTM " << std::endl;          	
                keyframe KFtoADD;
                KFtoADD.timestamp = msg->header.stamp;
                // add the robust ones to KeyFrmae
                KFtoADD.landmarkSize = num_feat_thru_N_frame;
                for (uint32_t i=0; i<totalCnt; i++) {
                    if ( whether_robust_feat[msg->feature_ids[i]] ) {
                        // feature location
                        KFtoADD.landLeft.push_back(Eigen::Vector2d(msg->left_xs[i],msg->left_ys[i]));
                        KFtoADD.landRight.push_back(Eigen::Vector2d(msg->right_xs[i],msg->right_ys[i]));
                        KFtoADD.LidSet.push_back(msg->feature_ids[i]);
                        KFtoADD.Lvalid.push_back(true);
                        // remove from candidate_feature_pool
                        candi_feature_pool.erase(msg->feature_ids[i]);
                    }
                }
                // display keyframe info
                std::cout << "Keyframe initializing with " << num_feat_thru_N_frame << " features:\n" << KFtoADD;
                // initilize the batch smoother
                sysInit(KFtoADD);
            }
        }
        // system initialized, there is a keyframe already
        else {
            //std::cout << "HERE CTM " << std::endl;  
            // robust feature cover check
            uint32_t num_feat_thru_N_frame = 0;
            uint32_t num_feat_as_landmark = 0;
            uint32_t totalCnt = msg->num_matches;
            // get IMU measurement between last keyframe and current frame
            bool imuSuccess=false;
            ros::Time ttteeee = msg->header.stamp;
            std::vector< std::pair< ros::Time,Eigen::Matrix<double,11,1> > > imuMeasurements =
                        imuMsg.GetInBetween(last_frame_time, ttteeee, imuSuccess);
            // integrate imu measurement
            for (std::vector< std::pair< ros::Time,Eigen::Matrix<double,11,1> > >::iterator
                        iter=imuMeasurements.begin(); iter!=imuMeasurements.end(); iter++) {
                Eigen::Matrix<double,11,1> imum = iter->second;
                imu_preintegrated_->integrateMeasurement(Vector3(imum[1],imum[2],imum[3]),
                                                       Vector3(imum[4],imum[5],imum[6]),
                                                       imum[0]);
            }
            // predict current frame pose by IMU
            gtsam::NavState pred_cur_pose = imu_preintegrated_->
            predict(gtsam::NavState(last_optimized_state.pose, last_optimized_state.velocity),
                             last_optimized_state.imu_bias);
            // clear object
            imu_preintegrated_->resetIntegrationAndSetBias(last_optimized_state.imu_bias);
            // stereo triangulation test
            const gtsam::Pose3 leftPose = pred_cur_pose.pose().compose(body_P_sensor);
            const gtsam::Cal3_S2 monoCal = Kstereo->calibration();
            const gtsam::PinholeCamera<gtsam::Cal3_S2> leftCamera_i(leftPose,monoCal);
            const gtsam::Pose3 lcam_P_rcam = gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(Kstereo->baseline(),0.0,0.0));
            const gtsam::Pose3 rightPose = leftPose.compose( lcam_P_rcam );
            const gtsam::PinholeCamera<gtsam::Cal3_S2> rightCamera_i(rightPose,monoCal);
            monoCameras.push_back(leftCamera_i);
            monoCameras.push_back(rightCamera_i);
            // check robustness score on features, tracked frame and triangulation
            for (uint32_t i=0; i<totalCnt; i++) {
                // initialize this robust score
                whether_robust_feat[msg->feature_ids[i]] = false;
                // this feature is already in candidate pool
                if ( candi_feature_pool.find(msg->feature_ids[i])!=candi_feature_pool.end() ) {
                    candi_feature_pool[msg->feature_ids[i]].numFrame++;
                    candi_feature_pool[msg->feature_ids[i]].recentFrameID = msg->frame_id-1;
                    // check if this feature is robust enough
                    if (candi_feature_pool[msg->feature_ids[i]].numFrame >= N_thru) {
                        // stereo triangulation
                        monoMeasured.push_back(gtsam::Point2(msg->left_xs[i],msg->left_ys[i]));
                        monoMeasured.push_back(gtsam::Point2(msg->right_xs[i],msg->right_ys[i]));
                        // check stereo triangulation
                        gtsam::TriangulationResult result =
                        gtsam::triangulateSafe(monoCameras, monoMeasured, triangulationParam);
                        // good result gets in
                        if (result) {
                            num_feat_thru_N_frame++;
                            whether_robust_feat[msg->feature_ids[i]] = true;
                        }
                        monoMeasured.clear();
                    }
                }
                // this feature is already in local landmark
                else if ( landMarks.find(msg->feature_ids[i])!=landMarks.end() )
                    num_feat_as_landmark++;
                // initialize this feature in candidate pool
                else
                    candi_feature_pool[msg->feature_ids[i]] = candi_feat_info(1,msg->frame_id-1);
            }
            // useful info
            std::cout << std::setprecision(8) << "relative translation: " << (pred_cur_pose.t()-last_optimized_state.pose.translation()).norm() << "\n";
            std::cout << "Feature number in total: " << totalCnt << " with robust new: " << num_feat_thru_N_frame
                      << " and local landmark " << num_feat_as_landmark << std::endl;
            // no IMU info.
            if ((pred_cur_pose.t()-last_optimized_state.pose.translation()).norm() == 0)
                return;
            // crate the keyframe
            keyframe KFtoADD;
            KFtoADD.timestamp = msg->header.stamp;
            // add the robust ones to KeyFrmae
            KFtoADD.landmarkSize = num_feat_thru_N_frame + num_feat_as_landmark;
            // add in landmarks
            for (uint32_t i=0; i<totalCnt; i++) {
                // already in local landmarks
                if ( landMarks.find(msg->feature_ids[i])!=landMarks.end() ) {
                    // faeture location
                    KFtoADD.landLeft.push_back(Eigen::Vector2d(msg->left_xs[i],msg->left_ys[i]));
                    KFtoADD.landRight.push_back(Eigen::Vector2d(msg->right_xs[i],msg->right_ys[i]));
                    KFtoADD.LidSet.push_back(msg->feature_ids[i]);
                    KFtoADD.Lvalid.push_back(true);
                }
                // or robust enough
                else if ( whether_robust_feat[msg->feature_ids[i]] ) {
                    // faeture location
                    KFtoADD.landLeft.push_back(Eigen::Vector2d(msg->left_xs[i],msg->left_ys[i]));
                    KFtoADD.landRight.push_back(Eigen::Vector2d(msg->right_xs[i],msg->right_ys[i]));
                    KFtoADD.LidSet.push_back(msg->feature_ids[i]);
                    KFtoADD.Lvalid.push_back(true);
                    // remove from candidate_feature_pool
                    candi_feature_pool.erase(msg->feature_ids[i]);
                }
            }
            // create keyframe object
            keyframeBuffer.push(KFtoADD);
            newKeyframe = true;
            std::cout << KFtoADD;
        }
        // calculated time
        //float diff( -((float)start-(float)clock())/CLOCKS_PER_SEC );
        //std::cout << "frame_callback time: " << std::setprecision(5) << diff*1.0e3 << "ms"<< std::endl;
        return;
    }
    // lidar callback function
    void lidar_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::cout << "lidar callback: [" << (int)msg->header.seq << "]\n";
        // initialize and set time
        gtsam::Pose3 lidar_cb_pose;
        ros::Time lidar_cb_time = msg->header.stamp;
        // get pose in Eigen, rotation matrix and translation
        Eigen::Quaterniond orient = Eigen::Quaternion<double>(msg->pose.pose.orientation.w,
                                                              msg->pose.pose.orientation.x,
                                                              msg->pose.pose.orientation.y,
                                                              msg->pose.pose.orientation.z);
        // get Eigen transformation matrix
        Eigen::Matrix4d T_O_lidar = Eigen::Matrix4d::Zero();
        T_O_lidar(3,3) = 1;
        T_O_lidar.block<3,3>(0,0) = orient.normalized().toRotationMatrix();
        T_O_lidar.block<3,1>(0,3) = Eigen::Matrix<double,3,1>(msg->pose.pose.position.x,
                                                              msg->pose.pose.position.y,
                                                              msg->pose.pose.position.z);
        // apply extrinsic transformation
        Eigen::Matrix4d T_O_I = T_O_lidar * T_L_I;
        // set the state_info
        gtsam::Rot3 rotation = gtsam::Rot3(T_O_I.block<3,3>(0,0));
        gtsam::Point3 point(T_O_I.block<3,1>(0,3));
        lidar_cb_pose = gtsam::Pose3(rotation, point);
        // write raw feedback (IMU) pose to file 
        outFile_result_lidar << point(0) << ", " << point(1) << ", " << point(2) << ", "
                        << rotation.ypr()(0) << ", " << rotation.ypr()(1) << ", " << rotation.ypr()(2) << "\n";
        outFile_result_lidar.close();
        outFile_result_lidar.open(res_lidar_Path, std::ios_base::app);
        // use time to find the frame for the constraint
        if (lcfb) {
            for (auto it = Win_keyFrame.begin(); it!=Win_keyFrame.end(); it++) {
                double diff = fabs( (double)lidar_cb_time.toNSec()/1e9-(double)it->second.timestamp.toNSec()/1e9 ); 
                std::cout << "Time diff lidar feedback: "<< diff << std::endl;
                // check
                if ( diff < diff_ ) {
                    std::cout << "\nLidar callback found and added at " << std::setprecision(16) << "Lidar time: " << lidar_cb_time.toNSec()/1e9 << " CB_frame time: " 
                        << it->second.timestamp.toNSec()/1e9 <<" with diff " << diff << "\n";
                    uint64_t lidarCB_frame_id = it->first;
                    // add prior
                    graph->add(gtsam::PriorFactor<Pose3>(X(lidarCB_frame_id), lidar_cb_pose, lidar_cb_poseNoise));
                    // print
                    std::cout << "Lidar Callback Pose:\n";
                    lidar_cb_pose.print();
                    std::cout << "Frame Pose:\n";
                    smootherBatch.calculateEstimate().at<gtsam::Pose3>(X(lidarCB_frame_id)).print();
                }
            }
        }
        return;
    }
    // loop closure frame pose callback function
    void lc_callback(const nav_msgs::Odometry::ConstPtr& msg) {return;}
    // this function publishes IMU rate poses
    void publish_imu_rate_pose(const state_info& imu_rate_pose) {
        nav_msgs::Odometry odom;
        odom.header.stamp = imu_rate_pose.msgtime;
        odom.header.frame_id = "world";
        //set the position
        odom.pose.pose.position.x = imu_rate_pose.pose.translation()(0);
        odom.pose.pose.position.y = imu_rate_pose.pose.translation()(1);
        odom.pose.pose.position.z = imu_rate_pose.pose.translation()(2);
        //set the orientation
        odom.pose.pose.orientation.x = imu_rate_pose.pose.rotation().toQuaternion().x();
        odom.pose.pose.orientation.y = imu_rate_pose.pose.rotation().toQuaternion().y();
        odom.pose.pose.orientation.z = imu_rate_pose.pose.rotation().toQuaternion().z();
        odom.pose.pose.orientation.w = imu_rate_pose.pose.rotation().toQuaternion().w();
        //publish the message
        odom_imu_pub.publish(odom);
        // to file
        gtsam::Vector3 position_rt = imu_rate_pose.pose.translation();
        gtsam::Vector3 orient_rt = imu_rate_pose.pose.rotation().ypr();
        outFile_result_rt_imuRate << position_rt(0) << ", " << position_rt(1) << ", " << position_rt(2) << ", "
                        << orient_rt(0) << ", " << orient_rt(1) << ", " << orient_rt(2) << "\n";
        outFile_result_rt_imuRate.close();
        outFile_result_rt_imuRate.open(res_rt_imuRate_Path, std::ios_base::app);
    }
    // this function publishes camera rate poses
    void publish_camera_rate_pose(const state_info& camera_rate_pose) {

        aftMappedTrans.frame_id_ = "world";
        aftMappedTrans.child_frame_id_ = "/camera_odom";

        nav_msgs::Odometry odom;
        geometry_msgs::PoseStamped pose_stamped;
        odom.header.stamp = camera_rate_pose.msgtime;
        odom.header.frame_id = "world";
        odom.header.seq = camera_rate_pose.Kid;
        // set the position
        odom.pose.pose.position.x = camera_rate_pose.pose.translation()(0);
        odom.pose.pose.position.y = camera_rate_pose.pose.translation()(1);
        odom.pose.pose.position.z = camera_rate_pose.pose.translation()(2);
        // set the orientation
        odom.pose.pose.orientation.x = camera_rate_pose.pose.rotation().toQuaternion().x();
        odom.pose.pose.orientation.y = camera_rate_pose.pose.rotation().toQuaternion().y();
        odom.pose.pose.orientation.z = camera_rate_pose.pose.rotation().toQuaternion().z();
        odom.pose.pose.orientation.w = camera_rate_pose.pose.rotation().toQuaternion().w();
        // get path
        pose_stamped.header.stamp = camera_rate_pose.msgtime;
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose = odom.pose.pose;
        path.header.stamp = camera_rate_pose.msgtime;
        path.header.frame_id = "world";
        path.poses.push_back(pose_stamped);
        // publish the message
        odom_camera_pub.publish(odom);
        camera_path_pub.publish(path);

        //publish the tf - for visualization
        tf::Quaternion q_tf(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        aftMappedTrans.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
        aftMappedTrans.setRotation(q_tf);
        tfBroadcaster.sendTransform(aftMappedTrans);

        // publish the marker for visualization
        cameraposevisual.reset();  
        cameraposevisual.add_pose(camera_rate_pose.pose.translation(), camera_rate_pose.pose.rotation().toQuaternion());
        cameraposevisual.publish_by(camera_pose_visual_pub, odom.header);
        // output real time results to file
        gtsam::Vector3 position_rt = last_optimized_state.pose.translation();
        gtsam::Vector3 orient_rt = last_optimized_state.pose.rotation().ypr();
        outFile_result_rt << position_rt(0) << ", " << position_rt(1) << ", " << position_rt(2) << ", "
                        << orient_rt(0) << ", " << orient_rt(1) << ", " << orient_rt(2) << "\n";
        outFile_result_rt.close();
        outFile_result_rt.open(res_rt_Path, std::ios_base::app);
        outFile_result_bias << last_optimized_state.imu_bias.accelerometer()(0) << ", " << last_optimized_state.imu_bias.accelerometer()(1) << ", " << last_optimized_state.imu_bias.accelerometer()(2) << ", "
                        << last_optimized_state.imu_bias.gyroscope()(0) << ", " << last_optimized_state.imu_bias.gyroscope()(1) << ", " << last_optimized_state.imu_bias.gyroscope()(2) << "\n";
        outFile_result_bias.close();
        outFile_result_bias.open(res_bias, std::ios_base::app);
    }
    // loop to run from the beginning
    void optimizeLoop() {
        while(true) {
            if (newKeyframe && initd) {
                newKeyframe = false;
                while(!keyframeBuffer.empty()) {
                    // time
                    clock_t start(clock());
                    // obtain the new keyframe
                    keyframe current_key_frame(keyframeBuffer.front());
                    keyframeBuffer.pop();
                    // check time
                    current_frame_time = current_key_frame.timestamp;
                    //std::cout << "OPT Frame id: " << current_key_frame.Kid << "\n";
                    // add imu factor and predict a prior pose
                    addImuFactor(last_frame_time, current_frame_time, current_key_frame.Kid);
                    // add stereo factors
                    addSmartStereoFactor(current_key_frame);
                    // calculated time
                    //float diff_1( -((float)start-(float)clock())/CLOCKS_PER_SEC );
                    //std::cout << "add factors once time: " << std::setprecision(5) << diff_1*1.0e3 << "ms"<< std::endl;
                    // optimize
                    gttic_(optimizing);
                    optimizer(current_frame_time, current_key_frame.Kid);
                    gttoc_(optimizing);
                    //tictoc_print2_();
                    // prepare for next loop
                    imuMsg.DeletePrevious(current_frame_time);
                    // update smartfactor
                    //clock_t start_u(clock());
                    updateLandMark(current_key_frame.Kid);
                    // calculated time
                    //float diff_u( -((float)start_u-(float)clock())/CLOCKS_PER_SEC );
                    //std::cout << "update landmark once time: " << std::setprecision(5) << diff_u*1.0e3 << "ms"<< std::endl;
                    last_frame_time = current_frame_time;
                    // calculated time
                    float diff( -((float)start-(float)clock())/CLOCKS_PER_SEC );
                    std::cout << "optimizeLoop once time: " << std::setprecision(5) << diff*1.0e3 << "ms"<< std::endl;
                    outFile_time << diff << "\n";
                    outFile_time.close();
                    outFile_time.open(time_Path, std::ios_base::app);
                }
            }
            ros::spinOnce();
        }
        return;
    }
    // extract imu measurements and add as factor
    void addImuFactor(ros::Time last_frame_time, ros::Time current_frame_time, uint64_t curr_id) {
        // get IMU measurement between last and current frame
        bool imuSuccess=false;
        std::vector< std::pair< ros::Time,Eigen::Matrix<double,11,1> > > imuMeasurements =
                    imuMsg.GetInBetween(last_frame_time, current_frame_time, imuSuccess);
        // debug
        //std::cout << "Total: " << imuMeasurements.size() << " imu measurements " << "\n";
        // integrate imu measurement
        for (std::vector< std::pair< ros::Time,Eigen::Matrix<double,11,1> > >::iterator
                    iter=imuMeasurements.begin(); iter!=imuMeasurements.end(); iter++) {
            Eigen::Matrix<double,11,1> imum = iter->second;
            imu_preintegrated_->integrateMeasurement(gtsam::Vector3(imum[1],imum[2],imum[3]),
                                                   gtsam::Vector3(imum[4],imum[5],imum[6]),
                                                   imum[0]);
        }
        // add imu factor
        #ifdef USE_COMBINED
            gtsam::PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<gtsam::PreintegratedCombinedMeasurements*>(imu_preintegrated_);
            gtsam::CombinedImuFactor imu_factor(X(curr_id-1), V(curr_id-1),
                                         X(curr_id  ), V(curr_id  ),
                                         B(curr_id-1), B(curr_id  ),
                                         *preint_imu_combined);
            graph->add(imu_factor);
        #else
            gtsam::PreintegratedImuMeasurements *preint_imu = dynamic_cast<gtsam::PreintegratedImuMeasurements*>(imu_preintegrated_);
            gtsam::ImuFactor imu_factor(X(curr_id-1), V(curr_id-1),
                                 X(curr_id  ), V(curr_id  ),
                                 B(curr_id-1),
                                 *preint_imu);
            graph->add(imu_factor);
            gtsam::imuBias::ConstantBias zero_bias(gtsam::Vector3(0, 0, 0), gtsam::Vector3(0, 0, 0));
            graph->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(curr_id-1),
                                                            B(curr_id  ),
                                                            zero_bias, bias_noise_model));
        #endif
        // predict an initial estimate by IMU
        gtsam::NavState prop_state = imu_preintegrated_->predict(gtsam::NavState(last_optimized_state.pose, last_optimized_state.velocity), last_optimized_state.imu_bias);
        initial_values.insert(X(curr_id), prop_state.pose());
        initial_values.insert(V(curr_id), prop_state.v());
        initial_values.insert(B(curr_id), last_optimized_state.imu_bias);
        if (imuMeasurements.size()==0)
            initial_values.print();
        return;
    }
    // Add stereo structureless vision factor (smart stereo factor)
    void addSmartStereoFactor(const keyframe& curr_frame) {
        // add keyframe in sliding window
        Win_keyFrame[curr_frame.Kid] = curr_frame;
        // create camera
        gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2> > monoCameras;
        //std::vector<gtsam::Point2> monoMeasured;
        gtsam::PinholeBase::MeasurementVector monoMeasured;
        // get predicted state
        gtsam::NavState pred_cur_pose = gtsam::NavState(initial_values.at<gtsam::Pose3>(X(curr_frame.Kid)), initial_values.at<gtsam::Vector3>(V(curr_frame.Kid)));
        const gtsam::Pose3 leftPose = pred_cur_pose.pose().compose(body_P_sensor);
        const gtsam::Cal3_S2 monoCal = Kstereo->calibration();
        const gtsam::PinholeCamera<gtsam::Cal3_S2> leftCamera_i(leftPose,monoCal);
        const gtsam::Pose3 lcam_P_rcam = gtsam::Pose3(gtsam::Rot3(),gtsam::Point3(Kstereo->baseline(),0.0,0.0));
        const gtsam::Pose3 rightPose = leftPose.compose( lcam_P_rcam );
        const gtsam::PinholeCamera<gtsam::Cal3_S2> rightCamera_i(rightPose,monoCal);
        monoCameras.push_back(leftCamera_i);
        monoCameras.push_back(rightCamera_i);
        // for all features observed
        for ( uint32_t i=0;i<curr_frame.landmarkSize;i++ ) {
            // no landmark or this landmark has not appeared before
            if ( landMarks.empty() || landMarks.find(curr_frame.LidSet[i])==landMarks.end() ) {
                // create a landmark
                landmark temp;
                temp.Win_KidSet.push_back(curr_frame.Kid);
                temp.Win_KindexSet.push_back(i);
                temp.Ksize = 1;
                temp.Win_added = false;
                temp.Lid = curr_frame.LidSet[i];
                SmartStereoFactor::shared_ptr ttt(new SmartStereoFactor(cam_noise_model, paramsSF, body_P_sensor));
                temp.smartfactor = ttt;
                // stereo triangulation
                monoMeasured.push_back( gtsam::Point2(curr_frame.landLeft[i](0),curr_frame.landLeft[i](1)) );
                monoMeasured.push_back( gtsam::Point2(curr_frame.landRight[i](0),curr_frame.landRight[i](1)) );
                // check stereo triangulation
                gtsam::TriangulationResult result =
                gtsam::triangulateSafe(monoCameras, monoMeasured, triangulationParam);
                if (result)
                    temp.Position = *result;
                else
                    std::cout << "not triangluating at adding smart factor!!\n";
                monoMeasured.clear();
                // add measurement to factor
                temp.smartfactor->add(gtsam::StereoPoint2(curr_frame.landLeft[i](0),curr_frame.landRight[i](0),
                                            curr_frame.landLeft[i](1)), X(curr_frame.Kid), Kstereo);
                // added to hash map
                landMarks[curr_frame.LidSet[i]] = temp;
            }
            // has landmark and this landmark has appeared before
            else {
                // add to the factor
                landMarks[curr_frame.LidSet[i]].smartfactor->add(gtsam::StereoPoint2(curr_frame.landLeft[i](0),curr_frame.landRight[i](0),curr_frame.landLeft[i](1)), X(curr_frame.Kid), Kstereo);
                landMarks[curr_frame.LidSet[i]].Win_KidSet.push_back(curr_frame.Kid);
                landMarks[curr_frame.LidSet[i]].Win_KindexSet.push_back(i);
                landMarks[curr_frame.LidSet[i]].Ksize++;
                // check if add to the graph
                if (!landMarks[curr_frame.LidSet[i]].Win_added && landMarks[curr_frame.LidSet[i]].Ksize > 1) {
                    graph->add(landMarks[curr_frame.LidSet[i]].smartfactor);
                    landMarks[curr_frame.LidSet[i]].Win_added = true;
                }
            }
        }
        return;
    }
    // optimization
    void optimizer(const ros::Time& cur_f_time, uint64_t curr_id) {
        gtsam::Values post_values;
        // optimize
        if (useISAM2) {
            isam2.update(*graph, initial_values);
            isam2.update();
            isam2.update();
            isam2.update();
            // obtain value
            post_values = isam2.calculateEstimate(); //post_values.print("iSAM2 Current Estimate: ");
        }
        else {                
            // for batch smoother
            keyStateMap[X(curr_id)] = (double)curr_id;
            keyStateMap[V(curr_id)] = (double)curr_id;
            keyStateMap[B(curr_id)] = (double)curr_id;
            // should be good ...
            smootherBatch.update(*graph, initial_values, keyStateMap);
            // obtain value
            post_values = smootherBatch.calculateEstimate(); //post_values.print("Batch Current Estimate: ");
            // reset
            keyStateMap.clear();
        }
        // reset optimization
        graph->resize(0);
        initial_values.clear();
        // store the result
        last_optimized_state.Kid = curr_id;
        last_optimized_state.msgtime = cur_f_time;
        last_optimized_state.imu_bias = post_values.at<gtsam::imuBias::ConstantBias>(B(curr_id));
        last_optimized_state.pose = post_values.at<gtsam::Pose3>(X(curr_id));
        last_optimized_state.velocity = post_values.at<gtsam::Vector3>(V(curr_id));
        // reset imu_preintegration and imu_propagation
        imu_preintegrated_->resetIntegrationAndSetBias(last_optimized_state.imu_bias);
        imu_propagation_->resetIntegrationAndSetBias(last_optimized_state.imu_bias);
        // publish camera rate pose and to file
        publish_camera_rate_pose(last_optimized_state);
        return;
    }
    // account for marginalization
    void updateLandMark(uint64_t curr_id) {
        // find the keyframe for marginalization
        // the off by 1 is by looking at the batch smoother that when optimizing
        // the 21th frame, frame 0 gets marginalized (if lag 20)
        int toMargiKid = curr_id - (int)lag - 1;
        if ( !useISAM2 && toMargiKid>=0 ) {
            //std::cout << "to marginilize ID: " << toMargiKid << std::endl;
            // obtain the keyframe to marginilize out
            keyframe toMargiKeyframe = Win_keyFrame[toMargiKid];
            // publish all robust landmarks in the frame
            liovil_sam::landmarks landmarks_msg;
            landmarks_msg.header.stamp = toMargiKeyframe.timestamp; 
            landmarks_msg.frame_id = toMargiKeyframe.Kid;
            // check robustness and add in the message
            for (uint32_t i=0; i<toMargiKeyframe.landmarkSize; i++) {
                gtsam::TriangulationResult result = landMarks[toMargiKeyframe.LidSet[i]].smartfactor->point();
                //std::cout << "triangluate result: " << result << std::endl;
                if (result) {
                    landmarks_msg.feature_ids.push_back(toMargiKeyframe.LidSet[i]);
                    landmarks_msg.leftPixel_xs.push_back(toMargiKeyframe.landLeft[i](0));
                    landmarks_msg.leftPixel_ys.push_back(toMargiKeyframe.landLeft[i](1));
                    landmarks_msg.location_x.push_back(result->x());
                    landmarks_msg.location_y.push_back(result->y());
                    landmarks_msg.location_z.push_back(result->z());
                }
            }
            landmarks_msg.size = landmarks_msg.feature_ids.size();
            landmark_pub.publish(landmarks_msg);
            // update all landmarks observed in this keyframe
            for (uint32_t i=0; i<toMargiKeyframe.landmarkSize; i++) {
                // see if this landmark is still valid in this frame
                if (!toMargiKeyframe.Lvalid[i])
                    continue;
                // Lid corresponds to this feature
                uint32_t tempLid = toMargiKeyframe.LidSet[i];
                if ( landMarks.find(tempLid)!=landMarks.end() ) {
                    if ( landMarks[tempLid].Ksize>0 ) {
                        // only keep the most recent keyframes observing this landmark
                        uint32_t KframeIdTemp = landMarks[tempLid].Win_KidSet[landMarks[tempLid].Ksize-1];
                        // if there is a remaining keyframe
                        if ( KframeIdTemp != (uint32_t)toMargiKid ) {
                            // update the Win_keyframe
                            for (int j=0; j<landMarks[tempLid].Ksize-1; j++)
                                Win_keyFrame[landMarks[tempLid].Win_KidSet[j]].Lvalid[landMarks[tempLid].Win_KindexSet[j]] = false;
                            // keep the last seen frame in the landmark
                            int KframeIndexTemp = landMarks[tempLid].Win_KindexSet[landMarks[tempLid].Ksize-1];
                            keyframe KframeTemp = Win_keyFrame[KframeIdTemp];
                            // temp variable
                            std::vector<uint64_t> tempWin_KidSet;
                            tempWin_KidSet.push_back(KframeIdTemp);
                            std::vector<int> tempWin_KindexSet;
                            tempWin_KindexSet.push_back(KframeIndexTemp);
                            // update the landmarks
                            landMarks[tempLid].Win_KidSet = tempWin_KidSet;
                            landMarks[tempLid].Win_KindexSet = tempWin_KindexSet;
                            landMarks[tempLid].Ksize = 1;
                            landMarks[tempLid].Win_added = false;
                            SmartStereoFactor::shared_ptr ttt(new SmartStereoFactor(cam_noise_model, paramsSF, body_P_sensor));
                            ttt->add(gtsam::StereoPoint2(KframeTemp.landLeft[KframeIndexTemp](0),KframeTemp.landRight[KframeIndexTemp](0),
                                                    KframeTemp.landLeft[KframeIndexTemp](1)), X(tempWin_KidSet[0]), Kstereo);
                            landMarks[tempLid].smartfactor = ttt;
                        }
                        // there is no remaining keyframe, remove this landmark
                        else
                            landMarks.erase(tempLid);
                    }
                    else
                        landMarks.erase(tempLid);
                }
            }
            // erase the keyframe to marginalize from window keyframe
            Win_keyFrame.erase(toMargiKid);
            //std::cout << "Marginilize ID: " << toMargiKid << " done." << std::endl;
        }
        return;
    }
};
// main
int main(int argc, char **argv) {
    ros::init(argc, argv, "smart_smoother");
    ros::NodeHandle nh;
    VINSmart_estimator *smartS_estimator = new VINSmart_estimator(nh);
    ROS_INFO("\033[1;32m----> VIO Started.\033[0m");
    smartS_estimator->optimizeLoop();

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
