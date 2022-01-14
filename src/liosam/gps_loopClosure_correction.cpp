#include "utility.h"

//#include "utility_vio.h" //usr for vio

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
//using gtsam::symbol_shorthand::G; // GPS pose


struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

typedef PointXYZIRPYT  PointTypePose;

//class TransformFusion : public ParamServer
//{
//public:
//    std::mutex mtx;
//
//    ros::Subscriber subImuOdometry;
//    ros::Subscriber subLaserOdometry;
//
//    ros::Publisher pubImuOdometry;
//    ros::Publisher pubImuPath;
//
//    Eigen::Affine3f lidarOdomAffine;
//    Eigen::Affine3f imuOdomAffineFront;
//    Eigen::Affine3f imuOdomAffineBack;
//
//    tf::TransformListener tfListener;
//    tf::StampedTransform lidar2Baselink;
//
//    double lidarOdomTime = -1;
//    deque<nav_msgs::Odometry> imuOdomQueue;
//
//    TransformFusion()
//    {
//        if(lidarFrame != baselinkFrame)
//        {
//            try
//            {
//                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
//                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
//            }
//            catch (tf::TransformException ex)
//            {
//                ROS_ERROR("%s",ex.what());
//            }
//        }
//        //subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
//
//        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
//
//        //subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
//        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>("/odom_lidar_usr_sync",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());
//
//        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
//        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
//    }
//
//    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
//    {
//        double x, y, z, roll, pitch, yaw;
//        x = odom.pose.pose.position.x;
//        y = odom.pose.pose.position.y;
//        z = odom.pose.pose.position.z;
//        tf::Quaternion orientation;
//        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
//        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
//        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
//    }
//
//    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
//    {
//        std::lock_guard<std::mutex> lock(mtx);
//
//        lidarOdomAffine = odom2affine(*odomMsg);
//
//        lidarOdomTime = odomMsg->header.stamp.toSec();
//    }
//
//    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
//    {
//        // static tf
//        static tf::TransformBroadcaster tfMap2Odom;
//        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
//        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));
//
//        std::lock_guard<std::mutex> lock(mtx);
//
//        imuOdomQueue.push_back(*odomMsg);
//
//        // get latest odometry (at current IMU stamp)
//        if (lidarOdomTime == -1)
//            return;
//        while (!imuOdomQueue.empty())
//        {
//            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
//                imuOdomQueue.pop_front();
//            else
//                break;
//        }
//        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
//        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
//        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
//        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
//        float x, y, z, roll, pitch, yaw;
//        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
//        
//        // publish latest odometry
//        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
//        laserOdometry.pose.pose.position.x = x;
//        laserOdometry.pose.pose.position.y = y;
//        laserOdometry.pose.pose.position.z = z;
//        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
//        pubImuOdometry.publish(laserOdometry);
//
//        // publish tf
//        static tf::TransformBroadcaster tfOdom2BaseLink;
//        tf::Transform tCur;
//        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
//        if(lidarFrame != baselinkFrame)
//            tCur = tCur * lidar2Baselink;
//        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
//        tfOdom2BaseLink.sendTransform(odom_2_baselink);
//
//        // publish IMU path
//        static nav_msgs::Path imuPath;
//        static double last_path_time = -1;
//        double imuTime = imuOdomQueue.back().header.stamp.toSec();
//        if (imuTime - last_path_time > 0.1)
//        {
//            last_path_time = imuTime;
//            geometry_msgs::PoseStamped pose_stamped;
//            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
//            pose_stamped.header.frame_id = odometryFrame;
//            pose_stamped.pose = laserOdometry.pose.pose;
//            imuPath.poses.push_back(pose_stamped);
//            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
//                imuPath.poses.erase(imuPath.poses.begin());
//            if (pubImuPath.getNumSubscribers() != 0)
//            {
//                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
//                imuPath.header.frame_id = odometryFrame;
//                pubImuPath.publish(imuPath);
//            }
//        }
//    }
//};

class IMUPreintegration : public ParamServer
{
public:

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;


    std::mutex mtx;

    float transformTobeMapped[6];

    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Subscriber subStereo;  //usr
    ros::Subscriber subOdometryByPass; //usr

    ros::Subscriber subGPS;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubLidarOdometry;


    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorFrame_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;
    std::deque<sensor_msgs::Imu> imuQueFrame;

    std::deque<nav_msgs::Odometry> stereoQueue;  //usr
    std::deque<nav_msgs::Odometry> gpsQueue;

    gtsam::Pose3 lastOpt_pose;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::Pose3 prevLidarPose;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    bool readKeyFrames = false; //usr
    bool inFrame_callback = false; //usr
    bool getFirstPose = false; //usr
    bool aLoopIsClosed = false;

    nav_msgs::Odometry prevPose_lidar;
    nav_msgs::Odometry currPose_lidar;

    bool odometryByPass = false;

    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double lastImuT_Frame = -1;

    gtsam::ISAM2 *optimizer;
    gtsam::NonlinearFactorGraph *graphFactors;
    gtsam::Values graphValues;
    gtsam::Values result;

    const double delta_t = 0;

    int key = 1;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    gtsam::Pose3 camera2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0.0870551272380779, -0.107604788194452, 0.0180391607070435));


    IMUPreintegration(){

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        graphFactors = new gtsam::NonlinearFactorGraph();

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;

        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = new gtsam::ISAM2(optParameters);

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }


        //subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic, 2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        //subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        //subOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 500,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 500,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        //subOdometryByPass = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 500,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        //subOdometry = nh.subscribe<nav_msgs::Odometry>("liovil_sam_smart_smoother/odom_camera", 300,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); //usr

        //subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom_camera", 300,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); //usr
        
        //subStereo = nh.subscribe<nav_msgs::Odometry> ("/odom_camera", 200, &IMUPreintegration::stereoHandler, this, ros::TransportHints().tcpNoDelay()); //usr
        //
        subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 2000, &IMUPreintegration::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        //
        //subStereo = nh.subscribe<nav_msgs::Odometry> ("liovil_sam_smart_smoother/odom_camera", 2000, &IMUPreintegration::stereoHandler, this, ros::TransportHints().tcpNoDelay()); //usr

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);
        pubLidarOdometry = nh.advertise<nav_msgs::Odometry> ("/odom_opt_gps", 2000);

        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
        imuIntegratorFrame_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization        
        
    }

    //void resetOptimization()
    //{
    //    gtsam::ISAM2Params optParameters;
    //    optParameters.relinearizeThreshold = 0.1;
    //    optParameters.relinearizeSkip = 1;
    //    optimizer = gtsam::ISAM2(optParameters);

    //    gtsam::NonlinearFactorGraph newGraphFactors;
    //    graphFactors = newGraphFactors;

    //    gtsam::Values NewGraphValues;
    //    graphValues = NewGraphValues;
    //}

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        readKeyFrames = false; //usr
        systemInitialized = false;
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[]) {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void addGPSFactor(double timeLaserInfoCur)
    {
        if (gpsQueue.empty())
            return;

        if (cloudKeyPoses3D->points.empty()){
            cout<<"No key Poses" <<endl;
            return;
        }
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0){ //5.0){
                cout<<"Cloud key poses lower than 5 mts" <<endl;
                return;
            }
        }

        // pose covariance small, no need to correct
        //if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold){
        //    cout<<"Lidar pose covariance lower than: "<< poseCovThreshold <<endl;
        //    return;
        //}

        // last gps position
        static PointType lastGPSPoint;

        //cout<<"While loop: " << !gpsQueue.empty() <<endl;
        while (!gpsQueue.empty())
        {
            if (gpsQueue.front().header.stamp.toSec() < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (gpsQueue.front().header.stamp.toSec() > timeLaserInfoCur + 0.2)
            { // message too new
                //cout<<"GPS too new" <<endl;
                break;
            }
            else
            {
                nav_msgs::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = 0.1; //thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold){
                    //cout<<"GPS noise grater than: "<< gpsCovThreshold <<endl;
                    continue;
                }

                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                
                /*not implemented here
                  if (!useGpsElevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }
                */
                //noise_z = 0.01;

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6){
                    //cout<<"GPS not properly initialized" << endl;
                    continue;
                }

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 3.0){
                    //cout<<"Distance between GPS points lower than: "<< 1.0 << endl;
                    continue;
                }
                else{
                    lastGPSPoint = curGPSPoint;
                }

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 0.01f), max(noise_y, 0.10f), max(noise_z, 0.01f);
                //Vector3 << 0.01f, 0.10f, 0.01f;
                gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(X(key), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                graphFactors->add(gps_factor);

                aLoopIsClosed = true;
                std::cout << "\033[1;35m---->Imu Preintegration:  GPS added\033[0m" << std::endl;
                break;
            }
        }
        //cout<<"chao GPS add" <<endl;
    }

    void stereoHandler(const nav_msgs::Odometry::ConstPtr& stereoOdom){ //usr3
        std::lock_guard<std::mutex> lock(mtx);


        nav_msgs::Odometry thisStereo = *stereoOdom;
        // set the orientation
        float q_x = stereoOdom->pose.pose.orientation.x;     //usr
        float q_y = stereoOdom->pose.pose.orientation.y;     //usr
        float q_z = stereoOdom->pose.pose.orientation.z;     //usr
        float q_w = stereoOdom->pose.pose.orientation.w;     //usr

        std::vector<double> extRotV{1, 0, 0,        //usr
                               0, -1, 0,        //usr
                               0, 0, -1};       //usr
        Eigen::Matrix3d extRot;     //usr
        Eigen::Quaterniond extQRPY;     //usr
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);        //usr
        extQRPY = Eigen::Quaterniond(extRot);       //usr
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(q_w, q_x, q_y, q_z);      //usr
        Eigen::Quaterniond q_final = q_from * extQRPY;      //usr

        float r_x = q_final.x();        //usr
        float r_y = q_final.y();        //usr
        float r_z = q_final.z();        //usr
        float r_w = q_final.w();        //usr

        thisStereo.pose.pose.orientation.x = r_x;
        thisStereo.pose.pose.orientation.y = r_y;
        thisStereo.pose.pose.orientation.z = r_z;
        thisStereo.pose.pose.orientation.w = r_w;

        stereoQueue.push_back(thisStereo);

        //std::cout << "\033[1;33m----> new Stereo Odom\033[0m" << std::endl;
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        //std::cout<<"Hola odometry handler"<< std::endl;

        double currentCorrectionTime = ROS_TIME(odomMsg);
       
        float p_x = odomMsg->pose.pose.position.x;        //usr
        float p_y = odomMsg->pose.pose.position.y;        //usr
        float p_z = odomMsg->pose.pose.position.z;        //usr
        float r_x = odomMsg->pose.pose.orientation.x;     //usr
        float r_y = odomMsg->pose.pose.orientation.y;     //usr
        float r_z = odomMsg->pose.pose.orientation.z;     //usr
        float r_w = odomMsg->pose.pose.orientation.w;     //usr


        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

        // 0. initialize system
        if (systemInitialized == false)
        {
            //resetOptimization();
            //
            // initial pose
            prevLidarPose = lidarPose;

            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), lidarPose, priorPoseNoise);
            graphFactors->add(priorPose);
            
            // add values
            graphValues.insert(X(0), lidarPose);

            optimizer->update(*graphFactors, graphValues);
            graphFactors->resize(0);
            graphValues.clear();
             
            lastOpt_pose = lidarPose;
            key = 1;
            systemInitialized = true;
            return;
        }

       gtsam::BetweenFactor<gtsam::Pose3> pose_factor(X(key-1), X(key), lastOpt_pose.between(lidarPose), degenerate ? correctionNoise2 : correctionNoise);
       graphFactors->add(pose_factor);
       graphValues.insert(X(key), lidarPose);
        

        addGPSFactor(currentCorrectionTime); //usr

        //add landmarks stereo
        //addStereo_new(currentCorrectionTime, key); //usr

        // optimize
        optimizer->update(*graphFactors, graphValues);
        optimizer->update();
        std::cout<< "\033[1;32m\n >>> affter optimize: \033[0m"<< std::endl;

        if (aLoopIsClosed == true)
        {
            optimizer->update();
            optimizer->update();
            optimizer->update();
            optimizer->update();
            optimizer->update();
            aLoopIsClosed = false;
            std::cout<< "\033[1;35m\n >>> Loop Closed true and false: \033[0m"<< std::endl;
        }

        graphFactors->resize(0);
        graphValues.clear();

        // Overwrite the beginning of the preintegration for the next step.
        result  = optimizer->calculateEstimate();
        
        lastOpt_pose = result.at<gtsam::Pose3>(X(key));

        PointType thisPose3D;
        PointTypePose thisPose6D;

        thisPose3D.x = lastOpt_pose.translation().x();
        thisPose3D.y = lastOpt_pose.translation().y();
        thisPose3D.z = lastOpt_pose.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        // publish odometry 

        nav_msgs::Odometry odometry;
        odometry.header.stamp = odomMsg->header.stamp;
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "odom_lidar_usr";

        odometry.pose.pose.position.x = lastOpt_pose.translation().x();
        odometry.pose.pose.position.y = lastOpt_pose.translation().y();
        odometry.pose.pose.position.z = lastOpt_pose.translation().z();
        odometry.pose.pose.orientation.x = lastOpt_pose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lastOpt_pose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lastOpt_pose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lastOpt_pose.rotation().toQuaternion().w();
        
        pubLidarOdometry.publish(odometry);

        ++key;
        doneFirstOpt = true;
        readKeyFrames = true; //usr

    }

   // void correctPoses()
   // {
   //     if (key==0)
   //         return;

   //     if (aLoopIsClosed == true)
   //     {
   //         // update key poses
   //         int numPoses = result.size();
   //         for (int i = 0; i < numPoses; ++i)
   //         {
   //             cloudKeyPoses3D->points[i].x = result.at<Pose3>(i).translation().x();
   //             cloudKeyPoses3D->points[i].y = result.at<Pose3>(i).translation().y();
   //             cloudKeyPoses3D->points[i].z = result.at<Pose3>(i).translation().z();

   //             cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
   //             cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
   //             cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
   //             cloudKeyPoses6D->points[i].roll  = result.at<Pose3>(i).rotation().roll();
   //             cloudKeyPoses6D->points[i].pitch = result.at<Pose3>(i).rotation().pitch();
   //             cloudKeyPoses6D->points[i].yaw   = result.at<Pose3>(i).rotation().yaw();
   //         }

   //         aLoopIsClosed = false;
   //     }
   // }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void addStereoFactor(double correctionTime){ //usr
        if(stereoQueue.empty())
            return;
        //if (key==0)
        //    return;

        while (!stereoQueue.empty())
        {
            if (stereoQueue.front().header.stamp.toSec() < correctionTime - 0.1)
            {
                // message too old
                stereoQueue.pop_front();
            }
            else if (stereoQueue.front().header.stamp.toSec() > correctionTime + 0.1)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::Odometry thisStereo = stereoQueue.front();
                stereoQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisStereo.pose.covariance[0];
                float noise_y = thisStereo.pose.covariance[7];
                float noise_z = thisStereo.pose.covariance[14];

                float noise_roll = thisStereo.pose.covariance[21];
                float noise_pitch = thisStereo.pose.covariance[28];
                float noise_yaw = thisStereo.pose.covariance[35];
                //if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                //    continue;

                float p_x = thisStereo.pose.pose.position.x;
                float p_y = thisStereo.pose.pose.position.y;
                float p_z = thisStereo.pose.pose.position.z;

                float q_x = thisStereo.pose.pose.orientation.x;
                float q_y = thisStereo.pose.pose.orientation.y;
                float q_z = thisStereo.pose.pose.orientation.z;
                float q_w = thisStereo.pose.pose.orientation.w;

                gtsam::Vector Vector6(6);
                //Vector6 << max(noise_roll, 1.0f), max(noise_pitch, 1.0f), max(noise_yaw, 1.0f), max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                Vector6 << 0.5, 0.5, 0.5, 0.1, 0.1, 0.1;
                gtsam::noiseModel::Diagonal::shared_ptr stereo_noise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                
                gtsam::Pose3 stereoPose = gtsam::Pose3(gtsam::Rot3::Quaternion(q_w, q_x, q_y, q_z), gtsam::Point3(p_x, p_y, p_z));

                gtsam::PriorFactor<gtsam::Pose3> stereo_factor(X(key), stereoPose, stereo_noise);
                graphFactors->add(stereo_factor);

                std::cout << "\033[1;34m---->ImuPreintegration: stereo factor added \033[0m" << std::endl;
                break;
            }
        }

    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw); 
        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);
        imuQueFrame.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        //odometry.header.frame_id = odometryFrame;
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
        
        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
        pubLidarOdometry.publish(odometry);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;

    //TransformFusion TF;

    ROS_INFO("\033[1;36m\n >>> IMU Preintegration Started <<< \033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
