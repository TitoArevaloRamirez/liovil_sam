#include "utility.h"

#include "utility_vio.h" //usr for vio

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

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

    TransformFusion()
    {
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }

        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

        pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        // static tf
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);
        
        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if(lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame, baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Subscriber subStereo;  //usr

    ros::Publisher pubImuOdometry;


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

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    bool readKeyFrames = false; //usr
    bool inFrame_callback = false; //usr
    bool inOdometryHandler = false; //usr
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double lastImuT_Frame = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    gtsam::Pose3 camera2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(0.0870551272380779, -0.107604788194452, 0.0180391607070435));


    // ##################################### start  VIO section ##################################
    
    ros::Subscriber f_sub;

    ros::Publisher odom_camera_pub;
    ros::Publisher camera_path_pub;

    nav_msgs::Path path;

    // state
    state_info last_optimized_state;
    gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model;
    
    // keeep data
    std::map<uint64_t, keyframe> Win_keyFrame; // Kid to keyframe
    std::map<uint64_t, landmark> landMarks; // Lid to landmark
    std::map<uint64_t, candi_feat_info> candi_feature_pool; // feature id to candi_feat_info

    // smart factor
    gtsam::Pose3 body_P_sensor; // left camera to IMU
    gtsam::Cal3_S2Stereo::shared_ptr Kstereo;
    gtsam::noiseModel::Isotropic::shared_ptr cam_noise_model;
    gtsam::SmartStereoProjectionParams paramsSF;
    
    // fixed lag optimzer

    // interface with front end
    bool initd;
    bool newKeyframe;
    std::deque<keyframe> keyframeBuffer;
    //TimeBasedRetriever< Eigen::Matrix<double,11,1> > imuMsg;

    TimeBasedRetriever< Eigen::Matrix<double,11,1> > odomImuMsg; //usr

    ros::Time current_frame_time;
    ros::Time last_frame_time;
    ros::Time last_imu_time;

    // robustness
    double featRatio;
    uint64_t N_thru;
    gtsam::TriangulationParameters triangulationParam;

    // initialization
    bool firstIMU;

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform aftMappedTrans;

    // ##################################### end  VIO section ##################################
    IMUPreintegration(): initd(false), newKeyframe(false), odomImuMsg(false), last_frame_time(0.0), last_imu_time(0.0), firstIMU(true){

        subImu      = nh.subscribe<sensor_msgs::Imu>  (imuTopic,                   2000, &IMUPreintegration::imuHandler,      this, ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        //subOdometry = nh.subscribe<nav_msgs::Odometry>("liovil_sam_smart_smoother/odom_camera", 300,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); //usr

        //subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom_camera", 300,    &IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay()); //usr
        
        //subStereo = nh.subscribe<nav_msgs::Odometry> ("/odom_camera", 200, &IMUPreintegration::stereoHandler, this, ros::TransportHints().tcpNoDelay()); //usr
        //
        //
        //subStereo = nh.subscribe<nav_msgs::Odometry> ("liovil_sam_smart_smoother/odom_camera", 2000, &IMUPreintegration::stereoHandler, this, ros::TransportHints().tcpNoDelay()); //usr

        pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

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
        
        // ##################################### start  VIO section ##################################
        std::string imuTopic, frontendTopic, lidarTopic, lcTopic, odomIMU, odomCamera, cameraPoseVisual, cameraPath, landmarkPub; 
        nh.getParam("vio/frontend_topic", frontendTopic);
        nh.getParam("vio/odom_camera_rate_topic", odomCamera);
        nh.getParam("vio/camera_path_publish_topic", cameraPath);
        
        // subscribers
        f_sub = nh.subscribe(frontendTopic, 1000, &IMUPreintegration::frame_callback, this);

        // publishers
        odom_camera_pub = nh.advertise<nav_msgs::Odometry>(odomCamera, 100);
        camera_path_pub = nh.advertise<nav_msgs::Path>(cameraPath, 1000);

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

        // set camera noise
        double camera_noise_sigma;
        nh.getParam("vio/camera_sigma", camera_noise_sigma);
        cam_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, camera_noise_sigma);

        //paramsSF.setEnableEPI(true);
        //paramsSF.setRankTolerance(0);
        //
        //paramsSF.print();
        

        // ##################################### end  VIO section ##################################

    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;

        // ##################################### start VIO section ##################################
        // set parameter for Smart Factor
        double distThresh, outlierRejThresh;
        nh.getParam("vio/landmark_distance_threshold", distThresh);
        nh.getParam("vio/outlier_rejection_threshold", outlierRejThresh);        
        paramsSF.setLinearizationMode(gtsam::LinearizationMode::JACOBIAN_SVD); // HESSIAN not implemented?
        paramsSF.setLandmarkDistanceThreshold(distThresh);
        paramsSF.setDynamicOutlierRejectionThreshold(outlierRejThresh);

        // set frontend screening parameter: for robustness
        nh.getParam("vio/robust_feature_ratio", featRatio);
        double nThru_;
        nh.getParam("vio/number_frame_tracked", nThru_);
        N_thru = nThru_;
        nh.getParam("vio/triangulation_landmark_distance_threshold", triangulationParam.landmarkDistanceThreshold);
        nh.getParam("vio/triangulation_outlier_rejection_threshold", triangulationParam.dynamicOutlierRejectionThreshold);

        // keeep data
        Win_keyFrame.clear(); // Kid to keyframe
        landMarks.clear(); // Lid to landmark
        candi_feature_pool.clear(); // feature id to candi_feat_info
        // ##################################### end VIO section ##################################
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        readKeyFrames = false; //usr
        systemInitialized = false;
        initd = false;
    }
        // ##################################### start VIO section ##################################
        
    void frame_callback(const liovil_sam::StereoFeatureMatches::ConstPtr& msg) {
        if (readKeyFrames == false)
            return;

       // if (inOdometryHandler == true)
       //     return;

        std::lock_guard<std::mutex> lock(mtx);

        inFrame_callback = true;

        //std::cout<< "Hola frame_callback" << std::endl;

        std::map<uint64_t, bool> whether_robust_feat;
        // set triangulation camera
        gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2> > monoCameras;
        gtsam::PinholeBase::MeasurementVector monoMeasured;

        double currentFrameTime = ROS_TIME(msg);

       
        // make sure we have imu data to integrate
        if (imuQueFrame.empty())
            return;
        // system initialized, there is a keyframe already
            // robust feature cover check
            uint32_t num_feat_thru_N_frame = 0;
            uint32_t num_feat_as_landmark = 0;
            uint32_t totalCnt = msg->num_matches;

            // 1. integrate imu data and optimize
            while (!imuQueFrame.empty())
            {
                // pop and integrate imu data that is between last optimized and current frame
                sensor_msgs::Imu *thisImu = &imuQueFrame.front();
                double imuTime = ROS_TIME(thisImu);
                if (imuTime < currentFrameTime - delta_t)
                {
                    double dt = (lastImuT_Frame < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_Frame);
                    imuIntegratorFrame_->integrateMeasurement(
                            gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                            gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                    
                    lastImuT_Frame = imuTime;
                    imuQueFrame.pop_front();
                }
                else
                    break;
            }

            gtsam::NavState pred_cur_pose = imuIntegratorFrame_->predict(prevStateOdom, prevBiasOdom);
            //imuIntegratorFrame_->resetIntegrationAndSetBias(prevBias_);

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

            // no IMU info.
            //if ((pred_cur_pose.t()-last_optimized_state.pose.translation()).norm() == 0)
            //    return;

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
            keyframeBuffer.push_back(KFtoADD);

            //// add imu factor to graph
            //const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorFrame_);
            //gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
            //graphFactors.add(imu_factor);
            //// add imu bias between factor
            //graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
            //                 gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorFrame_->deltaTij()) * noiseModelBetweenBias)));
            //// insert predicted values
            //graphValues.insert(X(key), pred_cur_pose.pose());
            //graphValues.insert(V(key), pred_cur_pose.v());
            //graphValues.insert(B(key), prevBias_);

            ////add landmarks stereo
           //// addStereo_new(currentFrameTime, key); //usr
            //addSmartStereoFactor(KFtoADD, key);

            //// optimize
            //optimizer.update(graphFactors, graphValues);
            //optimizer.update();
            //optimizer.update();
            //optimizer.update();
            //graphFactors.resize(0);
            //graphValues.clear();
            //// Overwrite the beginning of the preintegration for the next step.
            //gtsam::Values result = optimizer.calculateEstimate();
            //prevPose_  = result.at<gtsam::Pose3>(X(key));
            //prevVel_   = result.at<gtsam::Vector3>(V(key));
            //prevState_ = gtsam::NavState(prevPose_, prevVel_);
            //prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
            //// Reset the optimization preintegration object.
            //imuIntegratorFrame_->resetIntegrationAndSetBias(prevBias_);

            //// check optimization
            //if (failureDetection(prevVel_, prevBias_))
            //{
            //    resetParams();
            //    return;
            //}


        //// 2. after optiization, re-propagate imu odometry preintegration
        //prevStateOdom = prevState_;
        //prevBiasOdom  = prevBias_;

        //// first pop imu message older than current correction data
        //double lastImuQT = -1;
        //while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentFrameTime - delta_t)
        //{
        //    lastImuQT = ROS_TIME(&imuQueImu.front());
        //    imuQueImu.pop_front();
        //}
        //// repropogate
        //if (!imuQueImu.empty())
        //{
        //    // reset bias use the newly optimized bias
        //    imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
        //    // integrate imu message from the beginning of this optimization
        //    for (int i = 0; i < (int)imuQueImu.size(); ++i)
        //    {
        //        sensor_msgs::Imu *thisImu = &imuQueImu[i];
        //        double imuTime = ROS_TIME(thisImu);
        //        double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

        //        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
        //                                                gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
        //        lastImuQT = imuTime;
        //    }
        //}

        // ++key;

        return;
    }
        // ##################################### end VIO section ##################################

    void stereoHandler(const nav_msgs::Odometry::ConstPtr& stereoOdom){ //usr3
        std::lock_guard<std::mutex> lock(mtx);


        nav_msgs::Odometry thisStereo = *stereoOdom;
        // set the orientation
        float q_x = stereoOdom->pose.pose.orientation.x;     //usr
        float q_y = stereoOdom->pose.pose.orientation.y;     //usr
        float q_z = stereoOdom->pose.pose.orientation.z;     //usr
        float q_w = stereoOdom->pose.pose.orientation.w;     //usr

        std::vector<double> extRotV{1, 0, 0,        //usr
                               0, 1, 0,        //usr
                               0, 0, 1};       //usr
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

        std::cout << "\033[1;32m----> new Stereo Odom\033[0m" << std::endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {

        std::lock_guard<std::mutex> lock(mtx);
        //std::cout<<"Hola odometry handler"<< std::endl;

        double currentCorrectionTime = ROS_TIME(odomMsg);
       
        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        // set the position
        //float p_x = odomMsg->pose.pose.position.x;      //usr
        //float p_y = odomMsg->pose.pose.position.y;      //usr
        //float p_z = odomMsg->pose.pose.position.z;      //usr
        //// set the orientation

        //float q_x = odomMsg->pose.pose.orientation.x;     //usr
        //float q_y = odomMsg->pose.pose.orientation.y;     //usr
        //float q_z = odomMsg->pose.pose.orientation.z;     //usr
        //float q_w = odomMsg->pose.pose.orientation.w;     //usr

        //std::vector<double> extRotV{1, 0, 0,        //usr
        //                       0, -1, 0,        //usr
        //                       0, 0, -1};       //usr
        //Eigen::Matrix3d extRot;     //usr
        //Eigen::Quaterniond extQRPY;     //usr
        //extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);        //usr
        //extQRPY = Eigen::Quaterniond(extRot);       //usr
        //// rotate roll pitch yaw
        //Eigen::Quaterniond q_from(q_w, q_x, q_y, q_z);      //usr
        //Eigen::Quaterniond q_final = q_from * extQRPY;      //usr
        //
        //float r_x = q_final.x();        //usr
        //float r_y = q_final.y();        //usr
        //float r_z = q_final.z();        //usr
        //float r_w = q_final.w();        //usr


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
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }

            while (!imuQueFrame.empty())
            {
                if (ROS_TIME(&imuQueFrame.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_Frame= ROS_TIME(&imuQueFrame.front());
                    imuQueFrame.pop_front();
                }
                else
                    break;
            }

            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorFrame_->resetIntegrationAndSetBias(prevBias_);
            
            key = 1;
            systemInitialized = true;
            initd = true;
            return;
        }


        // reset graph for speed
        if (key >= 100)
        {
            std::cout<<"100 keys frame lidar odometry" << std::endl;
            readKeyFrames = false; //usr
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
            //std::cout<<" chao 100 keys frame lidar odometry" << std::endl;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                
                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));

        //add stereo factor
        //addStereoFactor(currentCorrectionTime); //usr
        
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        //add landmarks stereo
        addStereo_new(currentCorrectionTime, key); //usr


        // optimize
        optimizer.update(graphFactors, graphValues);
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) :(imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
        readKeyFrames = true; //usr

        //std::cout<<"Key: " << key<< std::endl;  
        //std::cout<<"chao odometry handler"<< std::endl;
    }

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
    void addStereo_new(double correctionTime, int key_current){ //usr
        if(keyframeBuffer.empty())
            return;

        while (!keyframeBuffer.empty())
        {
            keyframe current_key_frame(keyframeBuffer.front());
            
            double keyFrame_time = current_key_frame.timestamp.toSec();

            if (keyFrame_time < correctionTime - 0.1)
            {
                // message too old
                keyframeBuffer.pop_front();
            }
            else if (keyFrame_time > correctionTime + 0.1)
            {
                // message too new
                break;
            }
            else
            {
                keyframeBuffer.pop_front();
                addSmartStereoFactor(current_key_frame, key_current);
                break;

            }

        }
        return;
    }

    void addSmartStereoFactor(const keyframe& curr_frame, int key_current) {
        // add keyframe in sliding window
        //Win_keyFrame[curr_frame.Kid] = curr_frame;
        //
        // Win_keyFrame[key_current] = curr_frame;
        //
        // create camera
        gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2> > monoCameras;
        //std::vector<gtsam::Point2> monoMeasured;
        gtsam::PinholeBase::MeasurementVector monoMeasured;

        // get predicted state
        gtsam::NavState pred_cur_pose = gtsam::NavState(graphValues.at<gtsam::Pose3>(X(key_current)), graphValues.at<gtsam::Vector3>(V(key_current)));

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
                temp.Win_KidSet.push_back(key_current);
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
                temp.smartfactor->add(gtsam::StereoPoint2(curr_frame.landLeft[i](0),curr_frame.landRight[i](0), curr_frame.landLeft[i](1)), X(key_current), Kstereo);
                // added to hash map
                landMarks[curr_frame.LidSet[i]] = temp;
            }
            // has landmark and this landmark has appeared before
            else {
                // add to the factor
                landMarks[curr_frame.LidSet[i]].smartfactor->add(gtsam::StereoPoint2(curr_frame.landLeft[i](0),curr_frame.landRight[i](0),curr_frame.landLeft[i](1)), X(key_current), Kstereo);
                landMarks[curr_frame.LidSet[i]].Win_KidSet.push_back(key_current);
                landMarks[curr_frame.LidSet[i]].Win_KindexSet.push_back(i);
                landMarks[curr_frame.LidSet[i]].Ksize++;
                // check if add to the graph
                if (!landMarks[curr_frame.LidSet[i]].Win_added && landMarks[curr_frame.LidSet[i]].Ksize > 1) {
                    graphFactors.add(landMarks[curr_frame.LidSet[i]].smartfactor);
                    landMarks[curr_frame.LidSet[i]].Win_added = true;
                    ROS_INFO("Land marks added");
                }
            }
        }
        return;
    }

    void addStereoFactor(double correctionTime){ //usr
        if(stereoQueue.empty())
            return;
        if (key==0)
            return;

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
                graphFactors.add(stereo_factor);

                std::cout << "\033[1;32m----> stereo factor added \033[0m" << std::endl;
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
        odometry.header.frame_id = odometryFrame;
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
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;36m\n >>> IMU Preintegration Started <<< \033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
