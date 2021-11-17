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

#include <visual_frontend/StereoFeatureMatches.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


class VIO{
    protected:
        ros::Subscriber f_sub;

        ros::Time current_frame_time;
        ros::Time last_frame_time;
        ros::Time last_imu_time;

        bool initd;
        bool newKeyframe;
        std::queue<keyframe> keyframeBuffer;
    public:
        VIO(ros::NodeHandle& nh): last_frame_time(0.0), initd(false), newKeyframe(false){
            nh.getParam("vio/frontend_topic", frontendTopic);
            stereoMatches_sub = nh.subscribe(frontendTopic, 100, &VIO::stereoMatchesHandle, this);
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
         
         // set frontend screening parameter: for robustness
         nh.getParam("vio/robust_feature_ratio", featRatio);
         double nThru_;
         nh.getParam("vio/number_frame_tracked", nThru_);
         N_thru = nThru_;
         nh.getParam("vio/triangulation_landmark_distance_threshold", triangulationParam.landmarkDistanceThreshold);
         nh.getParam("vio/triangulation_outlier_rejection_threshold", triangulationParam.dynamicOutlierRejectionThreshold);

        }

        void sysInit(const Keyframe& first_keyframe){
            current_frame_time = first_keyframe.timestamp;
            last_frame_time = current_frame_time;

            if(odomSuccess == false)
                return;

            initd = true;

            std::cout << "system initialization done. " << std::endl;

            // set vio parameters after found first odomMsg
        }


        

        //Constructor


};

int main(int argc, char **argv) {
    ros::init(argc, argv, "vio");
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> VIO Started.\033[0m");

    std::cout << "Hola VIO " << std::endl;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
