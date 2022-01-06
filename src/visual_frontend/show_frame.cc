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
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

#include <liovil_sam/StereoFeatureMatches.h>

void frame_callback(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight, 
                const liovil_sam::StereoFeatureMatches::ConstPtr& msg) {
    // get image
    cv::Mat grayLeft, grayRight;
    cv_bridge::CvImagePtr cv_ptrLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImagePtr cv_ptrRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8);
    grayLeft = cv_ptrLeft->image;
    grayRight = cv_ptrRight->image;
    
    std::vector<cv::Point2f> forLinesl, forLinesr;
    // visualize pts set
    for ( size_t i=0; i<msg->num_matches; i++ ) {
        cv::Point2f ptsl = cv::Point2f(msg->left_xs[i], msg->left_ys[i]);
        cv::Point2f ptsr = cv::Point2f(msg->right_xs[i], msg->right_ys[i]);
        cv::circle(grayLeft, ptsl, 3, cv::Scalar( 0,255,0 ), 2, 8, 0 );
        cv::circle(grayRight, ptsr, 3, cv::Scalar( 0,255,0 ), 2, 8, 0 );
        // draw lines preparation
		cv::Point2f right_pt = (ptsr + cv::Point2f((float)grayLeft.cols, 0.f));
        forLinesl.push_back(ptsl);
        forLinesr.push_back(right_pt);
    }
    // stick left and right
    cv::Size sz1 = grayLeft.size(), sz2 = grayRight.size();
    cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3); // CV_8UC3
    cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
    grayLeft.copyTo(left);
    cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
    grayRight.copyTo(right);
    // draw lines
    for ( size_t i=0; i<forLinesl.size(); i++ )
      cv::line(im3, forLinesl[i], forLinesr[i], cv::Scalar(0, 255, 255), 2);
    // imshow
    cv::imshow("frontend matches", im3);
    cv::resizeWindow("frontend matches",cv::Size(1340,540));
    cv::waitKey(1);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "show_frame_node");
    ros::NodeHandle nh;

    std::string frontendTopic;
    //nh.getParam("vio/frontend_topic", frontendTopic);
    nh.param<std::string>("vio/frontend_topic", frontendTopic, "/stereo_matches");

    ROS_INFO("\033[1;36m\n >>> Show Frame Started <<< \033[0m");
  
  	// register callback
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/mapping/left/scaled/image_rect_color", 100);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/mapping/right/scaled/image_rect_color", 100);
    message_filters::Subscriber<liovil_sam::StereoFeatureMatches> f_sub(nh, frontendTopic, 100);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, liovil_sam::StereoFeatureMatches> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub, f_sub);
    sync.registerCallback( boost::bind(frame_callback,_1,_2,_3) );

    cv::namedWindow("frontend matches", cv::WINDOW_NORMAL);
    cv::startWindowThread();
    
    ros::spin();
    return 0;
}
