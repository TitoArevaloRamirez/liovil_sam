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
#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include "visual_frontend/feature_operations.h"
#include "visual_frontend/stereo_tracker.h"
#include "visual_frontend/frame.h"
#include "visual_frontend/optimizer.h"
#include "liovil_sam/StereoFeatureMatches.h"
#include "visual_frontend/parameters.h"
#include "visual_frontend/feature_operations.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <mutex>

using namespace sensor_msgs;
using namespace message_filters;

std::mutex mtx;

//std::string time_path = "time_frontend.txt";
//std::ofstream outFile_time(time_path);

std::shared_ptr<VisualFrontend::StereoTracker> stereo_tracker;

std::string leftImg_topic;
std::string rightImg_topic;
std::string stereoMatches_topic;


void left_img_callback(const sensor_msgs::Image::ConstPtr& msg) {  
  
  std::shared_ptr<cv::Mat> img(new cv::Mat);
  *img = cv_bridge::toCvShare(msg,msg->encoding.c_str())->image.clone();
  stereo_tracker->add_left_img(img,msg->header.stamp.toSec());
  
}

void right_img_callback(const sensor_msgs::Image::ConstPtr& msg) {
  
  std::shared_ptr<cv::Mat> img(new cv::Mat);
  *img = cv_bridge::toCvShare(msg,msg->encoding.c_str())->image.clone();
  stereo_tracker->add_right_img(img,msg->header.stamp.toSec());
  
}

void readInParams(ros::NodeHandle& nh) {

  // corner detector
  double numCorner, addlCorners; 
  double mmatches;

  nh.getParam("corner_number", numCorner);
  Params::num_corners = (unsigned)numCorner;
  nh.getParam("addl_corners", addlCorners);
  Params::addl_corners = (unsigned)addlCorners;

  nh.getParam("percent_matches", Params::percent_matches);
  nh.getParam("percent_min_corners", Params::percent_min_corners);
  nh.getParam("max_matches",mmatches);
  Params::max_matches = (unsigned)mmatches;
  nh.getParam("feature_extraction_boundary", Params::feature_extraction_boundary);
  nh.getParam("feature_search_boundary", Params::feature_search_boundary);
  nh.getParam("min_dist_from_camera", Params::min_dist_from_camera);
  nh.getParam("max_disparity", Params::max_disparity);
  nh.getParam("equalize_histogram", Params::equalize_histogram);
  nh.getParam("refine_corners_after_corner_extraction", Params::refine_corners_after_corner_extraction);
  nh.getParam("refine_corners_after_optical_flow", Params::refine_corners_after_optical_flow);
  nh.getParam("max_disparity_alignment_threshold", Params::max_disparity_alignment_threshold);
  nh.getParam("landmark_search_radius", Params::landmark_search_radius);

  double patch_numC, patch_numR, reExtractPatchNum;
  nh.getParam("patch_num_c", patch_numC);
  nh.getParam("patch_num_r", patch_numR);
  nh.getParam("reextract_patch_number", reExtractPatchNum);
  Params::patch_num_r = (int)patch_numR;
  Params::patch_num_c = (int)patch_numC;
  Params::reextract_patch_num = (int)reExtractPatchNum;
  nh.getParam("matcher_distance_threshold", Params::matcher_dist_threshold);
  //methods
  nh.getParam("old_hybrid",Params::old_hybrid);
  nh.getParam("custom_matcher",Params::custom_matcher);
  
  // descriptor
  nh.getParam("descriptor_distance_both_ways", Params::descriptor_distance_both_ways);
  std::string descriptor, tracking_method;
  nh.getParam("descriptor", descriptor);
  if (descriptor == "ORB")
    Params::descriptor_type = VisualFrontend::ORB;
  else if (descriptor == "SURF")
    Params::descriptor_type = VisualFrontend::SURF;
  else if (descriptor == "SIFT")
    Params::descriptor_type = VisualFrontend::SIFT;
  // tracking
  nh.getParam("tracking_method", tracking_method);
  if (tracking_method == "FEATURE_BASED")
    Params::tracking = Params::FEATURE_BASED;
  else if (tracking_method == "OPTICALFLOW_BASED")
    Params::tracking = Params::OPTICALFLOW_BASED;
  else if (tracking_method == "HYBRID_BASED")
    Params::tracking = Params::HYBRID_BASED;
  nh.getParam("hybrid_percent_tracked_corners",Params::hybrid_percent_tracked_corners);
  // LKT based
  double stereo_win, f2f_win;
  nh.getParam("stereo_tracking_search_window", stereo_win);
  nh.getParam("frame2frame_tracking_search_window", f2f_win);
  Params::stereo_tracking_search_window = cv::Size(stereo_win, stereo_win);
  Params::frame2frame_tracking_search_window = cv::Size(f2f_win, f2f_win);
  nh.getParam("percent_border",Params::percent_border);
  // display and output
  nh.getParam("display_tracking_ui", Params::display_tracking_ui);
  nh.getParam("write_matches_to_file", Params::write_matches_to_file);
  nh.getParam("display_verbose_output", Params::display_verbose_output);

  //Topics 
  nh.getParam("leftImg_topic", leftImg_topic);
  nh.getParam("rightImg_topic", rightImg_topic);
  nh.getParam("stereoMatches_topic", stereoMatches_topic);
}

void left_right_callback(const ImageConstPtr& left_msg, const ImageConstPtr& right_msg)
{
  //std::lock_guard<std::mutex> lock(mtx);

  std::shared_ptr<cv::Mat> left_img(new cv::Mat);
  *left_img = cv_bridge::toCvShare(left_msg,left_msg->encoding.c_str())->image.clone();
  stereo_tracker->add_left_img(left_img,left_msg->header.stamp.toSec());
  //stereo_tracker->just_arrived_left = std::make_shared<Frame>(left_img,left_msg->header.stamp.toSec(),VisualFrontend::LEFT);

  std::shared_ptr<cv::Mat> right_img(new cv::Mat);
  *right_img = cv_bridge::toCvShare(right_msg,right_msg->encoding.c_str())->image.clone();
  stereo_tracker->add_right_img(right_img,right_msg->header.stamp.toSec());
  //stereo_tracker->just_arrived_right = std::make_shared<Frame>(right_img,right_msg->header.stamp.toSec(),VisualFrontend::RIGHT);
  // Solve all of perception here...
}

int main(int argc, char **argv) {

  ros::init(argc,argv,"visual_frontend");

  ros::NodeHandle nh("~");
  readInParams(nh);

  ros::AsyncSpinner spinner(4);

  

  stereo_tracker.reset(new VisualFrontend::StereoTracker());
  stereo_tracker->visualize = false;
   
  //add subscribers to left image, pointcloud and imu data
  //ros::Subscriber left_sub = nh.subscribe(leftImg_topic, 30, left_img_callback);
  //ros::Subscriber right_sub = nh.subscribe(rightImg_topic, 30, right_img_callback);
  
  message_filters::Subscriber<Image> leftImg_sub(nh, leftImg_topic, 1);
  message_filters::Subscriber<Image> rightImg_sub(nh, rightImg_topic, 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(3), leftImg_sub, rightImg_sub);
  sync.registerCallback(boost::bind(&left_right_callback, _1, _2));

  // publisher
  ros::Publisher stereo_matches_pub = nh.advertise<liovil_sam::StereoFeatureMatches>(stereoMatches_topic,10);

  ROS_INFO("\033[1;36m \n >>> Visual Fronten Started <<< \033[0m");
  std::cout<< "\033[0;36m \t Subscribed to (left image): " << leftImg_topic << "\033[0m" << std::endl;
  std::cout<< "\033[0;36m \t Subscribed to (right image): " << rightImg_topic << "\033[0m" << std::endl;
  std::cout<< "\033[0;36m \t Publish to: " << stereoMatches_topic << "\033[0m" << std::endl;


  std::set<unsigned> prev_feature_ids;
  std::shared_ptr<VisualFrontend::FMatches> matches;

  spinner.start();
  //clock_t start;
  while(ros::ok()) {

    //auto start_clock = std::chrono::high_resolution_clock::now();

    //start = clock();
    stereo_tracker->track();
    
    //if stereo matches were generated
    if(stereo_tracker->stereo_feature_matches) {
    
       matches = stereo_tracker->stereo_feature_matches;
      //publish stereo messages
      if(matches->matches.size() > 0) {
        //publish the matched features
        //setting the left frame timestamp
        liovil_sam::StereoFeatureMatches matches_msg;
        matches_msg.header.stamp = ros::Time(matches->frame1->tstamp); 
        matches_msg.frame_id = matches->frame1->frameid;
        matches_msg.num_matches = matches->matches.size();

        std::set<unsigned> feature_ids;
        std::set<unsigned>::iterator feat_it;
        std::pair<std::set<unsigned>::iterator,bool> ret;

        //std::cout << "Publishing features:";
        int repeat_features_count = 0;

        for(auto iter=matches->matches.begin(); iter!=matches->matches.end(); ++iter) {
          //std::cout << "id:" << iter->id << std::endl;
          matches_msg.feature_ids.push_back((*iter)->f1->id);
          matches_msg.left_xs.push_back((*iter)->f1->lpix.x / Params::rescale_factor );
          matches_msg.left_ys.push_back((*iter)->f1->lpix.y / Params::rescale_factor);
          matches_msg.right_xs.push_back((*iter)->f2->lpix.x / Params::rescale_factor);
          matches_msg.right_ys.push_back((*iter)->f2->lpix.y / Params::rescale_factor);
          if(!std::isnan( (*iter)->f1->depth) ) 
            matches_msg.depths.push_back((*iter)->f1->depth);
          
          ret = feature_ids.insert((*iter)->f1->id);
          //std::cout << (*iter)->f1->id << ", ";
          if(ret.second == false) {
            std::cerr << "\033[1;31m Warning: Duplicate feature id: " << (*iter)->f1->id
                << "\033[0m" << std::endl;
            exit(-1);
          }

          if(prev_feature_ids.size() != 0) {
            //ignore first iteration
            ret = prev_feature_ids.insert((*iter)->f1->id);
            if(ret.second == false) {
              repeat_features_count++;
            }
          }
        }
        //std::cout << std::endl;
        stereo_matches_pub.publish(matches_msg);

        //std::cout << "Features tracked from previous frame:" << repeat_features_count << std::endl;
        prev_feature_ids = feature_ids;

        //after we consumed the stereo_feature_matches, delete it
        matches.reset();
        stereo_tracker->stereo_feature_matches.reset();
        stereo_tracker->tracking_feature_matches.reset();

	    //auto end_clock = std::chrono::high_resolution_clock::now();
	    //std::cout << "Total frontend one pair time:" << VisualFrontend::duration(start_clock,end_clock) << "ms" << std::endl;

        //float diff = -((float)start-(float)clock())/CLOCKS_PER_SEC;
        //std::cout << "Total frontend one pair time: " << diff*1.0e3 << "ms\n\n";
	
        //outFile_time << diff << "\n";
        //outFile_time.close();
        //outFile_time.open(time_path, std::ios_base::app);
      }
    }
    //ros::spinOnce();
  }
  ros::waitForShutdown();
  return 0;
}
