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
#include "visual_frontend/StereoFeatureMatches.h"
#include "visual_frontend/parameters.h"
#include "visual_frontend/feature_operations.h"

double frame_rate;
double tolerance;
double prev_left_time=0;
double prev_right_time=0;

std::string time_path = "time_frontend.txt";
std::ofstream outFile_time(time_path);

std::shared_ptr<VisualFrontend::StereoTracker> stereo_tracker;

void left_img_callback(const sensor_msgs::Image::ConstPtr& msg) {  
  
  std::shared_ptr<cv::Mat> img(new cv::Mat);
  *img = cv_bridge::toCvShare(msg,msg->encoding.c_str())->image.clone();
  
  double curr_left_time = msg->header.stamp.toSec();
  if(prev_left_time != 0) {
    if(curr_left_time - prev_left_time > (1.+tolerance)/frame_rate) {
      std::cerr << "Time gap larger. Lost a left frame" << std::endl;
    }
  }
  prev_left_time = curr_left_time;

  stereo_tracker->add_left_img(img,msg->header.stamp.toSec());
  
}

void right_img_callback(const sensor_msgs::Image::ConstPtr& msg) {
  
  std::shared_ptr<cv::Mat> img(new cv::Mat);
  *img = cv_bridge::toCvShare(msg,msg->encoding.c_str())->image.clone();
  
  double curr_right_time = msg->header.stamp.toSec();
  if(prev_right_time != 0) {
    if(curr_right_time - prev_right_time > (1.+tolerance)/frame_rate) {
      std::cerr << "Time gap larger. Lost a right frame" << std::endl;
    }
  }
  prev_right_time = curr_right_time;

  stereo_tracker->add_right_img(img,msg->header.stamp.toSec());
  
}

void readInParams(ros::NodeHandle& nh) {
  // camera
  double stereo_fx, stereo_fy, stereo_cx, stereo_cy;
  nh.getParam("rescale_factor", Params::rescale_factor);
  nh.getParam("fx", stereo_fx);
  nh.getParam("fy", stereo_fy);
  nh.getParam("cx", stereo_cx);
  nh.getParam("cy", stereo_cy);
  nh.getParam("baseline", Params::right_cam_info.baseline);
  Params::left_cam_info.fx = stereo_fx*Params::rescale_factor;
  Params::left_cam_info.fy = stereo_fy*Params::rescale_factor;
  Params::left_cam_info.cx = stereo_cx*Params::rescale_factor;
  Params::left_cam_info.cy = stereo_cy*Params::rescale_factor;
  Params::right_cam_info.fx = stereo_fx*Params::rescale_factor;
  Params::right_cam_info.fy = stereo_fy*Params::rescale_factor;
  Params::right_cam_info.cx = stereo_cx*Params::rescale_factor;
  Params::right_cam_info.cy = stereo_cy*Params::rescale_factor;
  // corner detector
  double numCorner, addlCorners; 
  nh.getParam("corner_number", numCorner);
  Params::num_corners = (unsigned)numCorner;
  nh.getParam("addl_corners", addlCorners);
  Params::addl_corners = (unsigned)addlCorners;
  nh.getParam("percent_matches", Params::percent_matches);
  nh.getParam("percent_min_corners", Params::percent_min_corners);
  double mmatches;
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
  /*
  // printParams
  std::cout << Params::percent_matches << "\n" << Params::percent_min_corners << "\n" 
            << Params::max_disparity << "\n" << Params::min_dist_from_camera << "\n"
            << Params::num_corners << "\n" << Params::feature_extraction_boundary << "\n";
  */
}

int main(int argc, char **argv) {

  ros::init(argc,argv,"visual_frontend");

  ros::NodeHandle nh("~");
  readInParams(nh);

  stereo_tracker.reset(new VisualFrontend::StereoTracker());
  stereo_tracker->write_to_file = false;
  stereo_tracker->visualize = true;
  frame_rate = 20; //hz
  tolerance = 0.3; //30 percent
   
  //add subscribers to left image, pointcloud and imu data
  ros::Subscriber left_sub = nh.subscribe("/mapping/left/scaled/image_rect_color",
					  10,left_img_callback);
  ros::Subscriber right_sub = nh.subscribe("/mapping/right/scaled/image_rect_color",
					   10,right_img_callback);
  // publisher
  ros::Publisher stereo_matches_pub =
    nh.advertise<visual_frontend::StereoFeatureMatches>("/stereo_matches",1000);

  unsigned published_id = 0;

  Eigen::Matrix4d T__k__0(Eigen::Matrix4d::Identity());

  std::set<unsigned> prev_feature_ids;

  clock_t start;
  while(ros::ok()) {

    auto start_clock = std::chrono::high_resolution_clock::now();

    start = clock();
    stereo_tracker->track();
    
    //if stereo matches were generated
    if(stereo_tracker->stereo_feature_matches) {
    
      std::shared_ptr<VisualFrontend::FMatches> matches = stereo_tracker->stereo_feature_matches;
      //publish stereo messages
      if(matches->matches.size() > 0) {
        //publish the matched features
        visual_frontend::StereoFeatureMatches matches_msg;
        //setting the left frame timestamp
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
        stereo_tracker->stereo_feature_matches.reset();

	auto end_clock = std::chrono::high_resolution_clock::now();
	std::cout << "Total frontend one pair time:" <<
	  VisualFrontend::duration(start_clock,end_clock)
		  << "ms" << std::endl;

        float diff = -((float)start-(float)clock())/CLOCKS_PER_SEC;
        std::cout << "Total frontend one pair time: " << diff*1.0e3 << "ms\n\n";
	
        outFile_time << diff << "\n";
        outFile_time.close();
        outFile_time.open(time_path, std::ios_base::app);
      }
    }
    ros::spinOnce();
  }
  
  return 0;
}
