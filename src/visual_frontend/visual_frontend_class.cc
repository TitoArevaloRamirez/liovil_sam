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

#include <queue>
#include <deque>


#include <thread>

using namespace sensor_msgs;
using namespace message_filters;

std::mutex mtx;

//std::string time_path = "time_frontend.txt";
//std::ofstream outFile_time(time_path);

std::string leftImg_topic;
std::string rightImg_topic;
std::string stereoMatches_topic;

class VisualFrontend_usr{
public:

    VisualFrontend_usr(ros::NodeHandle& nh){      

    readInParams(nh);
    
    leftImg_sub.subscribe(nh, leftImg_topic, 1);
    rightImg_sub.subscribe(nh, rightImg_topic, 1);
    sync.reset(new Sync(MySyncPolicy(10), leftImg_sub, rightImg_sub));  
    sync->registerCallback(boost::bind(&VisualFrontend_usr::left_right_callback, this, _1, _2));
    //sync2.registerCallback(boost::bind(&VisualFrontend_usr::left_right_callback,this, _1, _2));
    
    stereo_matches_pub = nh.advertise<liovil_sam::StereoFeatureMatches>(stereoMatches_topic,1000);

    stereo_tracker.reset(new VisualFrontend::StereoTracker());
    stereo_tracker->visualize = false;

    ROS_INFO("\033[1;36m \n >>> Visual Fronten Started <<< \033[0m");
    //ROS_INFO("\033[0;36m \t Subscribed to (left image): " + leftImg_topic + "\033[0m");
    //std::cout<< "\033[0;36m \t Subscribed to (right image): " << rightImg_topic << "\033[0m" << std::endl;
    //std::cout<< "\033[0;36m \t Publish to: " << stereoMatches_topic << "\033[0m" << std::endl;

}

    void left_right_callback(const ImageConstPtr& left_msg, const ImageConstPtr& right_msg){
    
       std::shared_ptr<cv::Mat> left_img(new cv::Mat);
       *left_img = cv_bridge::toCvShare(left_msg,left_msg->encoding.c_str())->image.clone();
       stereo_tracker->add_left_img(left_img,left_msg->header.stamp.toSec());
    
       std::shared_ptr<cv::Mat> right_img(new cv::Mat);
       *right_img = cv_bridge::toCvShare(right_msg,right_msg->encoding.c_str())->image.clone();
       stereo_tracker->add_right_img(right_img,right_msg->header.stamp.toSec());

    stereo_tracker->track();
    
    //if stereo matches were generated
    if(stereo_tracker->stereo_feature_matches) {
    
       matches = stereo_tracker->stereo_feature_matches;
      //publish stereo messages
      if(matches->matches.size() > 0) {

          matchesQueue.push_back(*matches);


        //after we consumed the stereo_feature_matches, delete it
        matches.reset();
        stereo_tracker->stereo_feature_matches.reset();
        stereo_tracker->tracking_feature_matches.reset();

      }
    }

}
void publishLoop () {
    while(ros::ok()){

        if (matchesQueue.empty())
            continue;

        VisualFrontend::FMatches matches;
        matches = matchesQueue.front();

        //publish the matched features
        //setting the left frame timestamp
        liovil_sam::StereoFeatureMatches matches_msg;
        matches_msg.header.stamp = ros::Time(matches.frame1->tstamp); 
        matches_msg.frame_id = matches.frame1->frameid;
        matches_msg.num_matches = matches.matches.size();

        std::set<unsigned> feature_ids;
        std::set<unsigned>::iterator feat_it;
        std::pair<std::set<unsigned>::iterator,bool> ret;

        //std::cout << "Publishing features:";
        int repeat_features_count = 0;

        for(auto iter=matches.matches.begin(); iter!=matches.matches.end(); ++iter) {
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

        prev_feature_ids = feature_ids;
        matchesQueue.pop_front();
    }
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


private:


    ros::Publisher stereo_matches_pub;

    message_filters::Subscriber<Image> leftImg_sub;
    message_filters::Subscriber<Image> rightImg_sub;

    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    typedef Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    //TimeSynchronizer<Image, Image> sync2;

    std::set<unsigned> prev_feature_ids;
    std::shared_ptr<VisualFrontend::FMatches> matches;

    std::shared_ptr<VisualFrontend::StereoTracker> stereo_tracker;

    std::string leftImg_topic;
    std::string rightImg_topic;
    std::string stereoMatches_topic;


    std::deque<VisualFrontend::FMatches> matchesQueue;

};





int main(int argc, char **argv) {

  ros::init(argc,argv,"visual_frontend");
  ros::NodeHandle nh("~");

  VisualFrontend_usr VF_usr(nh);

  ros::MultiThreadedSpinner spinner(4);

  std::thread loopthread(&VisualFrontend_usr::publishLoop, &VF_usr);

  spinner.spin();
  loopthread.join();

  return 0;
}


