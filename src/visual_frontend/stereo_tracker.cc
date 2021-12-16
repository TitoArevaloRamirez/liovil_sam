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
#include <visual_frontend/stereo_tracker.h>
#include <visual_frontend/feature_operations.h>
#include <visual_frontend/parameters.h>

#include <algorithm>
#include <sstream>
#include <map>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>



namespace VisualFrontend {
    
  StereoTracker::StereoTracker():
    //number of pixels we can assume error when searching for correspondence
    //in the right image. Make the number bigger for bad calibrations
    calibration_tolerance(5.),
    //variables for the display with matches
    visualize(false){
     //feature_descriptor.set_descriptor(ORB);
   }

  void StereoTracker::add_left_img(std::shared_ptr<cv::Mat> left, double time) {

    std::unique_lock<std::mutex> lock(left_mutex_);
    just_arrived_left = std::make_shared<Frame>(left,time,LEFT);
  }
  
  void StereoTracker::add_right_img(std::shared_ptr<cv::Mat> right, double time) {
    
    std::unique_lock<std::mutex> lock(right_mutex_);
    just_arrived_right = std::make_shared<Frame>(right,time,RIGHT);
  }
 
  void StereoTracker::track() {

    fill_current_pair();
    //wait until we have something in curr_pair
    if(!curr_pair)
      return;

    // hybrid based
    if (Params::tracking == Params::HYBRID_BASED) {

      if(Params::old_hybrid) {

        //stereo_feature_matches = hybrid_based_frontend.frame_callback(curr_pair);
      
        //curr_pair->get_left()->tracked = true;
        //curr_pair->get_right()->stereo_processed = true;

        //std::cout<< Params::CLR_GREEN <<"curr_pair \n \t frame_ids: " << curr_pair->get_ids() 
        //<< "\n \t tracked (left): " << curr_pair->get_left()->tracked
        //<< "\n \t stereo_processed (right): " << curr_pair->get_right()->stereo_processed << Params::CLR_RESET<< std::endl;
      
        //prev_pair = curr_pair;
        //curr_pair.reset();
      } 
      else {
        // frame2frame tracking
        if (prev_pair && curr_pair) {
          tracking_feature_matches = prev_pair->get_left()->track(curr_pair->get_left());
          tracking_feature_matches = prev_pair->get_right()->track(curr_pair->get_right());
        }
        // stereo matching
        if(curr_pair)
          stereo_feature_matches = curr_pair->get_left()->track(curr_pair->get_right());   
         //

        // reest
        if (!prev_pair || 
            (prev_pair->get_right()->tracked && prev_pair->get_right()->tracked) 
          ) {
          prev_pair = curr_pair;
          curr_pair.reset();
        }
      }
    }
    // feature_based or optical based
    else {
      //if the left frames exists in both the previous and the current frames,
      //do frame2frame tracking
      if(prev_pair && curr_pair)
        tracking_feature_matches = prev_pair->get_left()->track(curr_pair->get_left());
      //if the current frame exists then do stereo tracking
      if(curr_pair)
        stereo_feature_matches = curr_pair->get_left()->track(curr_pair->get_right());
      //if we have used the previous pair for frame2frame tracking, we can get rid of it
      if(!prev_pair || prev_pair->get_left()->tracked) {
        prev_pair.reset();
        prev_pair = curr_pair;
        curr_pair.reset();
      }
    }
  }

  void StereoTracker::get_features_for_motion_estimation(DepthFeatures &depth_features,
							 NoDepthFeatures &nodepth_features) {}
  
  void StereoTracker::fill_current_pair() {

      if(!next_pair)
	      next_pair = std::make_shared<StereoPair>();

      if(just_arrived_left) {
        //acquire lock and copy the object
        std::unique_lock<std::mutex> lock(left_mutex_);
        next_pair->add_left(just_arrived_left);

        just_arrived_left.reset();
      }

      if(just_arrived_right) {
        //acquire lock and copy the object
        std::unique_lock<std::mutex> lock(right_mutex_);
        next_pair->add_right(just_arrived_right);
        just_arrived_right.reset();
      }

      if (next_pair->has_left() && next_pair->has_right()) {
        if (next_pair->get_left()->frameid == next_pair->get_right()->frameid) {
	        // fill curr_pair
          curr_pair = next_pair;
	        next_pair.reset();
        }
        // bad fix?
        //else if (next_pair->get_left()->frameid > next_pair->get_right()->frameid) {
	      else {
	        //std::cout << Params::CLR_MAGENTA << Params::CLR_INVERT << "Triggered the else case " << Params::CLR_RESET << std::endl;
          next_pair->get_left()->frameid = next_pair->get_right()->frameid; 
          curr_pair = next_pair;
          next_pair.reset();
        }
      } 
  }
  
}; //end of namespace
 
