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
#ifndef __STEREO_TRACKER_H__
#define __STEREO_TRACKER_H__

#include "feature_operations.h"
#include "frame.h"
#include "stereo_pair.h"
#include "landmark.h"
#include "hybrid.cc"

#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <mutex>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace VisualFrontend {
  
  class StereoTracker {
  public:
    StereoTracker();
    void add_left_img(std::shared_ptr<cv::Mat> left, double time);
    void add_right_img(std::shared_ptr<cv::Mat> right, double time);

    void track();

    void get_features_for_motion_estimation(DepthFeatures &depth_features,
					    NoDepthFeatures &nodepth_features);
    
    bool visualize;
    bool write_to_file;

    std::shared_ptr<FMatches> stereo_feature_matches;
    std::shared_ptr<FMatches> tracking_feature_matches;

    
  protected:
    void fill_current_pair();
    
    //current frames operated on
    std::shared_ptr<Frame> just_arrived_left;
    std::shared_ptr<Frame> just_arrived_right;
    std::shared_ptr<StereoPair> curr_pair;
    std::shared_ptr<StereoPair> prev_pair;
    std::shared_ptr<StereoPair> next_pair;
    
    //FeatureDescriptor feature_descriptor;    
    
    double calibration_tolerance;
    std::mutex right_mutex_;
    std::mutex left_mutex_;

    FrontEnd hybrid_based_frontend;
  };


};

#endif //__STEREO_TRACKER_H__
