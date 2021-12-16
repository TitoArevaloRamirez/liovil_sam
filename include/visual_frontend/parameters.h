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
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include <iostream>
#include <opencv2/core/core.hpp>

#include "feature_operations.h"

namespace Params {
  
  typedef enum {FEATURE_BASED, OPTICALFLOW_BASED, HYBRID_BASED } TrackingMethod;

  extern TrackingMethod tracking;

  extern VisualFrontend::DescriptorType descriptor_type;

  extern bool descriptor_distance_both_ways;

  extern bool old_hybrid;

  extern bool custom_matcher;
  
  extern double feature_extraction_boundary;

  extern double feature_search_boundary;

  extern double min_dist_from_camera;

  extern double max_disparity;

  extern double percent_border;
  
  extern cv::Size stereo_tracking_search_window;
  extern cv::Size frame2frame_tracking_search_window;
  
  extern unsigned num_corners;

  extern float percent_min_corners;

  extern unsigned max_matches;
  
  extern float hybrid_percent_tracked_corners;
  
  extern float percent_matches;
  
  extern unsigned addl_corners;

  extern double rescale_factor;

  extern bool equalize_histogram;

  extern bool refine_corners_after_corner_extraction;

  extern bool refine_corners_after_optical_flow;
  
  extern double max_disparity_alignment_threshold;

  extern double landmark_search_radius;
  
  extern bool display_tracking_ui;

  extern bool write_matches_to_file;

  extern bool display_verbose_output;
  
  // parameters for hybrid based
  extern int patch_num_r;

  extern int patch_num_c;

  extern int reextract_patch_num;

  extern double matcher_dist_threshold;

  //Parameters that we generally wont change
  
  extern VisualFrontend::CameraInfo left_cam_info;
  extern VisualFrontend::CameraInfo right_cam_info;

  extern std::string ERR_START;
  extern std::string ERR_END;

  extern std::string CLR_RESET;
  extern std::string CLR_INVERT;
  extern std::string CLR_GREEN;
  extern std::string CLR_MAGENTA;

  //Topics
  extern std::string leftImg_topic;
  extern std::string rightImg_topic;
  extern std::string stereoMatches_topic;

};

#endif //__PARAMETERS_H__
