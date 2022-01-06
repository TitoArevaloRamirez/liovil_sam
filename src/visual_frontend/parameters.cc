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
#include <visual_frontend/parameters.h>
#include <visual_frontend/feature_operations.h>

namespace Params {
  //type of tracking
  TrackingMethod tracking = FEATURE_BASED;//OPTICALFLOW_BASED FEATURE_BASED HYBRID_BASED

  //if we use feature based method, this variable determines which feature
  //descriptor we will use
  VisualFrontend::DescriptorType descriptor_type = VisualFrontend::ORB;

  //flag which specifies whether we calculate descriptor distance both ways
  bool descriptor_distance_both_ways(false);

  //chooses between old and new hybrid
  bool old_hybrid(false);

  //chooses between custom and bruteforce matcher
  bool custom_matcher(false);
  
  //this parameter is passed during feature extraction (to the
  //goodfeaturestotrack method) the returned corners should be atleast
  //these many pixels apart
  double feature_extraction_boundary = 20.;

  //when the features are added and searched for, a minimum of these many
  //pixel distance is considered to distinguish one feature from another
  double feature_search_boundary = 10.; // 10.

  //minimum distance from the camera. This parameter sets the maximum allowed
  //disparity during stereo matching. Smaller the minimum distance from camera
  //larger the allowed disparity (this is because, closer the object, larger the
  //disparity observed). For the final application we might have 1m as the min distance
  //which will lead to very large disparities and search areas
  double min_dist_from_camera = 4.; // 4

  //temporary paramter that controls the maximum allowed disparity. This is not
  //the correct way to do this. Once the equation to set the max_disparity using
  //the mininum distance from the camera works, we should switch to using that
  double max_disparity = 1384*0.65;

  //set the percentage of image that needs to be considered as the border of the
  //image. This parameter is used for filtering during optical flow tracking
  double percent_border = 0.05;
  
  //this parameter is used in the optical flow function to set the search
  //window
  cv::Size stereo_tracking_search_window(49,49);
  cv::Size frame2frame_tracking_search_window(9,9);

  //the default number of corners extracted from an image
  unsigned num_corners = 300;

  //minimum number of corners that need to be tracked before which
  //additional corners will be added (in percentage)
  float percent_min_corners = 0.3; //70%

  //maximum number of matches to use
  unsigned max_matches = 50;
  
  //if the features tracked is lesser than this percent of num_corners,
  //then new features are extracted
  float hybrid_percent_tracked_corners = 0.10;
  
  //percent of matches that need to be included as positives from the total
  //number of features compared
  float percent_matches = 0.5;
  
  //in the event we dont have enough corners which are tracked, this number
  //states how many more corners we need to extract
  unsigned addl_corners = 0;

  //factor by which the image need to be rescaled for quicker processing
  //TODO: operating on a scale other than 1 causes the optical flow tracking to
  //crash. feature based tracking works fine though. Need to figure out why
  double rescale_factor = 1.;

  //flag which sets whether we apply histogram equalization before extracting
  //corners from the image
  bool equalize_histogram(true);

  //flag to choose or avoid corner refinement after corner extraction step
  bool refine_corners_after_corner_extraction(true);

  //flag to set/clear corner refinement operation after the optical flow step
  bool refine_corners_after_optical_flow(false);
  
  //maximum number of rows within which we should search to find the disparity
  //match. If the calibration is very good and the we track teh features perfectly
  //in the right image, this value could be 0
  double max_disparity_alignment_threshold = 5.;

  //parameter sets the distance search for when matching landmarks to lost features
  double landmark_search_radius = 10.;
  
  //flag which specifies if the tracking component should display the ui or not
  bool display_tracking_ui(false);

  //flag which specifies whether we should write the feature matches image to file
  bool write_matches_to_file(false);

  //setting this flag will display more verbose output including feature id,
  //line showing the tracked feature, etc
  bool display_verbose_output(false);


  //Parameters for hybrid based
  // divide the frame into patches to check reextraction
  int patch_num_r = 4;
  int patch_num_c = 4;
  
  // if patch number needs to reextract smaller than this, not reextract 
  int reextract_patch_num = 4;
  
  // matcher distance threshold
  double matcher_dist_threshold=50.;


  //Parameters that we generally wont change
  //intrinsics and extrinsics of the cameras
  VisualFrontend::CameraInfo left_cam_info;
  VisualFrontend::CameraInfo right_cam_info;

  //formatting characters for displaying messages in red
  std::string ERR_START("\033[1;31m");
  std::string ERR_END("\033[0m");

  std::string CLR_RESET("\033[0m");
  std::string CLR_INVERT("\033[1;7m");
  std::string CLR_GREEN("\033[1;32m");
  std::string CLR_MAGENTA("\033[1;35m");
  
};
