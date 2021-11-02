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
#ifndef __FRAME_H__
#define __FRAME_H__

#include "feature_operations.h"
#include "parameters.h"

#include <mutex>
#include <thread>
#include <sstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>

namespace VisualFrontend {  
  
  typedef enum {LEFT, RIGHT} FrameType;
  typedef enum {STEREO, FRAME2FRAME} TrackingType;

  class safe_pt2f_less {
  public:
    bool operator() (cv::Point2f left, cv::Point2f right) const {
      if (left.y == right.y)
	      return left.x < right.x;
      else
	      return left.y < right.y;
    }
  };


  class FeaturePoints {
  public:
    std::vector<cv::Point2f> pts;
    size_t kdtree_get_point_count() const;
    float kdtree_get_pt(const size_t idx, const size_t dim) const;
    template <class BBOX>
      bool kdtree_get_bbox(BBOX & /*bb*/) const;
  };
     
  typedef std::map<cv::Point2f, std::shared_ptr<Feature>, safe_pt2f_less> FeatureMapType;

  class Frame : public std::enable_shared_from_this<Frame> {

  public:
    Frame(std::shared_ptr<cv::Mat> img_sptr, double stamp, FrameType type);
    ~Frame();
    void compute_descriptors(bool reextract=false); //use this for feature based tracking
    std::shared_ptr<FMatches> track(std::shared_ptr<Frame>);
    friend std::ostream& operator<<(std::ostream &out, const Frame &);

    
    unsigned frameid;
    std::string str_frameid;
    cv::Mat img;
    cv::Mat gray; //grayscale image
    double tstamp;
    FeatureMapType features_map;	
    FrameType type;
    bool extracted_corners;
    bool tracked;
    bool stereo_processed;
    CameraInfo cam_info;
    
  protected:
    void _rescale();
    void _equalize_histogram();
    
    std::vector<cv::Point2f> _extract_corners(unsigned num_corners=Params::num_corners);
    void _refine_corners(std::vector<cv::Point2f> &corners);
    void _to_gray();
    
    std::vector<cv::Point2f> _corners();
    std::vector<cv::KeyPoint> _keypoints();
    
    void _fill_features_map(std::vector<cv::Point2f> &corners);
    void _print_features_map();
    void _mark_tracked_features(std::shared_ptr<Frame> nxt,
				std::vector<unsigned> &tracked_ids,
				FMatches &fmatches);
    void _get_all_descriptors(cv::Mat &desc);
    void _get_features_index_map(std::map<unsigned,std::shared_ptr<Feature> > &index_map);
    void _remove_untracked_features(FMatches &);
    
    void _opticalflow_tracking(std::shared_ptr<Frame> nxt,
			       FMatches &fmatches,
			       TrackingType type);

    void _bruteforce_feature_tracking(std::shared_ptr<Frame> nxt,
				      FMatches &fmatches,
				      TrackingType type);

    void _feature_tracking(std::shared_ptr<Frame> nxt,
			      FMatches &fmatches,
			      TrackingType type);

    void _hybrid_tracking(std::shared_ptr<Frame> nxt,
			      FMatches &fmatches,
			      TrackingType type);

    void _displayLR(std::shared_ptr<Frame> nxt, FMatches &matches);
  
    void _display(std::shared_ptr<Frame> nxt,
		  FMatches &matches,
		  TrackingType type);

    void _add_text_to_canvas(cv::Mat &canvas, std::string txt,unsigned line_num);
    void _write_feature_id(cv::Mat &canvas, std::shared_ptr<Feature> f);
    
    void debug__display_corners(std::vector<cv::Point2f> corners,std::string prefix);
    void debug__display_features_map(std::string prefix);
    
    static unsigned left_counter;
    static unsigned right_counter;
    double current_scale;
    bool histogram_equalized;
    double max_disparity;
  };

}; 
  
#endif //__FRAME_H__
