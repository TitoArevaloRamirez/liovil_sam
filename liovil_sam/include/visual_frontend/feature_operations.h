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
#ifndef __FEATURE_OPERATIONS_H__
#define __FEATURE_OPERATIONS_H__

#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <set>
#include <map>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace VisualFrontend {
  

  //typedefs
  typedef enum  {SIFT, SURF, ORB} DescriptorType;
  
  //timing info
  using Clock = std::chrono::high_resolution_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  double duration(TimePoint start,TimePoint finish);


  class Descriptor {
  public:
    Descriptor();
    Descriptor(cv::Mat, DescriptorType, cv::NormTypes);

    cv::Mat desc;
    double distance;
    DescriptorType type;
    cv::NormTypes norm;
    double double_dist(Descriptor &);
    double dist(Descriptor &);
  };

  //singleton class
  class FeatureDescriptor {
  public:
    static FeatureDescriptor& get_instance(); //factory method
    
    void set_descriptor(DescriptorType);
    void compute(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints,
		 cv::Mat &descriptors);
    
    DescriptorType current;
    std::string str_desc;
    cv::NormTypes norm_type;
    cv::BFMatcher bf_matcher;
    
  protected:
    FeatureDescriptor();
    cv::Ptr<cv::xfeatures2d::SURF> surf;
    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::xfeatures2d::SIFT> sift;
  };


  class Feature : public std::enable_shared_from_this<Feature> {
  public:
    Feature();
    Feature(const Feature &);
    Feature(cv::Point2f loc);
    Feature& operator=(const Feature&);

    void get_new_id();
    static Feature make_feature(cv::Point2f loc);

    friend std::ostream& operator<<(std::ostream &out, const Feature &);

    //variables - make sure you copy them all in constructors
    cv::Point2f lpix;
    double disparity;
    double depth;
    bool tracked;
    unsigned id;
    Descriptor descriptor;
    
    static unsigned int id_counter;	
  };


  //class definitions
  class Frame;
  class Landmark;


  class FMatch {
  public:
    FMatch();
    FMatch(const FMatch &);
    FMatch& operator=(const FMatch &fmatch);
    
    std::shared_ptr<Feature> f1;
    std::shared_ptr<Feature> f2;
    double desc_dist;
    bool operator<(const FMatch&) const;
  };

  class FMatches {
  public:
    void add_match(std::shared_ptr<Feature> ,std::shared_ptr<Feature> ,double desc_dist) ;
    void clear();

    void _add_entry(std::shared_ptr<FMatch>);
    void _remove_entry(std::shared_ptr<FMatch>);
    void _replace_feature(std::shared_ptr<Feature> f_old, std::shared_ptr<Feature> f_new);
    //std::shared_ptr<FMatch> _check_old_candidates(std::shared_ptr<Feature> f);
    void _print_feature_ids();
    void _print_matches();
    void _print_matches_map();
    void _print_old_candidates();
    void _check_datastructure_integrity(std::string);
    
    std::vector<std::shared_ptr<FMatch> > matches;
    std::shared_ptr<Frame> frame1;
    std::shared_ptr<Frame> frame2;
    //stores the feature ids as a set to check if we repeat them
    std::set<unsigned> feature_ids;

    //a map which stores the index of the matches for convenience
    //map<feature_id,match_index>
    std::map<unsigned,unsigned> matches_map;

    //when a feature is replaced from a fmatch object, the fmatch is added to this
    //map. Later if the other feature also gets replaced, then we can rematch those
    //two features back
    //map<feature_id,FMatch>
    std::map<unsigned,std::shared_ptr<FMatch> > old_candidates;
    
  };
  

  typedef std::vector<double> Point;

  struct DepthFeature {
  public:
    unsigned fid;
    Point wo_d_k_1;
    Point w_d_k_1;
    Point wo_d_k;
  };

  typedef std::vector<DepthFeature> DepthFeatures;

  struct NoDepthFeature {
  public:
    unsigned fid;
    Point wo_d_k_1;
    Point wo_d_k;
  };

  typedef std::vector<NoDepthFeature> NoDepthFeatures;
  
  class CameraInfo {
  public:
    double fx;
    double fy;
    double cx;
    double cy;
    double baseline;
  };
  

  //functions
  // void _normalize_pixel_coordinates(FeatureMatch &, CameraInfo c1, CameraInfo c2);
    
};

#endif //__FEATURE_OPERATIONS_H__
