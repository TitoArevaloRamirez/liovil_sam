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
#ifndef __LANDMARK_H__
#define __LANDMARK_H__

#include <iostream>
#include <vector>
#include <map>
#include <set>

#include <opencv2/core/core.hpp>

#include "frame.h"
#include "feature_operations.h"

namespace VisualFrontend {  
    
  class Landmark : public std::enable_shared_from_this<Landmark> {
  public:
    Landmark();
    //featureid,feature
    std::map<unsigned, std::shared_ptr<Feature> > observations;
    //frameid,featureid
    std::map<unsigned, unsigned> frames_observed;
    cv::Point2f latest_observ;
    cv::Point3f pt;
    unsigned landmarkid;

    static unsigned landmark_id_counter;
    
  };

  class LandmarkManager {
  public:
    static LandmarkManager& get_instance(); //factory method
    void update_landmarks(std::shared_ptr<Frame>);

    void add_observation(std::shared_ptr<Feature>,unsigned frameid);
    
    std::shared_ptr<Landmark> get_landmark(unsigned lid);
    std::shared_ptr<Landmark> get_landmark_using_feature(unsigned fid);
    std::shared_ptr<Landmark> get_landmark_close_to(cv::Point2f pt);
    
    //landmarkid,landmark
    std::map<unsigned,std::shared_ptr<Landmark> > landmarks;
    //reference the landmark using the feature id
    //featureid, landmarkid
    std::map<unsigned,unsigned> feature_id_rev_ref;
    //reference the landmark using the latest observation
    //latest_observation, landmarkid
    std::map<cv::Point2f, unsigned, safe_pt2f_less> pt_rev_ref;
  protected:
    LandmarkManager();
    
  };

};




    
#endif //__LANDMARK_H__
