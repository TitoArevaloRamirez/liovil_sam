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
#include <visual_frontend/landmark.h>
#include <visual_frontend/parameters.h>

namespace VisualFrontend {

  unsigned Landmark::landmark_id_counter = 0;

  Landmark::Landmark(): landmarkid(landmark_id_counter++) {}


  
  LandmarkManager& LandmarkManager::get_instance() {
    static auto instance = LandmarkManager();  
    return instance;
  }

  void LandmarkManager::update_landmarks(std::shared_ptr<Frame> frm_ptr) {

    //iterate through the features map
    for(auto iter = frm_ptr->features_map.begin(); iter != frm_ptr->features_map.end();
	++iter) {

      unsigned feature_id = iter->second->id;
      unsigned landmark_id;
      
      if(feature_id_rev_ref.find(feature_id) != feature_id_rev_ref.end()) {
	landmark_id = feature_id_rev_ref[feature_id];

	if(landmarks.find(landmark_id) != landmarks.end()) {
	}
      }
    }
	  
    //add a landmark if it does not already exist
    //if a landmark is more than 3 frames old, delete it
    
  }

  void LandmarkManager::add_observation(std::shared_ptr<Feature> feature,unsigned frameid) {

    //check if the landmark exists for the feature,
    //if not, check if we can find one closer to the feature location created not a
    //lot of frames ago
    
    if(!get_landmark_using_feature(feature->id)) {
      std::shared_ptr<Landmark> landmark = std::make_shared<Landmark>();
      landmark->observations[feature->id] = feature;
      landmark->frames_observed[frameid] = feature->id;
      landmark->latest_observ = feature->lpix;

      //update booking in the manager
      landmarks[landmark->landmarkid] = landmark;
      feature_id_rev_ref[feature->id] = landmark->landmarkid;
      pt_rev_ref[feature->lpix] = landmark->landmarkid;
      
    }
  }
  
  std::shared_ptr<Landmark> LandmarkManager::get_landmark(unsigned lid) {
    std::shared_ptr<Landmark> landmark;

    auto search = landmarks.find(lid);
    if(search != landmarks.end()) {
      return search->second;
    } else {
      return landmark;
    }
  }
  
  std::shared_ptr<Landmark> LandmarkManager::get_landmark_using_feature(unsigned fid) {
    std::shared_ptr<Landmark> landmark;
    
    auto search = feature_id_rev_ref.find(fid);
    if(search != feature_id_rev_ref.end()) {
      
      auto search2 = landmarks.find(search->second);
      if(search2 != landmarks.end()) {

	return search2->second;
      }
    }
    return landmark; 
  }
  
  std::shared_ptr<Landmark> LandmarkManager::get_landmark_close_to(cv::Point2f pt) {

    std::shared_ptr<Landmark> landmark;

    //iterate through the map and find a point closer to the given pt
    for(auto iter = pt_rev_ref.begin(); iter != pt_rev_ref.end(); ++iter) {
      cv::Point2f diff(fabs(pt.x - iter->first.x), fabs(pt.y-iter->first.y));

      if(diff.x < Params::landmark_search_radius && diff.y < Params::landmark_search_radius) {
	unsigned landmarkid = iter->second;

	return landmarks[landmarkid];
      }
    }
    
    return landmark;
  }
  
  
  LandmarkManager::LandmarkManager() {}
  
};
