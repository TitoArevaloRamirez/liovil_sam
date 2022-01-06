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
#include <visual_frontend/feature_operations.h>
#include <visual_frontend/frame.h>
#include <visual_frontend/landmark.h>

#include <chrono>
#include <algorithm>
#include <sstream>
#include <limits>
#include <map>
#include <assert.h>
#include <iomanip>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/ocl.hpp>

namespace VisualFrontend {

  static bool debug(false);
  
  //////////////////////////////////////////////////////////////////////////////////////////

  double duration(TimePoint start, TimePoint finish) {
    
    return std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count(); 
  }

  //////////////////////////////////////////////////////////////////////////////////////////

  Descriptor::Descriptor():
    type(Params::descriptor_type),
    norm((Params::descriptor_type==ORB?cv::NORM_HAMMING:cv::NORM_L2)) {
}

  Descriptor::Descriptor(cv::Mat d, DescriptorType t, cv::NormTypes n):
    desc(d), type(t), norm(n) {
  }
  
  double Descriptor::double_dist(Descriptor &nxt) {
    if(nxt.type != type) {
      //if the descriptor types are different, set a very large distance to show that the
      //descriptor doesn't match. In future if we want to interoperate between descriptors
      //we can add additional logic.
      distance = std::numeric_limits<double>::infinity();
      
    } else {
      
      double d1 = cv::norm(desc, nxt.desc, norm);
      double d2 = cv::norm(nxt.desc,desc, norm);
      distance = d1*d1+d2*d2;
    }
    return distance;
  }

  double Descriptor::dist(Descriptor &nxt) {
    if(nxt.type != type) {
      //if the descriptor types are different, set a very large distance to show that the
      //descriptor doesn't match. In future if we want to interoperate between descriptors
      //we can add additional logic.
      distance = std::numeric_limits<double>::infinity();

    } else {

      distance = cv::norm(desc, nxt.desc, norm);
    }

    return distance;
  }
    

  //////////////////////////////////////////////////////////////////////////////////////////

  FeatureDescriptor& FeatureDescriptor::get_instance() {
    static auto instance = FeatureDescriptor();  
    return instance;
  }
    
  FeatureDescriptor::FeatureDescriptor():
    surf(cv::xfeatures2d::SURF::create()),
    orb(cv::ORB::create()),
    sift(cv::xfeatures2d::SIFT::create()),
    norm_type(cv::NORM_L2) {
    set_descriptor(Params::descriptor_type);
  }


  void FeatureDescriptor::set_descriptor(DescriptorType desc) {
    if(desc == SURF) {
      current = desc;
      norm_type = cv::NORM_L2;
      str_desc = "SURF";
      std::cout << "Switching to SURF descriptor" << std::endl;
    } else if(desc == ORB) {
      current = desc;
      norm_type = cv::NORM_HAMMING;
      str_desc = "ORB";
      std::cout << "Switching to ORB descriptor" << std::endl;
    } else if(desc == SIFT) {
      current = desc;
      norm_type = cv::NORM_L2;
      str_desc = "SIFT";
      std::cout << "Switching to SIFT descriptor" << std::endl;
    }
    bf_matcher = cv::BFMatcher(norm_type, false);

  }

  void FeatureDescriptor::compute(cv::Mat &img,
				  std::vector<cv::KeyPoint> &keypoints,
				  cv::Mat &descriptors) {
    if(current != Params::descriptor_type) {
      set_descriptor(Params::descriptor_type);
    }
    
    if(current == SURF) {
      surf->compute(img,keypoints,descriptors);
    } else if(current == ORB) {
      orb->compute(img,keypoints,descriptors);
    } else if(current == SIFT) {
      sift->compute(img,keypoints,descriptors);
    }
  }

  
  //////////////////////////////////////////////////////////////////////////////////////////
  
  unsigned int Feature::id_counter = 0;

  Feature::Feature() :
    disparity(std::nan("")),
    depth(std::nan("")),
    tracked(false),
    id(id_counter++) {}

  Feature::Feature(const Feature &f):
    lpix(f.lpix),
    disparity(f.disparity),
    depth(f.depth),
    tracked(f.tracked),
    id(f.id),
    descriptor(f.descriptor) {}

  Feature::Feature(cv::Point2f loc):
    lpix(loc),
    disparity(std::nan("")),
    depth(std::nan("")),
    tracked(false),
    id(id_counter++) {}

  Feature& Feature::operator=(const Feature& f) {
    lpix = f.lpix;
    disparity = f.disparity;
    depth = f.depth;
    tracked = f.tracked;
    id = f.id;
    descriptor = f.descriptor;
    return *this;
  }

  void Feature::get_new_id() {
    id = id_counter++;
  }

  Feature Feature::make_feature(cv::Point2f loc) {
    Feature f(loc);
    return f;
  }

  std::ostream& operator<<(std::ostream &out, const Feature &f) {
    
    out << Params::CLR_MAGENTA << " [" << std::setw(7) <<  f.id << "] " <<
      Params::CLR_RESET << "(" << std::setw(7) << f.lpix.x << "," <<
      std::setw(7) << f.lpix.y << ")";
    return out;
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////

  FMatch::FMatch():  desc_dist(std::numeric_limits<double>::infinity()) {}

  FMatch::FMatch(const FMatch &fmatch):
    f1(fmatch.f1),
    f2(fmatch.f2),
    desc_dist(fmatch.desc_dist) {
  }

  FMatch& FMatch::operator=(const FMatch &fmatch) {
    f1 = fmatch.f1;
    f2 = fmatch.f2;
    desc_dist = fmatch.desc_dist;
    return *this;
  }
  
  
  bool FMatch::operator<(const FMatch &b) const {
    return desc_dist < b.desc_dist;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  
  void FMatches::add_match(std::shared_ptr<Feature> f1,
			   std::shared_ptr<Feature> f2,
			   double desc_dist) {
    
    //try inserting feature id to the feature_ids set.
    //if they are successful, then good, add it to the matches object
    //if not, check which one failed and check for the feature in matches map
    //pull the descriptor distance and replace the feature with the smallest distance

    if(debug) std::cout << "**add_match:" << *f1 << *f2 << std::endl;
    bool f1_exists(feature_ids.count(f1->id));
    bool f2_exists(feature_ids.count(f2->id));

    if(debug) std::cout << "f1_exists:" << feature_ids.count(f1->id) << std::endl;
    if(debug) std::cout << "f2_exists:" << feature_ids.count(f2->id) << std::endl;

    if(f1_exists && f2_exists) {
      //make sure the same pair doesn't exist already
      auto f1_match = matches[matches_map[f1->id]];
      auto f2_match = matches[matches_map[f2->id]]; 

      if((f1_match->f1->id == f1->id && f1_match->f2->id == f2->id) ||
	 (f2_match->f1->id == f1->id && f2_match->f2->id == f2->id)) {
	//std::cout << "Matches " << *f1 << *f2 << " already exists" << std::endl;
	return;
      }
      //if both exists, remove the entry for f2, try the current pair
      //and then recurse to the removed pair
      //auto f2_match = matches[matches_map[f2->id]];

      _remove_entry(f2_match);
      if(debug) std::cout <<  Params::CLR_MAGENTA << Params::CLR_INVERT <<
		  "Recurse call for "  << Params::CLR_RESET << *f1 << *f2 << std::endl; 
      add_match(f1, f2, desc_dist);

      //now try adding it back
      if(debug) std::cout <<  Params::CLR_MAGENTA << Params::CLR_INVERT <<
		  "Recurse call for "  << Params::CLR_RESET << *(f2_match->f1) <<
		  *(f2_match->f2) << std::endl; 
      
      add_match(f2_match->f1, f2_match->f2, f2_match->desc_dist);

      //when we are done we would have already parsed the current match
      //and tested with the existing pairs. so we should be done
      return;
    }
    
    if(f1_exists || f2_exists) {      
      //this feature is already matched to some other feature

      //get the records 
      std::vector<std::shared_ptr<FMatch> > old_matches;
      std::vector<unsigned> curr_feature_type;

      if(f1_exists) {
	_print_feature_ids();
	_print_matches_map();
	_print_matches();
        
	std::shared_ptr<FMatch> m = matches[matches_map[f1->id]];
	if(debug) std::cout << "Pulled out:" << *(m->f1) << *(m->f2) << std::endl;
        
	old_matches.push_back(std::make_shared<FMatch>(*matches[matches_map[f1->id]]));
	curr_feature_type.push_back(1);
      } 
      else if(f2_exists) {
	old_matches.push_back(std::make_shared<FMatch>(*matches[matches_map[f2->id]]));
	curr_feature_type.push_back(2);
      }
            
      auto feature_type_iter = curr_feature_type.begin();
      for(auto match_iter = old_matches.begin();
	  match_iter != old_matches.end() && feature_type_iter != curr_feature_type.end();
	  ++match_iter, ++feature_type_iter) {
        
	std::shared_ptr<FMatch> old_match = *match_iter;
	unsigned feature_type = *feature_type_iter;

	// if(feature_type == 1) {
	//   //make a copy
	//   old_match = std::make_shared<FMatch>(*matches[matches_map[f1->id]]);
          
	// } else if(feature_type == 2) {
	//   old_match = std::make_shared<FMatch>(*matches[matches_map[f2->id]]);
	// }
        
	std::shared_ptr<Feature> stranded_feature;

	std::shared_ptr<Feature> old_f1 = old_match->f1;
	std::shared_ptr<Feature> old_f2 = old_match->f2;
        
	if(debug) std::cout << "Matching:" << *f1 << *f2 << std::endl;

	//if the new feature matches better, then use it
	if(desc_dist < old_match->desc_dist) {

	  if(feature_type == 1) {
	    if(debug)  std::cout << "Replacing" <<
			 *(old_match->f2) <<
			 " with " << *f2 <<
			 " Desc: old:" << old_match->desc_dist << " new:" <<
			 desc_dist << std::endl;

	    stranded_feature = old_match->f2;
	    _replace_feature(old_match->f2,f2);
            
	  } else if(feature_type == 2) {
	    if(debug)  std::cout << "Replacing" <<
			 *(old_match->f1) <<
			 " with " << *f1 <<
			 " Desc: old:" << old_match->desc_dist << " new:" <<
			 desc_dist << std::endl;

	    stranded_feature = old_match->f1;
	    _replace_feature(old_match->f1,f1);
	  }
	} else {
	  if(debug) std::cout << "The previous match is pretty good. Lets leave it alone."
			      << (feature_type==1?*f1:*f2) << " and "
			      << (feature_type==1?*(old_match->f2):*(old_match->f1)) << std::endl;
          
	  stranded_feature = (feature_type==1 ? f2:f1);
	}

	_print_old_candidates();
        
	//check if the stranded feature has any old candidate pairs
	auto prev_cand = old_candidates.find(stranded_feature->id);
	if(prev_cand != old_candidates.end()) {
	  //make sure we are not trying to match the stranded feature
	  //back to original pair
	  //if f1_exists, then we are choosing f2 which is the stranded feature
	  //so check if f1 and the prev_candidate f1 are the same in which
	  //case we are comparing to the current pair
	  if((feature_type == 1 && prev_cand->second->f1->id != f1->id) ||
	     (feature_type == 2 && prev_cand->second->f2->id != f2->id)) {
	    //try adding it back
	    if(debug) std::cout << Params::CLR_MAGENTA << Params::CLR_INVERT <<
			"Recursive call for " << Params::CLR_RESET << *(prev_cand->second->f1) <<
			*(prev_cand->second->f2) << std::endl;
	    //remove the entry from the map
	    old_candidates.erase(stranded_feature->id);

	    //and then try to add the match back in
	    add_match(prev_cand->second->f1, prev_cand->second->f2,
		      prev_cand->second->desc_dist);
	  }
	}
      }
    }
      
    else {
      //add the entry in
      std::shared_ptr<FMatch> m = std::make_shared<FMatch>();
      m->f1 = f1;
      m->f2 = f2;
      m->desc_dist = desc_dist;
      
      _add_entry(m);
    }
  }
  

  void FMatches::clear() {
    //clear all the data structures
    matches.clear();
    frame1.reset();
    frame2.reset();
    feature_ids.clear();
    matches_map.clear();
  }

  void FMatches::_add_entry(std::shared_ptr<FMatch> match) {

    _check_datastructure_integrity("add_entry");
    
    //add entries to feature_ids
    feature_ids.insert(match->f1->id);
    feature_ids.insert(match->f2->id);

    //update the matches vector
    if(debug) std::cout << "Adding:" << *(match->f1) << " and " << *(match->f2) << std::endl;
    matches.push_back(match);

    //add corresponding entries to the matches_map 
    matches_map[match->f1->id] = matches.size()-1;
    matches_map[match->f2->id] = matches.size()-1;

    //check if old candidates have any entries and remove them
    auto old_f1 = old_candidates.find(match->f1->id);
    if(old_f1 != old_candidates.end()) {
      old_candidates.erase(match->f1->id);
    }
    auto old_f2 = old_candidates.find(match->f2->id);
    if(old_f2 != old_candidates.end()) {
      old_candidates.erase(match->f2->id);
    }
    
  }

  void FMatches::_remove_entry(std::shared_ptr<FMatch> match) {

    if(debug) std::cout << "Removing entry " << *(match->f1) << *(match->f2) << std::endl;

    _check_datastructure_integrity("remove_entry");
    
    //erase entries from feature_ids
    feature_ids.erase(match->f1->id);
    feature_ids.erase(match->f2->id);
    
    //find the entry in matches map and erase it
    if(matches_map[match->f1->id] != matches_map[match->f2->id]) {
      if(debug) std::cout << "Something went wrong. The matches_map should point to "
  		<< "the same index to the matches datastructure. But that "
  		<< "doesn't seem to be the case here" << std::endl;
      exit(-1);
    }

    //erase the entries from the matches vector
    matches.erase(matches.begin()+matches_map[match->f1->id]);

    unsigned erased_index = matches_map[match->f1->id];
    //now erase both entries from matches_map
    matches_map.erase(match->f1->id);
    matches_map.erase(match->f2->id);
    
    //iterate through the matches_map and decrease the index of all entries greater
    //than the entry deleted from the list
    for(auto iter = matches_map.begin(); iter != matches_map.end(); ++iter) {
      if(iter->second > erased_index) {
	iter->second--;
      }
    }
    
    //now add an entry to old candidates
    old_candidates[match->f1->id] = match;
    old_candidates[match->f2->id] = match;
  }
  
  void FMatches::_replace_feature(std::shared_ptr<Feature> f_old,
				  std::shared_ptr<Feature> f_new) {

    _check_datastructure_integrity("replace_feature");
    
    //remove f_old from features list and add f_new to features list
    feature_ids.erase(f_old->id);
    auto res = feature_ids.insert(f_new->id);
    if(!res.second) {
      std::cout << "The new feature id already exists in the feature list. "
		<< "Something wrong!!! " << std::endl;
      exit(-1);
    }

    //update the matches vector    
    std::shared_ptr<FMatch> fmatch = matches[matches_map[f_old->id]];

    //before replacing matches, add an entry to old_candidates
    old_candidates[f_old->id] = std::make_shared<FMatch>(*fmatch);
    
    //first check whether the replaced feature is f1 or f2
    if(f_old->id == fmatch->f1->id)
      fmatch->f1 = f_new;
    else if(f_old->id == fmatch->f2->id)
      fmatch->f2 = f_new;
    else {
      std::cout << *f_new << *f_old << *fmatch->f1 << *fmatch->f2 << std::endl;
      std::cout << "Matches map points to an index which does not have the "
		            << "same feature id of the feature being replaced. Something "
		            << "bad happened." << std::endl;

      
      exit(-1);
    }

    //update the matches_map
    //remove the entry for f_old and add an entry for f_new
    matches_map[f_new->id] = matches_map[f_old->id];
    matches_map.erase(f_old->id);

    //if there are any existing old_candidates for f_new, remove them
    auto old_entry = old_candidates.find(f_new->id);
    if(old_entry != old_candidates.end())
      old_candidates.erase(old_entry);
  }

  // std::shared_ptr<FMatch> FMatches::_check_old_candidates(std::shared_ptr<Feature> f) {

  //   //check if the feature is in the old candidates list, if not, add it in as it
  //   //has been considered for a match
  //   auto old_entry = old_candidates.find(f->id);
  //   if(old_entry != old_candidates.end()) {
  //     //remove the entry from the map
  //     old_candidates.erase(f->id);

  //     return old_entry->second;
      
  //   } else {
  //     //add the feature being matched to to the old_matches. If this gets
  //     //removed in a later step we can rematch it to its previous match
  //     old_candidates[f->id] = matches[matches_map[f->id]];
  //     std::shared_ptr<FMatch> empty;
  //     return empty;
  //   }
    
  // }

  void FMatches::_print_feature_ids() {
    if(debug) {
      std::cout << "Feature ids:";
      for(auto iter = feature_ids.begin(); iter != feature_ids.end();
	  ++iter) {
	std::cout << *iter << ",";
      }
      std::cout << std::endl;
    }
  }
  
  void FMatches::_print_matches() {
    if(debug) {
      std::cout << "Matches:";
      for(auto iter = matches.begin(); iter != matches.end(); ++iter)
	      std::cout << "(" << (*iter)->f1->id << "," << (*iter)->f2->id << ") ";
      std::cout << std::endl;
    }
  }
  
  void FMatches::_print_matches_map() {
    if(debug) {
      std::cout << "Matches_map:";
      for(auto iter = matches_map.begin(); iter != matches_map.end();
	  ++iter) {
	std::cout << "[" << iter->first << "->" << iter->second << "]";
	//  std::cout << "(" << matches[iter->second]->f1->id << ","
	// 		<< matches[iter->second]->f2->id << ") ";
      }
      std::cout << std::endl;
    }
  }
  
  void FMatches::_print_old_candidates() {
    if(debug) {
      std::cout << "Old candidates:";
      for(auto iter = old_candidates.begin(); iter != old_candidates.end();
	  ++iter) {
	std::cout << iter->first << ",";
      }
      std::cout << std::endl;
    }
  }

  void FMatches::_check_datastructure_integrity(std::string text) {

    //enabled only for debug
    if(!debug) {
      return;
    }
    
    if(debug) std::cout << Params::CLR_INVERT << "Integrity checking: " << text <<
		Params::CLR_RESET << std::endl;
    
    bool flag(false);
    
    for(auto iter = matches_map.begin(); iter != matches_map.end(); ++iter) {

      //make sure matches vector is intact
      std::shared_ptr<FMatch> fmatch = matches[iter->second];
      if(fmatch->f1->id != iter->first && fmatch->f2->id != iter->first) {
	std::cout << Params::ERR_START << "Corrupt entry in matches_map. Fid:" <<
	  iter->first << " FMatch f1id:" << fmatch->f1->id << " f2id:" <<
	  fmatch->f2->id << Params::ERR_END << std::endl;
	flag = true;
      }

      //next feature_id set
      if(feature_ids.count(iter->first) == 0) {
	std::cout << Params::ERR_START << "Entry " << iter->first <<
	  " exists in the matches_map " <<
	  " but not in the feature_ids set!!!" << Params::ERR_END << std::endl;
	flag = true;
      }

      //check integrity of matches_map itself
      if(matches_map[fmatch->f1->id] != matches_map[fmatch->f2->id]) {
	std::cout << Params::ERR_START << "Features " << fmatch->f1->id << " and " <<
	  fmatch->f2->id << " point to different entries in the matches_map, although " <<
	  " they are matched entries" << Params::ERR_END << std::endl;
      }
    }

    //check the old_candidates
    for(auto iter = old_candidates.begin(); iter != old_candidates.end(); ++iter) {

      std::shared_ptr<FMatch> fmatch = iter->second;
      if(fmatch->f1->id != iter->first && fmatch->f2->id != iter->first) {
	std::cout << Params::ERR_START << "Corrupt entry in old_candidates. Fid:" <<
	  iter->first << " FMatch f1id:" << fmatch->f1->id << " f2id:" <<
	  fmatch->f2->id << Params::ERR_END << std::endl;
	flag = true;
      }
    }

    if(flag) {
      std::cout << "Integrity compromized. Exiting" << std::endl;
      exit(-1);
    }
  }
  
  

  //////////////////////////////////////////////////////////////////////////////////////////


}; //end of namespace
 

