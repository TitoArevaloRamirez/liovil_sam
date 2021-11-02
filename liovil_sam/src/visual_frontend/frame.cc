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
#include <visual_frontend/frame.h>
#include <visual_frontend/landmark.h>
#include <visual_frontend/parameters.h>
#include <nanoflann/nanoflann.hpp>

#include <iomanip>
#include <numeric>
#include <limits>
#include <set>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/ocl.hpp>


namespace VisualFrontend {

  bool debug(false);

  unsigned Frame::left_counter = 1;
  unsigned Frame::right_counter = 1;
    
  size_t FeaturePoints::kdtree_get_point_count() const {
    return pts.size();
  }

  float FeaturePoints::kdtree_get_pt(const size_t idx, const size_t dim) const {
    if(dim == 0) {
      return pts[idx].x;
    } else if(dim == 1) {
      return pts[idx].y;
    }
  }

  template <class BBOX>
  bool FeaturePoints::kdtree_get_bbox(BBOX &bb) const {
    return false;
  }
  
  Frame::Frame(std::shared_ptr<cv::Mat> img_sptr, double stamp, FrameType ftype):
    img(img_sptr->clone()),
    tstamp(stamp),
    type(ftype),
    extracted_corners(false),
    tracked(false),
    stereo_processed(false),
    current_scale(1.),
    histogram_equalized(false),
    max_disparity(0) {
    
    std::stringstream ss;
    
    if(type == LEFT) {
      frameid = left_counter++;
      ss << std::setw(5) << "[L" << frameid << "]";
      str_frameid = ss.str();
    } else {
      frameid = right_counter++;
      ss << std::setw(5) << "[R" << frameid << "]";
      str_frameid = ss.str();
    }
  }

  Frame::~Frame() {

    //iterate through the features_map datastructure and clear all the
    //descriptors as they are no longer valid
    for(auto iter=features_map.begin(); iter != features_map.end(); ++iter) {
      iter->second->descriptor.desc.release();
    }
    
    if((type == LEFT && !tracked) || (type == RIGHT && !stereo_processed)) {
      std::cerr << Params::ERR_START << " Frame " << str_frameid <<
	" dropped " << Params::ERR_END << std::endl;
    }
  }

  void Frame::compute_descriptors(bool reextract) {

    std::vector<cv::Point2f> corners;
    
    //std::cout << "features_map size entering compute_decriptor is: " << features_map.size() <<"\n";

    //if corners are not yet extracted
    if(features_map.size() == 0) {
       //extract corners
      corners =  _extract_corners();
      //fill the features_map
      _fill_features_map(corners);
    }
    //if they are extracted already but not sufficient 
    else if(features_map.size() < size_t(Params::num_corners*Params::percent_min_corners) || reextract) {
      //reextract them
      std::cout << Params::CLR_GREEN << Params::CLR_INVERT <<
	" Re-extracting corners " << Params::CLR_RESET << std::endl;
      corners = _extract_corners(Params::num_corners+Params::addl_corners);
      //fill the features_map
      _fill_features_map(corners);
      //std::cout << "compute_descriptor extracting, size is:" << corners.size() <<"\n";
    }

    //std::cout << "features_map size before descriptor work is: " << features_map.size() <<"\n";

    //find the keypoints that need to create the descriptor
    std::vector<cv::KeyPoint> keypoints;
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
      if(iter->second->descriptor.desc.rows == 0) {
	      keypoints.push_back(cv::KeyPoint(iter->first,3));
      }
    }

    auto start = std::chrono::high_resolution_clock::now();

    //compute descriptors
    cv::Mat descriptors;
    FeatureDescriptor::get_instance().compute(img,keypoints,descriptors);

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "Computed " << keypoints.size() << " descriptors in " <<
      duration(start,finish) << " ms" << std::endl;
    
    //now fill the descriptors into the features object
    unsigned counter = 0;
    for(auto iter = keypoints.begin(); iter != keypoints.end(); ++iter) {
      // find the iterator
      auto bounds = features_map.equal_range(iter->pt);
      unsigned num_elements = std::distance(bounds.first,bounds.second);
      // num_elements must be 1
      if(num_elements > 1) {
      	std::cerr << Params::ERR_START << "More than one feature close to each other???"
      		  << Params::ERR_END << std::endl;
      	exit(-1);
      } else if(num_elements == 0) {
      	std::cerr << Params::ERR_START << "Couldn't find the element we calculated "
		  << "the descriptor for???" << Params::ERR_END << std::endl;
      	exit(-1);
      }
      //bounds.first is an iterator in features_map
      //feature_map is a map of point and feature 
      bounds.first->second->descriptor = Descriptor(descriptors.row(counter++),
						    FeatureDescriptor::get_instance().current,
						    FeatureDescriptor::get_instance().norm_type);
    }

    //while computing the feature descriptor, keypoints could have been deleted and new keypoints
    //added as per the compute function in the specs. So we need to iterate through the
    //the features list and find those entries which don't have descriptors and delete them
    for(auto iter = features_map.begin(); iter != features_map.end();)
      if(iter->second->descriptor.desc.rows == 0)
	      // std::cout << Params::ERR_START << "Missed descriptor for:" <<
	      //   iter->first <<  Params::ERR_END  <<std::endl;
	      iter = features_map.erase(iter);
      else
	      ++iter;
    //debug__display_corners(_corners(),"after_descriptor");
    //std::cout << "features_map size after descriptor work is: " << features_map.size() <<"\n";
  }

  std::shared_ptr<FMatches> Frame::track(std::shared_ptr<Frame> nxt) {
    //start the clock
    auto start = std::chrono::high_resolution_clock::now();
    std::shared_ptr<FMatches> fmatches = std::make_shared<FMatches>();
    fmatches->frame1 = shared_from_this();
    fmatches->frame2 = nxt;

    //first rescale the image
    _rescale();
    //its ok to rescale the next image as well because if its resized
    //already, then it wouldn't resize again
    nxt->_rescale();

    //convert to grayscale
    _to_gray();
    nxt->_to_gray();
    
    //next apply histogram equalization, if needed
    if(Params::equalize_histogram) {
      _equalize_histogram();
      nxt->_equalize_histogram();
    }
    
    //identify if its stereo tracking or frame2frame tracking
    TrackingType tracking_type;
    if(type == LEFT && nxt->type == RIGHT) {
      tracking_type = STEREO;
      nxt->stereo_processed = true;
      
    } 
    else if(type == LEFT && nxt->type == LEFT) {
      tracking_type = FRAME2FRAME;
      tracked = true;
    }
    else if(type == RIGHT && nxt->type == RIGHT) {
      tracking_type = FRAME2FRAME;
      tracked = true;
    } 
    else {
      std::cerr << Params::ERR_START << "Tracking is neither stereo not frame2frame!!!"
		    << Params::ERR_END << std::endl;
      exit(-1);
    }
    
    if(Params::tracking == Params::FEATURE_BASED) 
      _feature_tracking(nxt, *fmatches, tracking_type);
    else if(Params::tracking == Params::OPTICALFLOW_BASED)
      _opticalflow_tracking(nxt, *fmatches, tracking_type);
    else if(Params::tracking == Params::HYBRID_BASED)
      _hybrid_tracking(nxt, *fmatches, tracking_type);

    std::cout << *this << *nxt << std::endl;
    
    //after stereo tracking remove the features which are not tracked
    //so that they can be recomputed in optical flow if needed
    if(tracking_type == STEREO) {
      _remove_untracked_features(*fmatches);
      nxt->_remove_untracked_features(*fmatches);
    }

    std::cout << *this << *nxt << std::endl;

    //update the landmark database with the tracked features
    //LandmarkManager::get_instance().update_landmarks(shared_from_this());
    
    if (Params::tracking == Params::HYBRID_BASED && tracking_type == STEREO
	&& Params::display_tracking_ui)
      _displayLR(nxt, *fmatches);
    else if (Params::tracking != Params::HYBRID_BASED && tracking_type == STEREO
	     && Params::display_tracking_ui)
      _displayLR(nxt, *fmatches);
    else if (Params::tracking != Params::HYBRID_BASED && Params::display_tracking_ui &&
	     type == LEFT)
      _display(nxt,*fmatches,tracking_type);
      

    auto finish = std::chrono::high_resolution_clock::now();

    if(Params::tracking == Params::FEATURE_BASED)
      std::cout << " Feature_based:(" << FeatureDescriptor::get_instance().str_desc << ")";
    else if (Params::tracking == Params::OPTICALFLOW_BASED)
      std::cout << " Opticalflow:";
    else if (Params::tracking == Params::HYBRID_BASED)
      std::cout << " Hybrid_based:";
    
    std::cout << (tracking_type == STEREO?"      Stereo: ":" Frame2frame: ") << 
      std::setw(5) << duration(start,finish) << "ms ";
    std::cout << "<" << str_frameid << nxt->str_frameid << "> "; 
    std::cout << " [" << std::setw(4) << fmatches->matches.size() << " matches] ";
    std::cout << " [" << std::setw(4) << std::fixed << std::setprecision(2) 
              << Params::rescale_factor << " x]" << std::endl;

    return fmatches;
  }  

  std::ostream& operator<<(std::ostream &out, const Frame &f) {

    unsigned tracked_count = 0;
    for(auto iter=f.features_map.begin(); iter != f.features_map.end(); ++iter) {
      if(iter->second->tracked) {
	tracked_count++;
      }
    }
    
    out << Params::CLR_MAGENTA << std::setw(7) <<  f.str_frameid <<
      Params::CLR_RESET << "(:" << std::setw(5) << f.features_map.size() << "," <<
      " Stereo tracked:" << tracked_count << ")";
    return out;
  }

  void Frame::_rescale() {
    if(current_scale != Params::rescale_factor) {
      double scale = Params::rescale_factor/current_scale;
      cv::resize(img,img,cv::Size(img.cols*scale,
				  img.rows*scale));
      current_scale = Params::rescale_factor;
    }
    max_disparity = Params::max_disparity; 
  }  
  
  void Frame::_equalize_histogram() {
    if(!histogram_equalized) {
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3., cv::Size(8,8));
      clahe->apply(gray,gray);
      histogram_equalized = true;
    }
  }

  std::vector<cv::Point2f> Frame::_extract_corners(unsigned num_corners) {
    //convert to grayscale    
    _to_gray();

    auto start = std::chrono::high_resolution_clock::now();
    
    //find corners to track
    std::vector<cv::Point2f> corners;
    double min_distance = Params::feature_extraction_boundary;
    cv::goodFeaturesToTrack(gray, corners, num_corners,
			    0.01, //quality level
			    min_distance, //min distance
			    cv::Mat(),
			    3, //block size
			    false, //useHarrisDetector
			    0.04); //k

    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << "Computed " << corners.size() << " corners " <<
      duration(start,finish) << " ms for " << str_frameid << std::endl;

    //refine the corners
    if(Params::refine_corners_after_corner_extraction)
      _refine_corners(corners);

    //debug__display_corners(corners,"extract_corners");

    //filter the corners which lie on the image border
    double border = Params::percent_border;

    std::vector<cv::Point2f> filtered_corners;
    
    for(auto iter = corners.begin(); iter != corners.end(); ++iter) {

      if(iter->x > gray.cols * (1-border) ||
	 iter->x < gray.cols * border ||
	 iter->y > gray.rows * (1-border) ||
	 iter->y < gray.rows * border) {
	continue;
      } else {
	filtered_corners.push_back(*iter);
      }
    }
    return filtered_corners;
    //return corners;
  }

  void Frame::_refine_corners(std::vector<cv::Point2f> &corners) {
    //convert to grayscale
    _to_gray();
    //set size constant
    cv::Size win_size(5,5);
    cv::Size zero_zone(-1,-1);
    cv::TermCriteria criteria(cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER,40,0.001);
    //calculate
    cv::cornerSubPix(gray,corners,win_size,zero_zone,criteria);
  }
 
  std::vector<cv::Point2f> Frame::_corners() {
    // obtain corners from feature_map
    std::vector<cv::Point2f> corners;
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
      corners.push_back(iter->first);
    }
    return corners;
  }

  std::vector<cv::KeyPoint> Frame::_keypoints() {
    // obtain cv::KeyPoint from features_map
    std::vector<cv::KeyPoint> keypoints;
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter)
      keypoints.push_back(cv::KeyPoint(iter->first,3));
    return keypoints;
  }
  
  void Frame::_to_gray() {
    if(gray.rows == 0 || gray.cols == 0) {
      if(img.channels() == 1)
       	gray = img;
      else 
	cv::cvtColor(img,gray,CV_RGB2GRAY);
    }
  }
  
  void Frame::_fill_features_map(std::vector<cv::Point2f> &corners) {

    unsigned counter = 0;
      
    //If features_map is empty this is the first time corners are extracted
    //from this frame. It should be ok to add all the points in without
    //any checking (assuming the corners are not close to each other - the
    //goodfeaturestotrack method should have already checked for that)
    if(features_map.size() == 0) {
      for(auto iter = corners.begin(); iter != corners.end(); ++iter) {
	features_map[*iter] = std::make_shared<Feature>(*iter);
	counter++;
      }

    } else {
    
      
      //build a kd-tree to search for the points
      FeaturePoints feature_pts;
      for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
	feature_pts.pts.push_back(iter->first);
      }
      nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, FeaturePoints>,
					  FeaturePoints, 2 /*dims*/>
	index(2,feature_pts, nanoflann::KDTreeSingleIndexAdaptorParams(10/*max leaf */));
      index.buildIndex();
    
      const float search_radius = pow(Params::feature_search_boundary,2);
      
      for(auto iter = corners.begin(); iter != corners.end(); ++iter) {
	
	float query_pt[2] = {iter->x, iter->y};
	std::vector<std::pair<size_t,float> > ret_matches;
	nanoflann::SearchParams params;
	size_t nmatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

	if(nmatches > 0) {
	  // std::cout << Params::CLR_MAGENTA << Params::CLR_INVERT << "Found " <<
	  //   ret_matches.size() <<
	  //   " features " << *iter <<  " closer to the new point with distance " <<
	  //   ret_matches[0].second << " Pt:" << feature_pts.pts[ret_matches[0].first] <<
	  //   " search radius " << search_radius << Params::CLR_RESET << std::endl;
	} else {
	  features_map[*iter] = std::make_shared<Feature>(*iter);
	  counter++;
	}
      }
    }
  }

  void Frame::_print_features_map() {
    std::cout << "\nFrame:" << str_frameid << std::endl;
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
      std::cout << *(iter->second) << std::endl;
    }
    std::cout << std::endl;
  }

  void Frame::_mark_tracked_features(std::shared_ptr<Frame> nxt,
				     std::vector<unsigned> &tracked_ids,
				     FMatches &fmatches) {

    //iterate through the feature ids in both the frames and clear those features
    //which are tracked in both the left and right images	
    std::set<unsigned> feature_ids;
    //store the feature id and the corresponding index in the features_map datastructure
    std::map<unsigned,std::shared_ptr<Feature> > index_map;

    unsigned counter = 0;
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {

      feature_ids.insert(iter->second->id);
      index_map[iter->second->id] = iter->second;

      //reset the tracked flags-we shall set it in the next loop
      iter->second->tracked = false;
    }

    
    for(auto iter = nxt->features_map.begin(); iter != nxt->features_map.end(); ++iter) {

      unsigned feature_id = iter->second->id;
      
      //see if this feature is tracked
      if(feature_ids.count(feature_id) > 0) {
	
	//entry found, which means this feature is tracked
	//lets keep track of this feature id 
	tracked_ids.push_back(feature_id);

	//add match
	//std::shared_ptr<FMatch> m = std::make_shared<FMatch>();
	std::shared_ptr<FMatch> m(new FMatch);
	m->f1 = index_map[feature_id];
	m->f2 = iter->second;
	m->desc_dist = m->f1->descriptor.distance;
	fmatches._add_entry(m);
	
	//and set both the features as tracked
	iter->second->tracked = true;
	index_map[feature_id]->tracked = true;

      } else {
	//reset the tracked flags
	//and set both the features as tracked
	iter->second->tracked = false;
      }
    }
  }

  void Frame::_get_all_descriptors(cv::Mat &desc) {

    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {

      //ignore the features which are tracked
      if(iter->second->tracked) {
	continue;
      }
      
      desc.push_back(iter->second->descriptor.desc);
    }
    
  }

  void Frame::_get_features_index_map(std::map<unsigned,std::shared_ptr<Feature> > &index_map) {

    unsigned counter = 0;
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {

      //ignore the features which are tracked
      if(iter->second->tracked) {
	continue;
      }
      
      index_map[counter] = iter->second;
      counter++;
    }
  }

  void Frame::_remove_untracked_features(FMatches &fmatch) {

    //iterate through the feature matches and store all the ids in a set
    //iterate through the features_map, if the entry does not exist in the features map,
    //delete the entry in the features_map

    std::set<unsigned> matched_ids;
    for(auto iter = fmatch.matches.begin(); iter != fmatch.matches.end(); ++iter) {
      matched_ids.insert((*iter)->f1->id);
    }
        
    auto iter = features_map.begin();
    while(iter != features_map.end()) {

      unsigned id = iter->second->id;
      if(matched_ids.count(id) == 0) {
	iter = features_map.erase(iter);
      } else {
	++iter;
      }
    }

  }
  
  
  void Frame::_opticalflow_tracking(std::shared_ptr<Frame> nxt,
				    FMatches &fmatches,
				    TrackingType tracking_type) {
    
    fmatches._check_datastructure_integrity("opticalflow_tracking");    
    
    if(debug)  {
      std::cout << "OPTICALFLOW_TRACKING: ";
      
      if(tracking_type == FRAME2FRAME)
      	std::cout << "Frame2frame tracking:" << std::endl;
      else
      	std::cout <<"Stereo tracking:" << std::endl;
    
      debug__display_features_map("left");
      nxt->debug__display_features_map("right");
      
      std::cout << "Existing features:" << str_frameid << std::endl;
      for(auto iter = features_map.begin(); iter != features_map.end(); ++iter)
	      std::cout << *(iter->second) << std::endl;
      
      std::cout << "Existing features (nxt):" << nxt->str_frameid << std::endl;
      for(auto iter = nxt->features_map.begin(); iter != nxt->features_map.end(); ++iter)
	      std::cout << *(iter->second) << std::endl;
    }

    std::vector<cv::Point2f> tracked_corners;
    std::vector<unsigned char> status;
    std::vector<float> err;

    cv::Size search_window;

    if(features_map.size() == 0) {
      //extract corners
      std::vector<cv::Point2f> corners =  _extract_corners();
      //fill the features_map
      _fill_features_map(corners);
    }

    if(tracking_type == FRAME2FRAME) 
      search_window = Params::frame2frame_tracking_search_window;
    else 
      search_window = Params::stereo_tracking_search_window;

    nxt->_to_gray();
    cv::calcOpticalFlowPyrLK(gray, nxt->gray, 
			     _corners(),
			     tracked_corners,
			     status,
			     err,
			     search_window,
			     5,
			     cv::TermCriteria(cv::TermCriteria::COUNT +
					      cv::TermCriteria::EPS, 30, 0.01), 
			     cv::OPTFLOW_LK_GET_MIN_EIGENVALS,
			     0.01);
           
    //refine corners if needed
    if(Params::refine_corners_after_optical_flow) {
      //tracked_corners are the points on the next frame. So, to refine those corners
      //we need to use the next frame
      nxt->_refine_corners(tracked_corners);
    }

    auto fmap_iter = features_map.begin();
    auto tracked_corners_iter = tracked_corners.begin();

    //store information of the features which are not tracked
    std::vector<bool> untracked_features;
    unsigned untracked_counter = 0;
    
    for(auto status_iter = status.begin(); status_iter != status.end();
	++status_iter, ++tracked_corners_iter, ++fmap_iter,++untracked_counter) {      
      //std::cout << static_cast<unsigned>(*status_iter) << std::endl; 
      //if the feature is tracked successfully
      untracked_features.push_back(false);
      if(*status_iter == 1) {

	double border = Params::percent_border;
	//filter the points which lie on the boundary of the image
	if(tracked_corners_iter->x > gray.cols * (1-border) ||
	   tracked_corners_iter->x < gray.cols * border ||
	   tracked_corners_iter->y > gray.rows * (1-border) ||
	   tracked_corners_iter->y < gray.rows * border) {
	  untracked_features[untracked_counter] = true;
	  continue;
	}

	//filter the points which are beyond the y threshold
	if(fabs(fmap_iter->second->lpix.y - tracked_corners_iter->y) >
	   (float)Params::max_disparity_alignment_threshold) {
	  untracked_features[untracked_counter] = true;
	  continue;
	}
	
        std::shared_ptr<FMatch> fmatch(new FMatch);
        fmatch->f1 = fmap_iter->second;
        fmatch->f2 = std::shared_ptr<Feature>(new Feature);
        fmatch->f2->id = fmatch->f1->id;
        fmatch->f2->lpix = *tracked_corners_iter;

	
        if(tracking_type == STEREO) {
          //fill in the disparity and depth measurements
          //TODO: shouldn't this be the other way round (rightx - leftx)?
          double disparity = fmap_iter->first.x - tracked_corners_iter->x;

          double row_diff = fabs(tracked_corners_iter->y - fmap_iter->first.y);
          //if disparity is negative or difference 
          if(disparity < 0 || row_diff > Params::max_disparity_alignment_threshold) {
            //do not add to the feature matches list, just continue
	    untracked_features[untracked_counter] = true;
            continue;
	  } else {
            fmap_iter->second->disparity = disparity;
            fmap_iter->second->depth = Params::right_cam_info.baseline/disparity;
          }
        } else if(tracking_type == FRAME2FRAME) {
          //fill features_map of the next frame with the tracked corners
          nxt->features_map[*tracked_corners_iter] = fmatch->f2;
	}

	//if the match is close to the previous match, ignore the weaker match
	int fmatch_last_idx = fmatches.matches.size() - 1;
	if(fmatch_last_idx >= 0) {
	  auto prev_fmatch = fmatches.matches[fmatch_last_idx];
	  auto diff = prev_fmatch->f1->lpix - fmatch->f1->lpix;
	  //std::cout << "Distance between the previous and current matches: x:" <<
	  //  diff.x << " y:" << diff.y << std::endl;
	  //std::cout << "Boundary:" << Params::feature_search_boundary << std::endl;
	  if(fabs(diff.x) < Params::feature_search_boundary &&
	     fabs(diff.y) < Params::feature_search_boundary) {

	    //ignore one of them
	    if(prev_fmatch->f1->descriptor.distance < fmatch->f1->descriptor.distance) {
	      //previous match is stronger, lets ignore the current one
	      untracked_features[untracked_counter] = true;
	      continue;
	    } else {
	      //replace the previous match with the current match
	      prev_fmatch = fmatch;
	      untracked_features[untracked_counter] = true;
	    }
	  } else {
	    //the previous match is not close to the current match, lets add it to the matches
	    fmatches.matches.push_back(fmatch);
	  } 
	} else {
	  //the previous match is not close to the current match, lets add it to the matches
	  fmatches.matches.push_back(fmatch);
	}
      }
    }

    //iterate through the features_map and delete the features which are not tracked
    fmap_iter = features_map.begin();
    auto untracked_iter = untracked_features.begin();
    while(untracked_iter != untracked_features.end()) {
      if(*untracked_iter) {
	fmap_iter = features_map.erase(fmap_iter);
      } else {
	++fmap_iter;
      }
      ++untracked_iter;      
    }

    //std::cout << "after _opticalflow_tracking nxt size: " << nxt->features_map.size() << "\n";

    if(tracking_type == FRAME2FRAME && type == LEFT && Params::tracking != Params::HYBRID_BASED) {
      //find the number of features that are tracked
      unsigned num_features_tracked = std::accumulate(status.begin(), status.end(), 0);

      //if the number of matches is lesser than the threshold. find more (new) corners      
      if(num_features_tracked < (Params::num_corners * Params::percent_min_corners)) {
        //find more corners
	std::cout << Params::CLR_GREEN << Params::CLR_INVERT <<
	  " Re-extracting corners " << Params::CLR_RESET << std::endl;
        std::vector<cv::Point2f> new_corners = _extract_corners(Params::num_corners+
                      Params::addl_corners);

        std::cout << "Adding more corners" << std::endl;
        //add corners which do not fall within the epsilon threshold as features to the
        //features_map of the next frame	

	//build a kd-tree to search for the points
	FeaturePoints feature_pts;
	for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
	  feature_pts.pts.push_back(iter->first);
	}
	nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, FeaturePoints>,
					    FeaturePoints, 2 /*dims*/>
	  index(2,feature_pts, nanoflann::KDTreeSingleIndexAdaptorParams(10/*max leaf */));
	index.buildIndex();
	
	const float search_radius = pow(Params::feature_search_boundary,2);

        for(auto corner_iter = new_corners.begin(); corner_iter != new_corners.end();
            ++corner_iter) {
	  float query_pt[2] = {corner_iter->x, corner_iter->y};
	  std::vector<std::pair<size_t,float> > ret_matches;
	  nanoflann::SearchParams params;
	  size_t nmatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
	  
	  if(nmatches > 0) {
	    // std::cout << Params::CLR_MAGENTA << Params::CLR_INVERT << "Found " <<
	    //   ret_matches.size() <<
	    //   " features " << *corner_iter <<  " closer to the new point with distance " <<
	    //   ret_matches[0].second << " Pt:" << feature_pts.pts[ret_matches[0].first] <<
	    //   " search radius " << search_radius << 
	    //   Params::CLR_RESET << std::endl;
	  } else {
            //add it in
            nxt->features_map[*corner_iter] =  std::make_shared<Feature>(*corner_iter);
            //Feature::make_feature(*corner_iter);
          }
        }	
      }
    }
  }

  void Frame::_bruteforce_feature_tracking(std::shared_ptr<Frame> nxt,
					   FMatches &fmatches,
					   TrackingType type) {
    
    compute_descriptors();
    nxt->compute_descriptors();

    cv::Mat desp_l, desp_r;
    _get_all_descriptors(desp_l);
    nxt->_get_all_descriptors(desp_r);

    std::vector< cv::DMatch > matches;

    if(desp_l.rows != 0 && desp_r.rows != 0) {
      
      FeatureDescriptor::get_instance().bf_matcher.match(desp_l, desp_r, matches);
    }
    std::map<unsigned,std::shared_ptr<Feature> > features_index_map;
    std::map<unsigned,std::shared_ptr<Feature> > nxt_features_index_map;

    _get_features_index_map(features_index_map);
    nxt->_get_features_index_map(nxt_features_index_map);

    for(size_t i=0; i<matches.size(); ++i) {
     
      std::shared_ptr<Feature> f1 = features_index_map[matches[i].queryIdx];
      std::shared_ptr<Feature> f2 = nxt_features_index_map[matches[i].trainIdx];
      
      cv::Point2f lpt = f1->lpix;
      cv::Point2f rpt =f2->lpix;
      
      double y_dist = fabs(lpt.y - rpt.y);
      double x_dist = fabs(lpt.x - rpt.x);
     
      if(type == FRAME2FRAME) {
	
	if(y_dist > Params::feature_search_boundary ||
	   x_dist > Params::feature_search_boundary) {
	  continue;
	}

      } else if(type == STEREO) {
	
	if(y_dist > Params::max_disparity_alignment_threshold ||
	   x_dist > max_disparity/2) {
	  continue;
	}
      }

      //also check the descriptor distance
      if(matches[i].distance > Params::matcher_dist_threshold) {
	      continue;
      }

      //add the entry in
      std::shared_ptr<FMatch> m = std::make_shared<FMatch>();
      m->f1 = f1;
      m->f2 = f2;
      m->desc_dist = matches[i].distance;

      fmatches.matches.push_back(m);

    }

    //sort and resize
    std::sort(fmatches.matches.begin(),fmatches.matches.end());
    unsigned max_matches =  unsigned(Params::num_corners*Params::percent_matches);
    if(fmatches.matches.size() > max_matches)
      fmatches.matches.resize(max_matches);

    std::set<unsigned> feature_ids_set;

    // std::cout << "After sorting:" << std::endl;
    //for all the matched features, set the smaller id as the feature id
    for(auto iter = fmatches.matches.begin(); iter != fmatches.matches.end(); ++iter) {

      unsigned id = ((*iter)->f1->id < (*iter)->f2->id?(*iter)->f1->id:(*iter)->f2->id);

      if(feature_ids_set.count(id) > 0) {
	//id already assigned, choose the greater id
	id =  ((*iter)->f1->id > (*iter)->f2->id?(*iter)->f1->id:(*iter)->f2->id);
      } 
      if(debug) std::cout << "F1:" << *((*iter)->f1) << " F2:" << *((*iter)->f2) <<
		  " Chosen:" << id << std::endl;
      
      (*iter)->f1->id = (*iter)->f2->id = id;
      feature_ids_set.insert(id);
    }
    
  }
  

  void Frame::_feature_tracking(std::shared_ptr<Frame> nxt,
				FMatches &fmatches,
				TrackingType tracking_type) {

    fmatches._check_datastructure_integrity("feature_tracking");
    
    compute_descriptors();
    nxt->compute_descriptors();

    //_print_features_map();
    //nxt->_print_features_map();
        

    auto frame1_iter = features_map.begin();
    auto frame2_iter = nxt->features_map.begin();

    if(debug)  {

      std::cout << "FEATUREBASED_TRACKING: ";

      if(tracking_type == FRAME2FRAME)
      	std::cout << "Frame2frame tracking:" << std::endl;
      else
      	std::cout <<"Stereo tracking:" << std::endl;
    
      debug__display_features_map("left");
      nxt->debug__display_features_map("right");
      
      std::cout << "Existing features:" << str_frameid << std::endl;
      for(auto iter = features_map.begin(); iter != features_map.end(); ++iter)
	      std::cout << *(iter->second) << std::endl;
      
      std::cout << "Existing features (nxt):" << nxt->str_frameid << std::endl;
      for(auto iter = nxt->features_map.begin(); iter != nxt->features_map.end(); ++iter)
	      std::cout << *(iter->second) << std::endl;
    }

    // std::cout << "before featureMatching this size: " << _corners().size() << "\n";
    // std::cout << "before featureMatching next size: " << nxt->_corners().size() << "\n";
    
    //create the working set
    std::vector<std::shared_ptr<Feature> > prev_working_set;
    // iterate through features_map
    for(;frame1_iter != features_map.end() && frame2_iter != nxt->features_map.end();
	      ++frame1_iter) {

      double min_x,max_x,min_y,max_y;
      
      //check if frame2 keypoint lies in the range
      if(tracking_type == FRAME2FRAME) {
        min_y = frame1_iter->second->lpix.y - Params::feature_search_boundary;
        max_y = frame1_iter->second->lpix.y + Params::feature_search_boundary;
        min_x = frame1_iter->second->lpix.x - Params::feature_search_boundary;
        max_x = frame1_iter->second->lpix.x + Params::feature_search_boundary;
      } 
      else if(tracking_type == STEREO) {
        min_y = frame1_iter->second->lpix.y - Params::max_disparity_alignment_threshold;
        max_y = frame1_iter->second->lpix.y + Params::max_disparity_alignment_threshold;
        //x_search_radius should span between 0 and image width
        min_x = frame1_iter->second->lpix.x - max_disparity/2;
        max_x = frame1_iter->second->lpix.x + max_disparity/2;
      }

      //ignore if this feature is already tracked
      if(tracking_type == STEREO && frame1_iter->second->tracked) {
	continue;
      }

      //create the working set
      std::vector<std::shared_ptr<Feature> > working_set;

      //check if we have entries from the previous working set that need to be considered
      for(auto prev_iter=prev_working_set.begin(); 
          prev_iter != prev_working_set.end();
	        ++prev_iter) {
        //skip the pixel rows which fall beyond (lesser) the tolerance range
        if((*prev_iter)->lpix.y < min_y)
          continue;
        //pick the pixels which fall within the tolerance limit
        if((*prev_iter)->lpix.y <= max_y)
          working_set.push_back(*prev_iter);
      }

      //now apply the same logic to the pixels in frame2_iter
      while(frame2_iter != nxt->features_map.end()) {
        //skip the pixels rows which fall beyond (lesser) the tolerance range
        if(frame2_iter->second->lpix.y < min_y) {
          frame2_iter++;
          continue;
	      }
        //pick the pixels which fall within the tolerance limit
        if(frame2_iter->second->lpix.y <= max_y) {
          working_set.push_back(frame2_iter->second);
          frame2_iter++;
        } else {
          break;
        }
      }

      //set it so we can use in the next iteration
      prev_working_set = working_set;

      //now find the norm for the descriptors in the working set
      for(auto working_iter = working_set.begin();
	        working_iter != working_set.end();
	        ++working_iter) {

        //Now check if we are within the x bounds. Since we have set separate
        //bounds for both frame2frame and stereo, we should be ok
        if((*working_iter)->lpix.x < min_x || (*working_iter)->lpix.x >= max_x)
          continue;

        if(tracking_type == STEREO) {

	  //ignore the feature if its already tracked
	  if((*working_iter)->tracked) {
	    continue;
	  }
	  
	  //set the disparity
          //right - left
          double disparity = (*working_iter)->lpix.x - frame1_iter->second->lpix.x;

          // if(disparity < 0) {
          //   continue;

          // } else {
          frame1_iter->second->disparity = disparity;
          frame1_iter->second->depth = Params::right_cam_info.baseline/disparity;
          // }
        } 
          
        double desc_dist;
        if(Params::descriptor_distance_both_ways)
        //calculate the norm distance of the descriptor (both ways)
          desc_dist = (*working_iter)->descriptor.double_dist(frame1_iter->second->descriptor);
        else
          desc_dist = (*working_iter)->descriptor.dist(frame1_iter->second->descriptor);

      
        //if the distance is not infinity, then add it to the list
        //if its infinity, then its intentionally set to be ignored
        //so skip adding it to the list
	//if the distance is beyond the set distance threshold, ignore it
        if(!std::isinf(desc_dist) && desc_dist < Params::matcher_dist_threshold) {
          if(debug) std::cout << "Trying:" << *(frame1_iter->second) << " and "
                  << *(*working_iter) << std::endl;
	  
          fmatches.add_match(frame1_iter->second,*working_iter,desc_dist);
        }

        // if(debug) {
        //   std::cout << "Matches:";
        //   for(auto iter = fmatches.matches.begin(); iter != fmatches.matches.end(); ++iter)
        //     std::cout << "(" << ((*iter)->f1->id) << "," << ((*iter)->f2->id) << ") ";      
        //   std::cout << std::endl;
        // }
      }
    }
    
    if(debug) {
      std::cout << std::endl << "Before sorting (and resizing) size:  "<<  fmatches.matches.size() << std::endl;
      //for all the matched features, set the smaller id as the feature id
      for(auto iter = fmatches.matches.begin(); iter != fmatches.matches.end(); ++iter)
	      std::cout << "F1:" << *((*iter)->f1) << " F2:" << *((*iter)->f2) << std::endl;      
      std::cout << std::endl;
    }

    
    //sort the matches and choose the top n entries
    //Note: after sorting the features1_map and features2_map data structure
    //will become invalid
    std::sort(fmatches.matches.begin(),fmatches.matches.end());
    unsigned max_matches =  unsigned(float(Params::num_corners)*Params::percent_matches);
    if(fmatches.matches.size() > max_matches)
      fmatches.matches.resize(max_matches);
    
    // std::cout << "After sorting:" << std::endl;
    //for all the matched features, set the smaller id as the feature id
    for(auto iter = fmatches.matches.begin(); iter != fmatches.matches.end(); ++iter) {
      unsigned id = ((*iter)->f1->id < (*iter)->f2->id?(*iter)->f1->id:(*iter)->f2->id);
      if(debug) std::cout << "F1:" << *((*iter)->f1) << " F2:" << *((*iter)->f2) <<
		  " Chosen:" << id << std::endl;
      
      (*iter)->f1->id = (*iter)->f2->id = id;
    }
    
  }


  void Frame::_hybrid_tracking(std::shared_ptr<Frame> nxt,
				    FMatches &fmatches,
				    TrackingType tracking_type) {
    // frame2frame
    if(debug) std::cout << "Inside hybrid tracking" << std::endl;
    if (tracking_type == FRAME2FRAME)
      _opticalflow_tracking(nxt, fmatches, tracking_type);
    else if (tracking_type == STEREO) {

      //count the number of features tracked
      std::vector<unsigned> tracked_feature_ids;

      //If the features are tracked, feature_tracking is not called
      //and fmatches is empty. This has to be filled with already tracked
      //features in mark_tracked_features
      _mark_tracked_features(nxt,tracked_feature_ids,fmatches);

      double tracked_features = float(tracked_feature_ids.size());
      double threshold = (float(Params::num_corners) * Params::hybrid_percent_tracked_corners);
      // std::cout << "Tracked features count :" << tracked_features << " Threshold:" <<
      // 	threshold << std::endl;
      //if its below the set threshold, re-extract corners and track them using feature_tracking
      if(tracked_features < threshold) {

	if(Params::custom_matcher) {
	  _feature_tracking(nxt,fmatches,tracking_type);
	} else {
	  _bruteforce_feature_tracking(nxt,fmatches,tracking_type);
	}
	
      }
      //if its above the threshold, just continue
      
      
    }      
  }

  void Frame::_displayLR(std::shared_ptr<Frame> nxt, FMatches &matches) {
    // create window
    cv::namedWindow("Stereo Frames", cv::WINDOW_NORMAL);
    // initialize
    cv::Mat canvasL, canvasR;
    img.copyTo(canvasL);
    nxt->img.copyTo(canvasR);
    //convert to color image
    if(canvasL.channels() == 1)
      cv::cvtColor(canvasL, canvasL, cv::COLOR_GRAY2BGR);
    if(canvasR.channels() == 1)
      cv::cvtColor(canvasR, canvasR, cv::COLOR_GRAY2BGR);
    // plot matched points
    for(auto iter = matches.matches.begin(); iter != matches.matches.end(); ++iter) {
      cv::circle(canvasL,(*iter)->f1->lpix, 2, cv::Scalar( 0,255.,0 ), 1, 8, 0 );
      cv::circle(canvasR,(*iter)->f2->lpix, 2, cv::Scalar( 0,255.,0 ), 1, 8, 0 );
    }
		// resize
		//cv::resize(canvasL, canvasL, cv::Size(), 1, 1);
		//cv::resize(canvasR, canvasR, cv::Size(), 1, 1);
		// Showing the result
	  cv::Size sz1 = canvasL.size();
	  cv::Size sz2 = canvasR.size();
	  cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
	  cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
	  canvasL.copyTo(left);
	  cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
	  canvasR.copyTo(right);
	  // text
	  //cv::putText(im3, "This one!", cv::Point2f(230, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
	  imshow("Stereo Frames", im3);
    cv::resizeWindow("Stereo Frames",cv::Size(cv::Size(1450,680)));
	  // image to file
    std::stringstream ss;
    ss << "/home/wzshao/Desktop/myOut" << "/" << "frame_" << frameid << ".jpg";
    std::string fullPath = ss.str();
		imwrite(fullPath, im3);
    cv::waitKey(1);
  }

  void Frame::_display(std::shared_ptr<Frame> nxt,
		       FMatches &matches,
		       TrackingType tracking_type) {

    //blue - actively tracked points
    //red - points that are not tracked in this frame
    //green - actively tracked and has depth
    cv::Scalar blue(255,0,0);
    cv::Scalar red(0,0,255);
    cv::Scalar green(0,255,0);
    cv::Scalar cyan(255,0,255);

    cv::namedWindow("Matches", cv::WINDOW_NORMAL);

    cv::Mat canvas;
    img.copyTo(canvas);
    //cv::Mat canvas = img;
    
    //convert to color image
    if(canvas.channels() == 1) {
      cv::cvtColor(canvas,canvas,cv::COLOR_GRAY2BGR);
    }

    //iterator through the features map
    //display all points as red, if its tracked we can overwrite it with blue
    //draw green circles for points with depth
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
      cv::circle(canvas, iter->first,2,red,2);
      _write_feature_id(canvas,iter->second);

      if(!std::isnan(iter->second->disparity))
	      cv::circle(canvas, iter->first,4,green,2); 
    }
    
    //now plot the tracked points
    for(auto iter = matches.matches.begin(); iter != matches.matches.end(); ++iter) {
	
      cv::circle(canvas,(*iter)->f1->lpix,2,blue,2);
      //_write_feature_id(canvas,iter->f1);

      if(Params::display_verbose_output) {
        //display the matches
        cv::circle(canvas,(*iter)->f2->lpix,2,cyan,2);
        _write_feature_id(canvas,(*iter)->f2);
        cv::line(canvas,(*iter)->f1->lpix,(*iter)->f2->lpix,cyan,2);
      }
    }

    //write some useful information on the image
    std::stringstream txt;
    unsigned line_num = 1;
    txt << "Tracking type:" << (tracking_type==FRAME2FRAME?"Frame2frame":"Stereo");
    _add_text_to_canvas(canvas,txt.str(),line_num++);
    txt.str("");
    txt << str_frameid << " Features:" << features_map.size() <<
      " Scale:" << current_scale;
    _add_text_to_canvas(canvas,txt.str(),line_num++);
    txt.str("");
    txt << nxt->str_frameid << " Features:" << nxt->features_map.size() <<
      " Scale:" << current_scale;
    _add_text_to_canvas(canvas,txt.str(),line_num);
		
    cv::imshow("Matches",canvas);

    static int counter = 0;
    std::stringstream ss;
    ss << "matches_" << counter++ << ".jpg";
    
    if(Params::write_matches_to_file)
      cv::imwrite(ss.str(),canvas);

    cv::resizeWindow("Matches",cv::Size(canvas.cols*1.5,canvas.rows*1.5));
    cv::waitKey(1);
  }


  void Frame::_add_text_to_canvas(cv::Mat &canvas, std::string txt,unsigned line_num) {
    const unsigned pix_increment = 20;
    cv::Scalar red(0,0,255);

    cv::putText(canvas,txt,
		cv::Point(pix_increment,pix_increment*line_num),//coordinates
		cv::FONT_HERSHEY_COMPLEX_SMALL,
		0.75,//scale 2.0=2x bigger
		red,//color
		1);//thickness
  }

  void Frame::_write_feature_id(cv::Mat &canvas, std::shared_ptr<Feature> f) {
    cv::Scalar yellow(0,255,255);

    std::stringstream ss;
    ss << f->id;
    cv::putText(canvas,ss.str(),f->lpix,//coordinates
		cv::FONT_HERSHEY_COMPLEX_SMALL,
		0.75,//scale 2.0=2x bigger
		yellow,//color
		1);//thickness
    
  }
  
  void Frame::debug__display_corners(std::vector<cv::Point2f> corners,
				     std::string prefix) {
    ///debug
    std::stringstream ss;
    static int counter = 0;
    ss << prefix <<"_" << str_frameid << "_" << counter++ << ".jpg";
    cv::Mat canvas;
    img.copyTo(canvas);
    for(auto iter = corners.begin(); iter != corners.end(); ++iter) {
      cv::circle(canvas,*iter,2,cv::Scalar(0,0,255),2);
    }
    cv::imwrite(ss.str(),canvas);

  }

  void Frame::debug__display_features_map(std::string prefix) {
    ///debug
    std::stringstream ss;
    static int counter = 0;
    ss << prefix <<"_" << str_frameid << "_" << counter++ << ".jpg";
    cv::Mat canvas;
    img.copyTo(canvas);
    for(auto iter = features_map.begin(); iter != features_map.end(); ++iter) {
      cv::circle(canvas,iter->first,2,cv::Scalar(0,0,255),2);
      _write_feature_id(canvas,iter->second);
      
    }
    cv::imwrite(ss.str(),canvas);

  }

  
};

