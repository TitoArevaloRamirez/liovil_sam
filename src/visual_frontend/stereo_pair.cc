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
#include <visual_frontend/stereo_pair.h>

#include <iostream>
#include <sstream>

namespace VisualFrontend {

  StereoPair::StereoPair():
    processed_stereo(false),
    new_left(false),
    new_right(false)
  {}

  void StereoPair::add_left(std::shared_ptr<Frame> l) {
    left = l;
    new_left = true;
  }

  void StereoPair::add_right(std::shared_ptr<VisualFrontend::Frame> r) {
    right = r;
    new_right = true;
  }

  //TODO:Get rid of these methods and move the left and right variables to public
  std::shared_ptr<Frame> StereoPair::get_left() {
    return left;
  }

  std::shared_ptr<Frame> StereoPair::get_right() {
    return right;
  }

  bool StereoPair::has_left() {
    if(left) { return true; }
    else { return false; }
  }
  
  bool StereoPair::has_right() {
    if(right) { return true; }
    else { return false; }
  }
  
  bool StereoPair::ready_for_stereo() {
    return (new_left && new_right && 
	    left->extracted_corners && right->extracted_corners);
  }

  std::string StereoPair::get_ids() {
    if(!left || !right) {
      std::string ss;
      if(!left) {
	ss = "NO LEFT";
      }
      if(!right) {
	ss = ss + " NO RIGHT";
      }
      return ss;
    }
    return std::string("<" + left->str_frameid + "-" + right->str_frameid + ">");
  }

  std::string StereoPair::status() {
    std::stringstream result;
    result << "Left:" << (left?"yes":"no")  << " Right:" << (right?"yes":"no");
    return result.str();
  }
  
};

