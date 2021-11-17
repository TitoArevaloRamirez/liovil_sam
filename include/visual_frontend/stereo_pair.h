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
#ifndef __STEREOPAIR_H__
#define __STEREOPAIR_H__

#include "frame.h"

#include <iostream>
#include <memory>

namespace VisualFrontend {

  class StereoPair {
  public:
    StereoPair();
    void add_left(std::shared_ptr<Frame> left);
    void add_right(std::shared_ptr<Frame> right);

    std::shared_ptr<Frame> get_left();
    std::shared_ptr<Frame> get_right();
    
    bool has_left();
    bool has_right();

    bool ready_for_stereo();

    std::string get_ids();

    std::string status();
    
    bool processed_stereo;
    
  protected:
    std::shared_ptr<Frame> left;
    std::shared_ptr<Frame> right;

    bool new_left;
    bool new_right;
    
  };
};

#endif //__STEREOPAIR_H__
