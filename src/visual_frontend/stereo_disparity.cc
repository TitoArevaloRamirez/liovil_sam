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
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>


using namespace sensor_msgs;
using namespace message_filters;

void stereoCallback(const ImageConstPtr& left, const ImageConstPtr& right) {
  ROS_INFO_STREAM("Received callback");
}

int main(int argc, char **argv) {

  ros::init(argc,argv,"stereo_disparity");

  ros::NodeHandle nh;

  //add subscriber to the left and right images
  message_filters::Subscriber<Image> left_sub(nh,"/mapping/left/image_rect_color",1);
  message_filters::Subscriber<Image> right_sub(nh,"/mapping/right/image_rect_color",1);

  typedef sync_policies::ApproximateTime<Image, Image> stereoSyncPolicy;

  Synchronizer<stereoSyncPolicy> sync(stereoSyncPolicy(10), left_sub, right_sub);
  sync.registerCallback(boost::bind(&stereoCallback,_1,_2));

  ros::spin();
  return 0;
}
