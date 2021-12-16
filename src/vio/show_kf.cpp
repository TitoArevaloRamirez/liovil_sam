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
/*

	Show keyframe choice and the results

*/
#include <ros/ros.h>
#include <cstdlib>
#include <map>
#include <cmath>
#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <Eigen/Core>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vio/time_based_retriever.h>
//#include <visual_frontend/StereoFeatureMatches.h>

std::map<std::string, std::vector<Eigen::Vector4d> > featPoints;
//
// for eigen output
//Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
//std::string sep = "\n----------------------------------------\n";
// to file
//std::string folderName = "/home/wzshao/Desktop/myOutput";
std::string folderName = "/home/fr-ag/Documents/autel_misc";
int ct = 0;
std::string name = "keyframe_";
std::string type = ".jpg";
std::stringstream ss;
// callback left frame
void Lframe_callback(const sensor_msgs::ImageConstPtr& msgLeft) {
	// timestamp
	std::cout << std::setprecision(20) << msgLeft->header.seq << " " << msgLeft->header.stamp.toNSec()/1.0e9 << "\n";
	// get timestamp in string
	std::ostringstream os;
	os << std::setprecision(16) << msgLeft->header.stamp.toNSec()/1.0e9;
	std::string str = os.str();
	// find in keyframe database
	if ( featPoints.find(str) != featPoints.end() ) {
		std::cout << "callback at left seq " << msgLeft->header.seq << " left time " << str << std::endl;
		// get image
		cv::Mat grayLeft, grayRight;
		cv_bridge::CvImagePtr cv_ptrLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
		grayLeft = cv_ptrLeft->image;
		// put circle
		for (size_t i=0;i<featPoints[str].size();i++) {
			cv::Point2f leftToDraw(featPoints[str][i](0), featPoints[str][i](1));
			cv::circle( grayLeft, leftToDraw, 3, cv::Scalar( 0,255.,0 ), 2, 8, 0 );
			// cout
			//std::cout << featPoints[str][i].format(OctaveFmt);
			//std::cout << "Left: " << leftToDraw.x << ", " << leftToDraw.y << "\n";
			//std::cout << sep;
		}
		// resize
		cv::resize(grayLeft, grayLeft, cv::Size(), 0.75, 0.75);
		//cv::resize(grayRight, grayRight, cv::Size(), 0.5, 0.5); // imgIn, imgOut
		// Showing the result
	    // text
	    //cv::putText(im3, "This one!", cv::Point2f(230, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
	    imshow("left keyframe", grayLeft);
		cv::waitKey(100);
	}
}
// callback both frame
void frame_callback(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight) {
	// timestamp
	std::cout << std::setprecision(20) << msgLeft->header.seq << " " << msgLeft->header.stamp.toNSec()/1.0e9 << " "
	   	      << std::setprecision(20) << msgRight->header.seq << " " << msgRight->header.stamp.toNSec()/1.0e9 << "\n";
	// get timestamp in string
	std::ostringstream os;
	os << std::setprecision(16) << msgLeft->header.stamp.toNSec()/1.0e9;
	std::string str = os.str();
	// find in keyframe database
	if ( featPoints.find(str) != featPoints.end() ) {
		std::cout << "callback at left seq " << msgLeft->header.seq << " left time " << std::setprecision(20) << str
				  << " right seq " << msgRight->header.seq << " right time " << std::setprecision(20)
				  <<  msgRight->header.stamp.toNSec()/1.0e9 << std::endl;
		// get image
		cv::Mat grayLeft, grayRight;
		cv_bridge::CvImagePtr cv_ptrLeft = cv_bridge::toCvCopy(msgLeft, sensor_msgs::image_encodings::BGR8);
		cv_bridge::CvImagePtr cv_ptrRight = cv_bridge::toCvCopy(msgRight, sensor_msgs::image_encodings::BGR8);
		grayLeft = cv_ptrLeft->image;
		grayRight = cv_ptrRight->image;
		// put circle
		for (size_t i=0;i<featPoints[str].size();i++) {
			cv::Point2f leftToDraw(featPoints[str][i](0), featPoints[str][i](1));
			cv::Point2f rightToDraw(featPoints[str][i](2), featPoints[str][i](3));
			cv::circle( grayLeft, leftToDraw, 3, cv::Scalar( 0,255.,0 ), 2, 8, 0 );
			cv::circle( grayRight, rightToDraw, 3, cv::Scalar( 0,255.,0 ), 2, 8, 0 );
			/*
			// cout
			std::cout << featPoints[str][i].format(OctaveFmt);
			std::cout << "Left: " << leftToDraw.x << ", " << leftToDraw.y << "\n";
			std::cout << "Right: " << rightToDraw.x << ", " << rightToDraw.y << "\n";
			std::cout << sep;
			*/
		}
		// resize
		cv::resize(grayLeft, grayLeft, cv::Size(), 0.5, 0.5);
		cv::resize(grayRight, grayRight, cv::Size(), 0.5, 0.5); // imgIn, imgOut
		// Showing the result
	    cv::Size sz1 = grayLeft.size();
	    cv::Size sz2 = grayRight.size();
	    cv::Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
	    cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
	    grayLeft.copyTo(left);
	    cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
	    grayRight.copyTo(right);
	    // text
	    //cv::putText(im3, "This one!", cv::Point2f(230, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
	    imshow("keyframe pairs", im3);
	    // image to file
		ss << folderName << "/" << name << (ct++) << type;
		std::string fullPath = ss.str();
		ss.str("");
		//imwrite(fullPath, im3);
		cv::waitKey(1);
	}
}
// main
int main(int argc, char **argv) {
	// image folder
	std::string folderCreateCommand = "rm -rf " + folderName + "/";
  std::system(folderCreateCommand.c_str());
	folderCreateCommand = "mkdir " + folderName;
	std::system(folderCreateCommand.c_str());
	// ros
	ros::init(argc, argv, "show_keyframe");
  	ros::NodeHandle nh;
  	// read in file
  	std::ifstream inFile("/home/fr-ag/Documents/autel_ws2/devel/lib/visual_inertial_odometry/keyframes.txt");
	std::string time="init";
	std::string delimiter = ", ";
	std::vector<Eigen::Vector4d> temp;
	for( std::string line; getline(inFile,line); ) {
	    // a message
	    if (line.find(',') != std::string::npos) {
			size_t pos = 0;
			std::string token;
			Eigen::Vector4d tempVec;
			int i = 0;
			while ((pos = line.find(delimiter)) != std::string::npos) {
			    token = line.substr(0, pos);
			    tempVec(i++) = stod(token);
			    line.erase(0, pos + delimiter.length());
			}
			tempVec(i++) = stod(line);
			//std::cout << tempVec.format(OctaveFmt) << sep;
	    	temp.push_back(tempVec);
	    }
	    // a timestamp
	    else {
	    	featPoints[time] = temp;
	    	time = line;
	    	temp.clear();
	    	//std::cout << time << std::endl;
	    }
	}
	// last kf
	featPoints[time] = temp;
	std::cout << "size: " << featPoints.size() << std::endl;
	/*
	// cout
	for (std::map<std::string, std::vector<Eigen::Vector4d> >::iterator it = featPoints.begin();
			it != featPoints.end(); it++) {
		std::cout << it->first << std::endl;
		for (size_t i=0;i<it->second.size();i++) {
			std::cout << it->second[i].format(OctaveFmt) << sep;
		}
	}
	*/
	///*
	// register callback
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/mapping/left/image_rect_color", 100000);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/mapping/right/image_rect_color", 100000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback( boost::bind(frame_callback,_1,_2) );
	//*/

	//ros::Subscriber f_sub = nh.subscribe("/mapping/left/image_rect_color", 1000, Lframe_callback);
	//ros::Subscriber f_sub = nh.subscribe("/mapping/left/image_color", 1000, Lframe_callback);
	// visualization
	cv::namedWindow("keyframe", cv::WINDOW_NORMAL);
	cv::startWindowThread();
	// optimization loop
	//smartS_estimator->optimizeLoop();
	ros::spin();
	cv::destroyWindow("keyframe");
	return 0;
}
