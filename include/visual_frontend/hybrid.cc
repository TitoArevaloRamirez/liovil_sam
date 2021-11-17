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

  Naive frontend using LKT tracking on both left and right frames
                       Extract using opencv

*/
#include <fstream>
#include <iostream>
#include <math.h>
#include <vector>
#include <time.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <image_transport/image_transport.h>

#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visual_frontend/stereo_pair.h>
#include <visual_frontend/frame.h>
#include <visual_frontend/feature_operations.h>
#include <visual_frontend/stereo_tracker.h>
#include "parameters.h"

#include <Eigen/Dense>
#include <Eigen/Core>

// static fucntions
static bool inBBorder(const cv::Point2f &pt, float cols, float rows, float borderCheck) {
  return (pt.x > borderCheck && pt.y > borderCheck && pt.x < cols-borderCheck && pt.y < rows-borderCheck);
}
static void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status) {
    int j = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
static void reduceVector(std::vector<int> &v, std::vector<uchar> status) {
    int j = 0;
    for (size_t i = 0; i < v.size(); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
static void reduceMat(cv::Mat &v, std::vector<uchar> status) {
    int j = 0;
    cv::Mat newMat;
    for (size_t i = 0; i < v.rows; i++)
        if (status[i])
          newMat.push_back(v.row(i));   
    v = newMat.clone();
}
// class frontend
class FrontEnd {
private:
  bool useCorner, useFAST, useORB, useBrisk;
  uint64_t frame_id;
  // image segmentation
  int patchNum_r;
  int patchNum_c;
  std::vector<bool> patchCheck;
  // histogram equalization
  bool histogram_equalization;
  // corner detection
  int blockSize;
  double k;
  unsigned maxCorners;
  // qualityLevel – Characterizes the minimal accepted quality of image corners;
  // the value of the parameter is multiplied by the by the best corner quality
  // measure (which is the min eigenvalue, see cornerMinEigenVal() ,
  // or the Harris function response, see cornerHarris() ).
  // The corners, which quality measure is less than the product, will be rejected.
  // For example, if the best corner has the quality measure = 1500,
  // and the qualityLevel=0.01 , then all the corners which quality measure is
  // less than 15 will be rejected.
  double qualityLevel;
  // minDistance – The minimum possible Euclidean distance between the returned corners
  double minDistance;
  bool useHarrisDetector;
  // FAST detector
  int fastThresh;
  cv::BFMatcher fast_matcher;
  // ORB descriptor
  cv::Ptr<cv::ORB> ptrORB;
  cv::BFMatcher orb_matcher;
  // BRISK Descriptor
  int BRISK_Threshl;
  int BRISK_Octaves;// (pyramid layer) from which the keypoint has been extracted
  float BRISK_PatternScales;
  double matcherDistThreshold;
  cv::Ptr<cv::BRISK> ptrBrisk;
  cv::BFMatcher brisk_matcher;
  size_t maxMatch;
  // boder check
  float borderCheck;
  // LKT
  std::vector<int> prev_featIDset, cur_featIDset;
  cv::Size winSz;
  int pyramidLevel;
  int featID;
  double minEigVal;
  cv::TermCriteria TC;
  // re-extraction
  int reExtractNum;
  double y_alignmentThres;
  // visualization
  //double boxSize;
  // for next frame
  cv::Mat prev_img, prev_img_r, cur_img, cur_img_r;
  std::vector<cv::Point2f> prev_pts, prev_pts_r, cur_pts, cur_pts_r;
  cv::Mat prev_desp_l, prev_desp_r, cur_desp_l, cur_desp_r;
public:
  FrontEnd() : useCorner(true), useFAST(false), useORB(false), useBrisk(false), frame_id(1) {
    // image segment
    patchNum_r = Params::patch_num_r;
    patchNum_c = Params::patch_num_c;
    histogram_equalization = Params::equalize_histogram;
    // corner detection
    blockSize = 3;
    qualityLevel = 0.05;
    maxCorners = Params::num_corners;
    minDistance = Params::feature_extraction_boundary;
    useHarrisDetector = false;
    k = 0.06;
    // FAST detector
    fastThresh = 8;
    // ORB Descriptor
    ptrORB = cv::ORB::create();
    //ptrORB->setMaxFeatures(500);
		//ptrORB->setFastThreshold(20);
    orb_matcher = cv::BFMatcher(cv::NORM_HAMMING, false);
    // BRISK Descriptor
    BRISK_Threshl = 40;
    BRISK_Octaves = 3;
    BRISK_PatternScales = 1.0f;
    // BRISK descriptor initialize
    ptrBrisk = cv::BRISK::create(BRISK_Threshl, BRISK_Octaves, BRISK_PatternScales);
    brisk_matcher = cv::BFMatcher(cv::NORM_HAMMING, false);
    // matcher
    maxMatch = 200;
    matcherDistThreshold = Params::matcher_dist_threshold;
    // LKT
    winSz = Params::frame2frame_tracking_search_window;
    pyramidLevel = 5;
    minEigVal = 0.01;
    TC = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 500, 0.001);
    featID = 0;
    // re-extraction
    reExtractNum = Params::reextract_patch_num;
    y_alignmentThres = Params::max_disparity_alignment_threshold;
    // border check
    borderCheck = 5.;
    // visualization
    //boxSize = 20;
  }
  // detect and compute
  void detectCompute(cv::Mat& imL, cv::Mat& imR, cv::Mat& desp_l, cv::Mat& desp_r,
              std::vector<cv::KeyPoint>& keypoints_l, std::vector<cv::KeyPoint>& keypoints_r, bool r_extracted) {
    if (useBrisk) {
      ptrBrisk->detectAndCompute(imL, cv::Mat(), keypoints_l, desp_l);
      if (!r_extracted)
        ptrBrisk->detectAndCompute(imR, cv::Mat(), keypoints_r, desp_r);
    }
    else if (useORB) {
      ptrORB->detectAndCompute(imL, cv::Mat(), keypoints_l, desp_l);
      if (!r_extracted)
        ptrORB->detectAndCompute(imR, cv::Mat(), keypoints_r, desp_r);
    }
    else if (useCorner) {
      // left
      std::vector<cv::Point2f> corners_l;
      cv::goodFeaturesToTrack(imL, corners_l, maxCorners,
            qualityLevel,
            minDistance,
            cv::Mat(),
            blockSize,
            useHarrisDetector,
            k);
      for(auto it = corners_l.begin(); it != corners_l.end(); it++) {
          cv::KeyPoint kp(*it, 8);
          keypoints_l.push_back(kp);
      }
      ptrORB->compute(imL, keypoints_l, desp_l);
      // right
      if (!r_extracted) {
        std::vector<cv::Point2f> corners_r;
        cv::goodFeaturesToTrack(imR, corners_r, maxCorners,
              qualityLevel,
              minDistance,
              cv::Mat(),
              blockSize,
              useHarrisDetector,
              k);
        for(auto it = corners_r.begin(); it != corners_r.end(); it++) {
            cv::KeyPoint kp(*it, 8);
            keypoints_r.push_back(kp);
        }
        ptrORB->compute(imR, keypoints_r, desp_r);
      }
    }
    else if (useFAST) {
      // detector
      cv::FAST(imL, keypoints_l, fastThresh, true);
      cv::FAST(imR, keypoints_r, fastThresh, true);
      // calculate descriptor
      ptrORB->compute(imL, keypoints_l, desp_l);
      ptrORB->compute(imR, keypoints_r, desp_r);
    }
    return;
  }
  // extraction
  void extract(size_t y_offset, size_t x_offset, double interval_r, double interval_c,
               cv::Mat& imL, cv::Mat& imR, cv::Mat& imFullL, cv::Mat& imFullR,
               bool r_extracted, cv::Mat& desp_r_extracted, std::vector<cv::KeyPoint>& keypoints_r_extracted) {
    // initial extract L R
    cv::Mat desp_l, desp_r;
    std::vector<cv::KeyPoint> keypoints_l, keypoints_r = keypoints_r_extracted;
    // detect and compute
    detectCompute(imL, imR, desp_l, desp_r, keypoints_l, keypoints_r, r_extracted);
    if (keypoints_l.empty() || (keypoints_r.empty() /*&& !r_extracted*/) )
      return;
    // do match
    std::vector< cv::DMatch > matches;
    if (useBrisk) {
      if (r_extracted)
        brisk_matcher.match(desp_l, desp_r_extracted, matches);
      else
        brisk_matcher.match(desp_l, desp_r, matches);
    }
    else if (useORB || useCorner) {
      if (r_extracted)
        orb_matcher.match(desp_l, desp_r_extracted, matches);
      else
        orb_matcher.match(desp_l, desp_r, matches);
    }
    else if (useFAST) {
      if (r_extracted)
        fast_matcher.match(desp_l, desp_r_extracted, matches);
      else
        fast_matcher.match(desp_l, desp_r, matches);
    }
    // get points
    std::vector<cv::Point2f> new_pts, new_pts_r;
    cv::Mat new_desp, new_desp_r;
    for(size_t i=0; i<matches.size(); i++) {
      if ( matches[i].distance < matcherDistThreshold && i < maxMatch ) {
        cv::Point2f temp(keypoints_l[matches[i].queryIdx].pt);
        cv::Point2f temp_r(keypoints_r[matches[i].trainIdx].pt);
        // check y to filter out more 
        float y_dif = (float)temp.y - (float)temp_r.y; 
        if ( fabs(y_dif)>=y_alignmentThres )
          continue;
        temp.x = temp.x + x_offset*interval_c;
        temp.y = temp.y + y_offset*interval_r;
        temp_r.y = temp_r.y + y_offset*interval_r;
        new_pts.push_back(temp);
        new_pts_r.push_back(temp_r);
        new_desp.push_back(desp_l.row(matches[i].queryIdx));
        if (r_extracted)
          new_desp_r.push_back(desp_r_extracted.row(matches[i].trainIdx));
        else
          new_desp_r.push_back(desp_r.row(matches[i].trainIdx));
      }
    }
    // update status using border
    std::vector<uchar> status(new_pts.size(),1);
    for (size_t i=0; i<new_pts.size();i++ )
      if ( !inBBorder( new_pts[i], float(imFullL.cols), float(imFullL.rows), borderCheck ) ||
           !inBBorder( new_pts_r[i], float(imFullR.cols), float(imFullR.rows), borderCheck ) )
        status[i] = 0;
    // update vectors using status
    reduceVector(new_pts, status);
    reduceVector(new_pts_r, status);
    // insert in current point set
    cur_pts.insert( cur_pts.end(), new_pts.begin(), new_pts.end() );
    cur_pts_r.insert( cur_pts_r.end(), new_pts_r.begin(), new_pts_r.end() );
    for (size_t i=0; i<new_desp.rows; i++) {
      cur_desp_l.push_back(new_desp.row(i));
      cur_desp_r.push_back(new_desp_r.row(i));
    }
    // get feature ID
    for (size_t i = 0;i<new_pts.size();i++)
      cur_featIDset.push_back(featID++);
  }
  // check if reextract
  bool checkReExtract(std::vector< std::vector<bool> >& patchReExtract, const std::vector<cv::Point2f>& pts, cv::Size s) {
    double rows = (double)s.height;
    double cols = (double)s.width;
    double interval_r = rows/patchNum_r;
    double interval_c = cols/patchNum_c;
    // for every pts
    for (size_t i=0;i<pts.size();i++) {
      int idx_r = floor(pts[i].y/interval_r);
      int idx_c = floor(pts[i].x/interval_c);

      if ( idx_r==patchNum_r || idx_c==patchNum_c )
        std::cout << "In border check is too weak!" << std::endl;

      patchReExtract[idx_r][idx_c] = false;
    }
    // get return value
    unsigned reextract = 0;
    for (size_t i=0;i<patchReExtract.size();i++)
      for (size_t j=0;j<patchReExtract[i].size();j++)
        if (patchReExtract[i][j])
          reextract++;
    // re-extract only number of patches that are empty is greater or equal to reExtractNum
    if (reextract < reExtractNum ) 
      return false;
    else
      return true;
  }
  // aid LKT with feature match
  void featMatchAid(cv::Mat& cur_img, cv::Mat& cur_img_r, std::vector<uchar>& status, std::vector<uchar>& status_r) {
    // aiding LKT with feature descriptor matches
    clock_t start = clock();
    // fill in patch index map
    std::unordered_map< size_t, std::vector<size_t> > patchIndexMap, patchIndexMap_r;
    double rows = (double)cur_img.size().height;
    double cols = (double)cur_img.size().width;
    double interval_r = rows/patchNum_r;
    double interval_c = cols/patchNum_c;
    for (size_t i=0;i<prev_pts.size();i++) {
      int idx_r = floor(prev_pts[i].y/interval_r);
      int idx_c = floor(prev_pts[i].x/interval_c);
      patchIndexMap[idx_r*patchNum_c+idx_c].push_back(i);
    }
    for (size_t i=0;i<prev_pts_r.size();i++) {
      int idx_r = floor(prev_pts_r[i].y/interval_r);
      int idx_c = floor(prev_pts_r[i].x/interval_c);
      patchIndexMap_r[idx_r*patchNum_c+idx_c].push_back(i);
    }
    // FIRST: LEFT
    std::vector< std::vector<bool> > patchReExtract( patchNum_r, std::vector<bool>(patchNum_c, true) );
    bool reextract = checkReExtract(patchReExtract, cur_pts, cur_img.size() );
    // output
    std::vector<cv::Point2f> new_pts;
    cv::Mat new_desp;
    std::vector<int> new_ids;
    // start
    for (size_t i=0; i<patchReExtract.size(); i++) {
      for (size_t j=0; j<patchReExtract[i].size(); j++) {
        if ( patchReExtract[i][j] && !patchIndexMap[i*patchNum_c+j].empty() ) {
          //std::cout << "Feature based tracking (left) on patch at row " << i << " and column " << j << std::endl;
          // obtain the one to do feature based track
          std::vector<cv::Point2f> prevLeftTryBFM;
          cv::Mat prevLeftDespTryBFM;
          std::vector<size_t> prevLeftIDtryBFM;
          for (size_t z=0; z<patchIndexMap[i*patchNum_c+j].size(); z++) {
            size_t tempIdx = patchIndexMap[i*patchNum_c+j][z];
            if ( status[tempIdx]==1 )
              continue;
            prevLeftIDtryBFM.push_back(tempIdx);
            prevLeftTryBFM.push_back(prev_pts[tempIdx]);
            prevLeftDespTryBFM.push_back( prev_desp_l.row(tempIdx) );
          }
          // extract in the current left, do the feature match
          double rows = (double)cur_img.size().height;
          double cols = (double)cur_img.size().width;
          double interval_r = rows/patchNum_r;
          double interval_c = cols/patchNum_c;
          cv::Mat tempDespFB_l;
          std::vector<cv::KeyPoint> tempKPFB_l;
          // obtain cropped image, keypoints, and descriptors
          cv::Rect myROI_l(j*interval_c, i*interval_r, interval_c, interval_r);
          cv::Mat cropped_cur_img_l = cur_img(myROI_l);
          std::vector<cv::Point2f> corners_l;
          cv::goodFeaturesToTrack(cropped_cur_img_l, corners_l, maxCorners,
                qualityLevel,
                5,
                cv::Mat(),
                blockSize,
                useHarrisDetector,
                k);
          for(auto it = corners_l.begin(); it != corners_l.end(); it++) {
              cv::KeyPoint kp(*it, 8);
              tempKPFB_l.push_back(kp);
          }
          ptrORB->compute(cropped_cur_img_l, tempKPFB_l, tempDespFB_l);
          if (tempDespFB_l.rows == 0)
            continue;
          // do the matching
          std::vector< cv::DMatch > matches;
          orb_matcher.match(prevLeftDespTryBFM, tempDespFB_l, matches);
          //std::cout << "matching size: " << matches.size() << std::endl;
          // get points
          for(size_t i=0; i<matches.size(); i++) {
            if ( matches[i].distance < matcherDistThreshold && i < maxMatch ) {
              cv::Point2f temp(tempKPFB_l[matches[i].trainIdx].pt);
              temp.x = temp.x + i*interval_c;
              temp.y = temp.y + j*interval_r;
              new_pts.push_back(temp);
              new_desp.push_back(tempDespFB_l.row(matches[i].trainIdx));
              new_ids.push_back(prev_featIDset[ prevLeftIDtryBFM[matches[i].queryIdx] ]);
            }
          }
        }
      }
    }
    // SECOND: RIGHT
    std::vector< std::vector<bool> > patchReExtract_r( patchNum_r, std::vector<bool>(patchNum_c, true) );
    reextract = checkReExtract(patchReExtract_r, cur_pts_r, cur_img_r.size() );
    // output
    std::vector<cv::Point2f> new_pts_r;
    cv::Mat new_desp_r;
    std::vector<int> new_ids_r;
    // start
    for (size_t i=0; i<patchReExtract_r.size(); i++) {
      for (size_t j=0; j<patchReExtract_r[i].size(); j++) {
        if ( patchReExtract_r[i][j] && !patchIndexMap_r[i*patchNum_c+j].empty() ) {
          //std::cout << "Feature based tracking (right) on patch at row " << i << " and column " << j << std::endl;
          // obtain the one to do feature based track
          std::vector<cv::Point2f> prevRightTryBFM;
          cv::Mat prevRightDespTryBFM;
          std::vector<size_t> prevRightIDtryBFM;
          for (size_t z=0; z<patchIndexMap_r[i*patchNum_c+j].size(); z++) {
            size_t tempIdx = patchIndexMap_r[i*patchNum_c+j][z];
            if ( status_r[tempIdx]==1 )
              continue;
            prevRightIDtryBFM.push_back(tempIdx);
            prevRightTryBFM.push_back(prev_pts_r[tempIdx]);
            prevRightDespTryBFM.push_back(prev_desp_r.row(tempIdx));
          }
          // extract in the current left, do the feature match
          double rows = (double)cur_img_r.size().height;
          double cols = (double)cur_img_r.size().width;
          double interval_r = rows/patchNum_r;
          double interval_c = cols/patchNum_c;
          cv::Mat tempDespFB_r;
          std::vector<cv::KeyPoint> tempKPFB_r;
          // obtain cropped image, keypoints, and descriptors
          cv::Rect myROI_r(j*interval_c, i*interval_r, interval_c, interval_r);
          cv::Mat cropped_cur_img_r = cur_img_r(myROI_r);
          std::vector<cv::Point2f> corners_r;
          cv::goodFeaturesToTrack(cropped_cur_img_r, corners_r, maxCorners,
                qualityLevel,
                5,
                cv::Mat(),
                blockSize,
                useHarrisDetector,
                k);
          for(auto it = corners_r.begin(); it != corners_r.end(); it++) {
              cv::KeyPoint kp(*it, 8);
              tempKPFB_r.push_back(kp);
          }
          ptrORB->compute(cropped_cur_img_r, tempKPFB_r, tempDespFB_r);
          // no current point extracted
          if (tempDespFB_r.rows == 0)
            continue;
          // do the matching
          std::vector< cv::DMatch > matches;
          orb_matcher.match(prevRightDespTryBFM, tempDespFB_r, matches);
          // get points
          for(size_t i=0; i<matches.size(); i++) {
            if ( matches[i].distance < matcherDistThreshold && i < maxMatch ) {
              cv::Point2f temp(tempKPFB_r[matches[i].trainIdx].pt);
              temp.x = temp.x + i*interval_c;
              temp.y = temp.y + j*interval_r;
              new_pts_r.push_back(temp);
              new_desp_r.push_back(tempDespFB_r.row(matches[i].trainIdx));
              new_ids_r.push_back(prev_featIDset[ prevRightIDtryBFM[matches[i].queryIdx] ]);
            }
          }
        }
      }
    }
    size_t aided = 0;
    // working on new_pts and new_pts_r
    for (size_t i=0; i<new_pts.size(); i++) {
      for (size_t j=0; j<new_pts_r.size(); j++) {
        if (new_ids[i] == new_ids_r[j]) {
          cur_pts.push_back(new_pts[i]);
          cur_pts_r.push_back(new_pts_r[j]);
          cur_desp_l.push_back(new_desp.row(i));
          cur_desp_r.push_back(new_desp_r.row(j));
          aided++;
          cur_featIDset.push_back(new_ids[i]);
        }
      }
    }
    // to filter out
    float diff = ((float)clock()-(float)start)/CLOCKS_PER_SEC;
    std::cout << "feature based tracking done " << aided << " added takes " << diff*1.0e3 << "ms\n\n";
  }
  // curr_pair callback
  std::shared_ptr<VisualFrontend::FMatches> frame_callback(std::shared_ptr<VisualFrontend::StereoPair> curr_pair) {
    // get time
    clock_t start_fcb(clock());
    // grayscale
    cv::Mat grayL, grayR;
    // cv::cvtColor( curr_pair->get_left()->img, cur_img, CV_RGB2GRAY );
    // cv::cvtColor( curr_pair->get_right()->img, cur_img_r, CV_RGB2GRAY );
    cv::cvtColor( curr_pair->get_left()->img, grayL, CV_RGB2GRAY );
    cv::cvtColor( curr_pair->get_right()->img, grayR, CV_RGB2GRAY );

    cur_img = grayL.clone();
    cur_img_r = grayR.clone();

    float diff = -((float)start_fcb-(float)clock())/CLOCKS_PER_SEC;
    //std::cout << "finish grayscale time: " << diff*1.0e3 << "ms\n";
    // resize
    // cv::resize(cur_img, cur_img, cv::Size(cur_img.cols*0.5, cur_img.rows*0.5));
    // cv::resize(cur_img_r, cur_img_r, cv::Size(cur_img_r.cols*0.5, cur_img_r.rows*0.5));
    diff = -((float)start_fcb-(float)clock())/CLOCKS_PER_SEC;
    std::cout << "finish resize time: " << diff*1.0e3 << "ms\n";

    clock_t he(clock());
    if (histogram_equalization) {
      // histogram equlization
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(10, 10));
      clahe->apply(cur_img, cur_img);
      clahe->apply(cur_img_r, cur_img_r);
    }
    diff = -((float)he-(float)clock())/CLOCKS_PER_SEC;
    std::cout << "histogram equlization time: " << diff*1.0e3 << "ms\n";

    // first frame extraction
    if (prev_pts.empty()) {
      clock_t start(clock());
      // extract and push in cur_pts, cur_pts_r, cur_featIDset.
      cv::Mat desp_r_idle;
      std::vector<cv::KeyPoint> kp_r_idle;
      extract(0, 0, 0, 0, cur_img, cur_img_r, cur_img, cur_img_r, false, desp_r_idle, kp_r_idle);
      diff =  -((float)start-(float)clock())/CLOCKS_PER_SEC;
      std::cout << "fist frame extraction time: " << diff*1.0e3 << "ms\n";
    }
    // frame to frame tracking
    else 
    {
      // track first
      std::vector<uchar> status, status_r;
      std::vector<float> err;

      clock_t start = clock();
      // track L R
      cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err,
                winSz, pyramidLevel, TC, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, minEigVal);
      cv::calcOpticalFlowPyrLK(prev_img_r, cur_img_r, prev_pts_r, cur_pts_r, status_r, err,
                winSz, pyramidLevel, TC, cv::OPTFLOW_LK_GET_MIN_EIGENVALS, minEigVal);


      // assert(prev_img == grayL);

      // std::cout << "prev pts" << std::endl;
      // for (int i=0; i<prev_pts.size();i++)
      // {
      //   std::cout << "i" << prev_pts[i] << std::endl;
      // }

      // std::cout << "cur pts" << std::endl;
      // for (int i=0; i<cur_pts.size();i++)
      // {
      //   std::cout << "i" << cur_pts[i] << std::endl;

      // }

      diff = -((float)start-(float)clock())/CLOCKS_PER_SEC;
      //std::cout << "track both frame time: " << diff*1.0e3 << "ms\n"; 

      // update status using border
      for (size_t i=0; i<cur_pts.size();i++ ) {
        status[i] = (int)status[i] & (int)status_r[i];
        if (status[i] && (
          !inBBorder( cur_pts[i], float(cur_img.cols), float(cur_img.rows), borderCheck ) ||
          !inBBorder( cur_pts_r[i], float(cur_img_r.cols), float(cur_img_r.rows), borderCheck ) )
          )
          status[i] = 0;
      }
      // update vectors using status
      reduceVector(cur_pts, status);
      reduceVector(cur_pts_r, status);
      reduceVector(cur_featIDset, status);
      reduceMat(cur_desp_l, status);
      reduceMat(cur_desp_r, status);
      // aid LKT with feature match
      if (false) {
        featMatchAid(cur_img, cur_img_r, status, status_r);
      }
      // check reextraction
      std::vector< std::vector<bool> > patchReExtract( patchNum_r, std::vector<bool>(patchNum_c, true) );
      bool reextract = checkReExtract(patchReExtract, cur_pts, cur_img.size() );
      // start to reextract
      if (reextract) {
        int ct=0;
        double interval_r = double(cur_img.size().height)/patchNum_r;
        double interval_c = double(cur_img.size().width)/patchNum_c;
        for (size_t i=0;i<patchReExtract.size();i++) { // patchNum_r
          // check if this strip is used
          bool this_strip_used = false;
          for (size_t j=0;j<patchReExtract[i].size();j++)
            if (patchReExtract[i][j]) {
              this_strip_used = true;
              break;
            }
          // this strip is not used, all patches are good
          if (!this_strip_used)
            continue;
          start = clock();
          // extract for the strip first.
          cv::Rect myROI_r(0, i*interval_r, cur_img_r.size().width, interval_r);
          cv::Mat cropped_cur_img_r = cur_img_r(myROI_r);          
          cv::Mat desp_r;
          std::vector<cv::KeyPoint> keypoints_r;
          std::vector<cv::Point2f> corners_r;
          cv::goodFeaturesToTrack(cropped_cur_img_r, corners_r, maxCorners,
                qualityLevel,
                minDistance,
                cv::Mat(),
                blockSize,
                useHarrisDetector,
                k);
          for(auto it = corners_r.begin(); it != corners_r.end(); it++) {
              cv::KeyPoint kp(*it, 8);
              keypoints_r.push_back(kp);
          }
          ptrORB->compute(cropped_cur_img_r, keypoints_r, desp_r);
          // no keypoints extracted for the strip
          if (keypoints_r.empty())
            continue;

          diff = ((float)clock()-(float)start)/CLOCKS_PER_SEC;
          //std::cout << "reextract the strip time is " << diff*1.0e3 << "ms\n";
          // start on patches corresponds to this strip
          for (size_t j=0;j<patchReExtract[i].size();j++) { // patchNum_c
            //std::cout << "status on patch at x: " << j << " and y: " << i << ": " << patchReExtract[i][j] << std::endl;
            if (patchReExtract[i][j]) {
              
              start = clock();
              
              // obtain cropped image
              cv::Rect myROI(j*interval_c, i*interval_r, interval_c, interval_r);
              cv::Mat cropped_cur_img = cur_img(myROI);
              // extract and push in cur_pts, cur_pts_r, cur_featIDset.
              extract(i, j, interval_r, interval_c, cropped_cur_img, cropped_cur_img_r, cur_img, cur_img_r,
                        true, desp_r, keypoints_r);
              
              diff = ((float)clock()-(float)start)/CLOCKS_PER_SEC;
              std::cout << "reextract " << ct++ << " time is " << diff*1.0e3 << "ms\n";
            }
          }
        }
        std::cout << "Re-extract sz: " << ct << "\n"; 
      }
    }
    
    // one more filtering using y value
    std::vector<uchar> status(cur_pts.size(), 1);
    for ( size_t i=0; i<cur_pts.size(); i++ )
      if ( fabs(cur_pts[i].y-cur_pts_r[i].y)>=y_alignmentThres )
        status[i] = 0;
    reduceVector(cur_pts, status);
    reduceVector(cur_pts_r, status);
    reduceVector(cur_featIDset, status);
    reduceMat(cur_desp_l, status);
    reduceMat(cur_desp_r, status);

    // clock_t epip(clock());
    // // check Epipolar constraint
    // cv::Mat Mask;
    // cv::Mat E = cv::findEssentialMat(cur_pts, cur_pts_r, 498.1357145, 
    //                   cv::Point2d(351.726944, 255.9642885), cv::FM_LMEDS, 0.999, 0.01, Mask); // cv::FM_LMEDS

    // cv::Mat R1, R2 ,t;
    // cv::decomposeEssentialMat(E, R1, R2, t);
    
    // diff = -((float)epip-(float)clock())/CLOCKS_PER_SEC;
    // std::cout << "Find E time: " << diff*1.0e3 << "ms\n";

    // std::cout << "t: " << t << "\n";
    // std::cout << "Previous pt size: " << cur_pts.size() << "\n";

    // std::vector<cv::Point2f> E_verified_cur_pts, E_verified_cur_pts_r;

    // // std::cout << "mask: "<< Mask.rows << " " << Mask.cols << " " << Mask.type() << "\n";
    // // std::cout << Mask << "\n";

    // for (uint i=0; i<Mask.rows; i++) {
    //   if ( static_cast<unsigned>( Mask.at<uchar>(i,0) ) == 1 ) {
    //     E_verified_cur_pts.push_back(cur_pts[i]);
    //     E_verified_cur_pts_r.push_back(cur_pts_r[i]);
    //   }
    // }
    // //std::cout << "Mask: " << Mask << "\n";
    // std::cout << "Current pt size: " << E_verified_cur_pts.size() << "\n";
    // cur_pts = E_verified_cur_pts;
    // cur_pts_r = E_verified_cur_pts_r;

    // pubish
    std::shared_ptr<VisualFrontend::FMatches> matches = std::make_shared<VisualFrontend::FMatches>();
    matches->frame1 = curr_pair->get_left();
    matches->frame2 = curr_pair->get_right();
    for ( size_t i=0;i<cur_pts.size();i++ ) {
      std::shared_ptr<VisualFrontend::FMatch> fmatch(new VisualFrontend::FMatch);
      fmatch->f1 = std::shared_ptr<VisualFrontend::Feature>(new VisualFrontend::Feature);
      fmatch->f2 = std::shared_ptr<VisualFrontend::Feature>(new VisualFrontend::Feature);
      fmatch->f1->id = cur_featIDset[i];
      fmatch->f1->lpix = cur_pts[i];
      fmatch->f2->id = cur_featIDset[i];
      fmatch->f2->lpix = cur_pts_r[i];
      matches->matches.push_back(fmatch);
    }

    // prepare for next
    prev_img = cur_img;
    prev_img_r = cur_img_r;
    prev_pts = cur_pts;
    prev_pts_r = cur_pts_r;
    prev_featIDset = cur_featIDset;
    prev_desp_l = cur_desp_l.clone();
    prev_desp_r = cur_desp_r.clone();
    cur_pts.clear();
    cur_pts_r.clear();
    return matches;
  }
};
