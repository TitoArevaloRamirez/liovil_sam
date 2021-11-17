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
#ifndef __OPTIMIZER_H__
#define __OPTIMIZER_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "feature_operations.h"

namespace Optimizer {
  
  struct Result {
  public:
    std::string summary;
    Eigen::Matrix4d T__k__k_1;
    bool converged;
    double duration;
    int iterations;
    int active_xdepth_features;
    int active_ydepth_features;
    int active_nodepth_features;
    double bisquare_C_depth;
    double bisquare_C_nodepth;
  };
  
  typedef Eigen::Matrix<double, 6, 1> Vector6d;

  class LevenbergMarquardt {
    
  public:
    LevenbergMarquardt(VisualFrontend::DepthFeatures &depth_features,
		       VisualFrontend::NoDepthFeatures &nodepth_features,
		       double imu_weight);
    
    Result Solve(Eigen::Matrix4d guess=Eigen::Matrix4d::Identity());
    
    //Helper functions
    void FillResiduals(Vector6d &x);
    void FillJacobians(Vector6d &x);
    void FindAdaptiveBisquareWeightConstant(Vector6d &x);
    double Error();
    
  protected:
    double lambda;
    int _num_depth_eqns;
    int _num_nodepth_eqns;
    int _num_eqns;
    Eigen::VectorXd _residuals;
    Eigen::MatrixXd _jacobians;
    VisualFrontend::DepthFeatures _depth_features;
    Eigen::VectorXd _depth1_features_flag;
    Eigen::VectorXd _depth2_features_flag;
    Eigen::VectorXd _depth1_features_weight;
    Eigen::VectorXd _depth2_features_weight;
    Eigen::Vector3d _imu_rot;
    VisualFrontend::NoDepthFeatures _nodepth_features;
    Eigen::VectorXd _nodepth_features_flag;
    Eigen::VectorXd _nodepth_features_weight;
    Vector6d _currX;
    int _max_iterations;
    double _min_error;
    double _bisquare_C_depth;
    double _bisquare_C_nodepth;
    bool _bisquare_C_adaptive;
    double _nodepth_scale_factor;
    double _imu_weight;
    double _max_duration;
  };

  Eigen::Matrix4d vectToHomogeneous(Vector6d &x);

  double bisquareWeight(double residual,double C);

  double stdDeviation(Eigen::VectorXd &x);

  Vector6d homogeneousToVect(Eigen::Matrix4d H);

  double mean(double *x, int num);

  void perturb(Vector6d &x);
};

#endif // __OPTIMIZER_H__
