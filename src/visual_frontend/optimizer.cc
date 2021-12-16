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
#include <visual_frontend/optimizer.h>
#include <visual_frontend/reprojection_fns.h>

#include <chrono>
#include <stdlib.h>


namespace Optimizer {

  LevenbergMarquardt::LevenbergMarquardt(VisualFrontend::DepthFeatures &depth_features,
					 VisualFrontend::NoDepthFeatures &nodepth_features,
					 double imu_weight):
    lambda(0.001),
    _num_depth_eqns(depth_features.size()*2),
    _num_nodepth_eqns(nodepth_features.size()),
    // the plus three is because of imu
    _num_eqns(_num_depth_eqns + _num_nodepth_eqns + 3), 
    _imu_weight(imu_weight),

    _residuals(_num_eqns), //residual vector
    _jacobians(_num_eqns,6), //jacobian matrix

    _depth_features(depth_features), //depth features vector
    //flag vector to show if depth eqn 1 is included
    _depth1_features_flag(depth_features.size()),
    //flag vector to show if depth eqn 2 is included
    _depth2_features_flag(depth_features.size()),
    //vector - stores the weight of depth eqn 1
    _depth1_features_weight(depth_features.size()),
    //vector - stores the weight of depth eqn 2
    _depth2_features_weight(depth_features.size()), 

    //no depth features vector
    _nodepth_features(nodepth_features),
    //flag vector to show if nodepth eqn is included
    _nodepth_features_flag(nodepth_features.size()),
    //vector - stores the weight * scale_factor for no depth eqn
    _nodepth_features_weight(nodepth_features.size()), 

    //maximum number of iterations to run
    _max_iterations(100), 
    _min_error(1e-5), //minimum precision

    _bisquare_C_depth(0.6),
    _bisquare_C_nodepth(0.001),
    //flag which determines if bisquare weight is calculated adaptively or not
    _bisquare_C_adaptive(true), 

    //scale factor used to weight the no depth equations more
    _nodepth_scale_factor(10), 
    _max_duration(1./25) //in seconds (20hz)
  {
    //set the flag to 1 so that we are including all the features
    //set the weight to 1, we will update it later
    for(int i=0;i<_depth1_features_flag.size();i++) {
      _depth1_features_flag(i) = 1;
      _depth2_features_flag(i) = 1;
      _depth1_features_weight(i) = 1;
      _depth2_features_weight(i) = 1;

    }
    //do the same of no depth equations
    for(int i=0;i<_nodepth_features_flag.size();i++) {
      _nodepth_features_flag(i) = 1;
      _nodepth_features_weight(i) = 1;
    }

  }

  Result LevenbergMarquardt::Solve(Eigen::Matrix4d guess) {
    
    _imu_rot = homogeneousToVect(guess).block<3,1>(3,0);
    _currX = homogeneousToVect(guess);

    //if _currX is all zeros, set some small value to prevent it from crashing
    perturb(_currX);
    
    int iter = 0;
    
    double prev_error, curr_error;
    Vector6d prev_delta, curr_delta;

    //start the timers
    auto start = std::chrono::high_resolution_clock::now();

    //calculate the bisquare-weight-constant adaptively if the flag is set
    if(_bisquare_C_adaptive) {
      FindAdaptiveBisquareWeightConstant(_currX);
    }

    //predeclare the matrices so that the iterations are faster
    Eigen::MatrixXd JtJ;
    Eigen::VectorXd diag;
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> diagonal;
    Eigen::MatrixXd temp1;
    Eigen::MatrixXd temp2;

    //set up optimization loop
    while(iter < _max_iterations) {

      iter++;

      //std::cout << "Solving !!!" << std::endl;

      //now fill the residuals and jacobians
      FillResiduals(_currX);
      FillJacobians(_currX);

      double curr_error = Error();
      //std::cout << "Current error: " << curr_error << std::endl;

      JtJ = _jacobians.transpose()*_jacobians;
      diag = JtJ.diagonal();
      diagonal.diagonal() = diag;
      temp1 = lambda * diagonal;
      temp2 = JtJ + temp1;

      //LM optimization is of the form
      // x <- x - (J'J + lambda * diag(J'J))^-1 J'residuals
      curr_delta = temp2.inverse()*_jacobians.transpose()*_residuals;
 
      if(iter == 1) {
	lambda = 1e-3 * diag.mean();
	
	//for the first iteration we just accept the update
	_currX = _currX - curr_delta;

      } else {
	if(curr_error > prev_error) {

	  //current error is larger, lets get aggressive and set lambda larger
	  lambda = lambda * 10;
	  //std::cout << "Increasing lambda to :" << lambda << std::endl;

	  //and ignore this update
	} else {
	  
	  //current error is smaller, lets scale back lambda
	  lambda = lambda / 10;
	  //std::cout << "Decreasing lambda to :" << lambda << std::endl;

	  //and accept this udpate
	  _currX = _currX - curr_delta;
	}

	//exit criteria
	if(fabs(curr_error - prev_error) < _min_error) {
	  //std::cout << "Error not changing much." << std::endl;
	  break;
	}
      }

      //let update the prev pointers
      prev_error = curr_error;
      prev_delta = curr_delta;

      auto stop = std::chrono::high_resolution_clock::now();
      double lapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>
	(stop-start).count();
      if(lapsed_time >= _max_duration*1e3) {
	break;
      }
      
    }
    
    auto finish = std::chrono::high_resolution_clock::now();

    //fill the result structure
    Result result;
    result.T__k__k_1 = vectToHomogeneous(_currX);
    result.duration = std::chrono::duration_cast<std::chrono::milliseconds>(finish-start).count();
    if(iter >= _max_iterations || result.duration >= _max_duration*1e3) {
      //TODO:Uncomment
      // std::cerr << "LM did not converge. " <<
      // 	"Lapsed:" << result.duration.toSec() << "s Max:" << _max_duration << "s" <<
      // 	" Iterations:" << iter << " Max iterations:" << _max_iterations << std::endl;
    
      result.converged = false;
    } else {
      result.converged = true;
    }
    result.iterations = iter;
    result.active_xdepth_features = _depth1_features_flag.sum();
    result.active_ydepth_features = _depth2_features_flag.sum();
    result.active_nodepth_features = _nodepth_features_flag.sum();

    result.bisquare_C_depth = _bisquare_C_depth;
    result.bisquare_C_nodepth = _bisquare_C_nodepth;
    return result;
  }
  
  void LevenbergMarquardt::FillResiduals(Vector6d &x) {
    
    int res_counter = 0;
    int feature_counter = 0;

    double residual1,residual2;
    double weight1, weight2;

    //residuals for depth expressions
    for(auto &ent : _depth_features) {

      //if this feature is active
      if(_depth1_features_flag(feature_counter) != 0) {
	//find the residuals and weight
	residual1 = reprojectionFns::DepthExp1(x(0),x(1),x(2),x(3),x(4),x(5),
					       ent.wo_d_k_1[0], ent.wo_d_k_1[1],
					       ent.w_d_k_1[0],ent.w_d_k_1[1],ent.w_d_k_1[2],
					       ent.wo_d_k[0],ent.wo_d_k[1]);
	weight1 = bisquareWeight(residual1, _bisquare_C_depth);
	_depth1_features_weight(feature_counter) = weight1;

	//set inactive if the weight is zero
	if(weight1 == 0) {
	  _depth1_features_flag(feature_counter) = 0;
	}
	//update the residual (will be zero if weight turned out to be zero
	_residuals[res_counter++] = residual1 * weight1;

	//if the feature is not active, set it to 0
      } else {
	_residuals[res_counter++] = 0;
      }

      //do the same thing for the second expression
      //if this feature is active
      if(_depth2_features_flag(feature_counter) != 0) {
	//find the residuals and weight
	residual2 = reprojectionFns::DepthExp2(x(0),x(1),x(2),x(3),x(4),x(5),
					       ent.wo_d_k_1[0], ent.wo_d_k_1[1],
					       ent.w_d_k_1[0],ent.w_d_k_1[1],ent.w_d_k_1[2],
					       ent.wo_d_k[0],ent.wo_d_k[1]);
	weight2 = bisquareWeight(residual2, _bisquare_C_depth);
	_depth2_features_weight(feature_counter) = weight2;

	//set inactive if the weight is zero
	if(weight2 == 0) {
	  _depth2_features_flag(feature_counter) = 0;
	}
	//update the residual (will be zero if weight turned out to be zero
	_residuals[res_counter++] = residual2 * weight2;

	//if the feature is not active, set it to 0
      } else {
	_residuals[res_counter++] = 0;
      }

      //now increment the feature counter
      feature_counter++;
    }
    
     
    double residual;
    double weight;
    feature_counter = 0;

    //residuals for nodepth expressions
    for(auto &ent : _nodepth_features) {
      
      //if this feature is active
      if(_nodepth_features_flag(feature_counter) != 0) {
	//find the residuals and weight
	residual = reprojectionFns::NoDepthExp1(x(0),x(1),x(2),x(3),x(4),x(5),
						ent.wo_d_k_1[0], ent.wo_d_k_1[1],
						ent.wo_d_k[0],ent.wo_d_k[1]);
	weight = _nodepth_scale_factor * bisquareWeight(residual, _bisquare_C_nodepth);
	_nodepth_features_weight(feature_counter) = weight;

	//set inactive if the weight is zero
	if(weight == 0) {
	  _nodepth_features_flag(feature_counter) = 0;
	}
	//update the residual (will be zero if weight turned out to be zero
	_residuals[res_counter++] = residual * weight;

	//if the feature is not active, set it to 0
      } else {
	_residuals[res_counter++] = 0;
      }
      feature_counter++;
    }      
    _residuals[res_counter++] = _imu_weight * (x[3] - _imu_rot[0]);
    _residuals[res_counter++] = _imu_weight * (x[4] - _imu_rot[1]);
    _residuals[res_counter++] = _imu_weight * (x[5] - _imu_rot[2]);
  }

  void LevenbergMarquardt::FillJacobians(Vector6d &x) {

    std::vector<double> jd1, jd2, jnd1;
    int res_counter = 0;
    int feature_counter = 0;

    for(auto &ent : _depth_features) {

      if(_depth1_features_flag(feature_counter) != 0) {
	jd1 = reprojectionFns::DepthExp1Jacobian(x(0),x(1),x(2),x(3),x(4),x(5),
						 ent.wo_d_k_1[0], ent.wo_d_k_1[1],
						 ent.w_d_k_1[0],ent.w_d_k_1[1],ent.w_d_k_1[2],
						 ent.wo_d_k[0],ent.wo_d_k[1]);
	double w = _depth1_features_weight(feature_counter);
	_jacobians.block<1,6>(res_counter++,0) << w*jd1[0], w*jd1[1], w*jd1[2], w*jd1[3], w*jd1[4], w*jd1[5];
           
      } else {
	_jacobians.block<1,6>(res_counter++,0) << 0, 0, 0, 0, 0, 0;
      }

      if(_depth2_features_flag(feature_counter) != 0) {
	jd2 = reprojectionFns::DepthExp2Jacobian(x(0),x(1),x(2),x(3),x(4),x(5),
						 ent.wo_d_k_1[0], ent.wo_d_k_1[1],
						 ent.w_d_k_1[0],ent.w_d_k_1[1],ent.w_d_k_1[2],
						 ent.wo_d_k[0],ent.wo_d_k[1]);
	double w = _depth2_features_weight(feature_counter);
	_jacobians.block<1,6>(res_counter++,0) << w*jd2[0], w*jd2[1], w*jd2[2], w*jd2[3], w*jd2[4], w*jd2[5];
      
      } else {
	_jacobians.block<1,6>(res_counter++,0) << 0, 0, 0, 0, 0, 0;
      }

      feature_counter++;
    }

    feature_counter = 0;

    for(auto &ent : _nodepth_features) {
      
      if(_nodepth_features_flag(feature_counter) != 0) {
	jnd1 = reprojectionFns::NoDepthExp1Jacobian(x(0),x(1),x(2),x(3),x(4),x(5),
						    ent.wo_d_k_1[0], ent.wo_d_k_1[1],
						    ent.wo_d_k[0],ent.wo_d_k[1]);
	double w = _nodepth_features_weight(feature_counter);
	_jacobians.block<1,6>(res_counter++,0) << w*jnd1[0], w*jnd1[1], w*jnd1[2], w*jnd1[3], w*jnd1[4], w*jnd1[5];

      } else {
	_jacobians.block<1,6>(res_counter++,0) << 0, 0, 0, 0, 0, 0;
      }
      feature_counter++;
    } 

    // finally, add the entries for imu term
    _jacobians.block<1,6>(res_counter++,0) << 0, 0, 0, _imu_weight, 0, 0;
    _jacobians.block<1,6>(res_counter++,0) << 0, 0, 0, 0, _imu_weight, 0;
    _jacobians.block<1,6>(res_counter++,0) << 0, 0, 0, 0, 0, _imu_weight;
  }
  
  void LevenbergMarquardt::FindAdaptiveBisquareWeightConstant(Vector6d &x) {

    //for depth expressions
    int res_counter = 0;
    Eigen::VectorXd depth_residuals(_num_depth_eqns);

    for(auto &ent : _depth_features) {
      
      depth_residuals[res_counter++] = reprojectionFns::DepthExp1(x(0),x(1),x(2),x(3),x(4),x(5),
								 ent.wo_d_k_1[0], ent.wo_d_k_1[1],
								 ent.w_d_k_1[0],ent.w_d_k_1[1],ent.w_d_k_1[2],
								 ent.wo_d_k[0],ent.wo_d_k[1]);
      depth_residuals[res_counter++] = reprojectionFns::DepthExp2(x(0),x(1),x(2),x(3),x(4),x(5),
								 ent.wo_d_k_1[0], ent.wo_d_k_1[1],
								 ent.w_d_k_1[0],ent.w_d_k_1[1],ent.w_d_k_1[2],
								 ent.wo_d_k[0],ent.wo_d_k[1]);
    }
    
    _bisquare_C_depth = depth_residuals.mean() + stdDeviation(depth_residuals);

    //for no depth expressions
    res_counter = 0;
    Eigen::VectorXd nodepth_residuals(_num_nodepth_eqns);

    for(auto &ent : _nodepth_features) {

      nodepth_residuals[res_counter++] = reprojectionFns::NoDepthExp1(x(0),x(1),x(2),x(3),x(4),x(5),
								     ent.wo_d_k_1[0], ent.wo_d_k_1[1],
								     ent.wo_d_k[0],ent.wo_d_k[1]);
    }
    
    _bisquare_C_nodepth = nodepth_residuals.mean() + stdDeviation(nodepth_residuals);
    
  }

  double LevenbergMarquardt::Error() {
    return _residuals.norm();
  }			  

  Eigen::Matrix4d vectToHomogeneous(Vector6d &x) {

    //set the translation part
    Eigen::Matrix4d H;
    H(0,3) = x[0];
    H(1,3) = x[1];
    H(2,3) = x[2];

    //set rotation
    Eigen::Matrix3d rot;
    rot = (Eigen::AngleAxisd(x[3],Eigen::Vector3d::UnitX()) *
	   Eigen::AngleAxisd(x[4],Eigen::Vector3d::UnitY()) *
	   Eigen::AngleAxisd(x[5],Eigen::Vector3d::UnitZ()));
    H.block(0,0,3,3) = rot;
    
    return H;
    
  }

  double bisquareWeight(double residual, double C) {
    double weight;
    if(C == 0) { 
      weight = 1;
    } else {
      if(fabs(residual) <= C) {
	weight = pow(1 - pow(residual/C,2),2);
      } else {
	weight = 0;
      }
    }
    return weight;
  }

  double stdDeviation(Eigen::VectorXd &x) {
    
    double m = x.mean();
    double sum = 0;
    for(int i=0;i<x.size();i++) {
      sum += pow(x(i) - m,2);
    }
    
    double stdDev = sqrt(sum/(x.size()-1));
    return stdDev;
  }

  Vector6d homogeneousToVect(Eigen::Matrix4d H) {

    
    Eigen::Matrix3d rot(H.block(0,0,3,3));
    Eigen::Vector3d ypr = rot.eulerAngles(2,1,0);
    // std::cout << "ypr:" << ypr << std::endl;
    Eigen::AngleAxisd ax = Eigen::AngleAxisd(ypr[0],Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ay = Eigen::AngleAxisd(ypr[1],Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd az = Eigen::AngleAxisd(ypr[2],Eigen::Vector3d::UnitZ());

    // std::cout << "Angle axis: ax" << ax.angle() << " ay:" << ay.angle()
    // 	      << " az:" << az.angle() << std::endl;

    Vector6d vect;
    vect << H(0,3), H(1,3), H(2,3), ax.angle(), ay.angle(), az.angle();
    // std::cout << "Vect: x:" << vect[0] << " y:" << vect[1] << " z:" << vect[2] <<
    //   " ax:" << vect[3] << " ay:" << vect[4] << " az:" << vect[5] << std::endl;
    return vect;
  }

  double mean(double *x, int num) {
    double sum=0;
    for(int i=0;i<num;i++) {
      sum += x[i];
    }
    return sum/num;
  }

  void perturb(Vector6d &x) {

    if(fabs(x[0]) < 1e-10 && fabs(x[1]) < 1e-10 && fabs(x[2]) < 1e-10 &&
       fabs(x[3]) < 1e-10 && fabs(x[4]) < 1e-10 && fabs(x[5]) < 1e-10) {
      std::cout << "Perturbing the initial guess as the numbers are all zeros" << std::endl;
      x[0] = (rand() % 1000)*1./1e6; //a number between 0.00001 to 0.001
      x[1] = (rand() % 1000)*1./1e6;
      x[2] = (rand() % 1000)*1./1e6;
      x[3] = (rand() % 1000)*1./1e6;
      x[4] = (rand() % 1000)*1./1e6;
      x[5] = (rand() % 1000)*1./1e6;
      std::cout << "Perturbed values: " << x << std::endl;
    }
  } 
    
       
};
