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
#ifndef __REPROJECTION_FNS_H__
#define __REPROJECTION_FNS_H__

#include <iostream>
#include <vector>

namespace reprojectionFns {
  double DepthExp1(double T1, double T2, double T3, 
			  double tx, double ty, double tz, 
			  double xbk1i, double ybk1i, 
			  double xk1i, double yk1i, double zk1i, 
			  double xbki, double ybki);

  double DepthExp2(double T1, double T2, double T3, 
			  double tx, double ty, double tz, 
			  double xbk1i, double ybk1i, 
			  double xk1i, double yk1i, double zk1i, 
			  double xbki, double ybki);
  
  double NoDepthExp1(double T1, double T2, double T3, 
			    double tx, double ty, double tz, 
			    double xbk1i, double ybk1i, 
			    double xbki, double ybki);

  std::vector<double> DepthExp1Jacobian(double T1, double T2, double T3, 
					       double tx, double ty, double tz, 
					       double xbk1i, double ybk1i, 
					       double xk1i, double yk1i, double zk1i, 
					       double xbki, double ybki);
  
  std::vector<double> DepthExp2Jacobian(double T1, double T2, double T3, 
					       double tx, double ty, double tz, 
					       double xbk1i, double ybk1i, 
					       double xk1i, double yk1i, double zk1i, 
					       double xbki, double ybki);

  std::vector<double> NoDepthExp1Jacobian(double T1, double T2, double T3, 
						 double tx, double ty, double tz, 
						 double xbk1i, double ybk1i, 
						 double xbki, double ybki);
};

#endif //__REPROJECTION_FNS_H__

