#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  if ((estimations.size()==0)|(estimations.size()!=ground_truth.size())){
  std::cout << "Error! Invalid inputs to calculate rmse" <<std::endl;
    return rmse;
  }
  for (int i=0;i<ground_truth.size();++i){
  	VectorXd err= (estimations[i]-ground_truth[i]);
    err=err.array()*err.array();
    rmse+=err;
  }
  
  rmse=rmse/(ground_truth.size());
  rmse=rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  float den2=(px*px+py*py);
  
  if(den2< 0.01){
    std::cout<<"den2 is very low!"<<std::endl;
    den2=1e-4;
    Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return Hj;
  }
  
  float den1=std::sqrt(den2);
  float den3=den1*den2;
  
  Hj << px/den1,py/den1,0,0,
  		-py/den2,px/den2,0,0,
  		py*(vx*py-vy*px)/den3,px*(vy*px-vx*py)/den3,px/den1,py/den1;
  
  return Hj;
}
