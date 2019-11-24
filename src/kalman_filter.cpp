#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &Q_in) {
  /*
                        MatrixXd &H_in, MatrixXd &R_in, 
  */
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;

//   H_ = H_in;
//   R_ = R_in;
//   Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  //std::cout<< "Predicting"<<std::endl;
  x_=F_*x_;
  P_=F_*P_*F_.transpose()+Q_;
  //std::cout<< "Prediction finished"<<std::endl;
}

void KalmanFilter::Update(const VectorXd &z, const MatrixXd &H, const MatrixXd &R) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z-H*x_;
  MatrixXd S_ = H*P_*H.transpose()+R;
  MatrixXd K = P_*H.transpose()*S_.inverse();
  x_=x_+K*y;
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_=(I-K*H)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, const MatrixXd &R, const MatrixXd &Hj) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px=x_(0);
  float py=x_(1);
  float vx=x_(2);
  float vy=x_(3);
  float mag=std::sqrt(px*px+py*py);
  float angle=std::atan2(py,px);
  if (mag<0.1){
    std::cout<< "Very low mag found"<<std::endl;
    mag=0.1;
  }
  
  //mag= (mag<1e-4)? 1e-4:mag; //cap minimum value of mag to 1e-4
  
  VectorXd hxp(3);
  hxp<< mag, angle, (px*vx+py*vy)/mag;
  
  VectorXd y = z-hxp;
  MatrixXd S_ = Hj*P_*Hj.transpose()+R;
  MatrixXd K = P_*Hj.transpose()*S_.inverse();
  x_=x_+K*y;
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_=(I-K*Hj)*P_;
}
