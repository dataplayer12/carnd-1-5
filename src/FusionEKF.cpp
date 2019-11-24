#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#define noise_ax 9
#define noise_ay 9

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;
//using Tools::CalculateJacobian;
//Tools t;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
H_laser_<< 1,0,0,0,
  			0,1,0,0;
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    //cout << "EKF: " << endl;
    VectorXd x(4);
    //ekf_.x_ << measurement_pack.raw_measurements_;
    previous_timestamp_=measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
	float rho=measurement_pack.raw_measurements_[0];
    float phi=measurement_pack.raw_measurements_[1];
    float rhodot=measurement_pack.raw_measurements_[2];
    x<<rho*std::cos(phi),rho*std::sin(phi),rhodot*std::cos(phi),rhodot*std::sin(phi);
    //initialize positions and velocities considering tangential velocity to be zero
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	x<<measurement_pack.raw_measurements_(0),
      		measurement_pack.raw_measurements_(1),
      		0,
      		0;
    }

    
  MatrixXd F(4,4);
	F << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

	MatrixXd P(4,4);
    
	P<< 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

  MatrixXd Q(4,4);

  Q<< 0,0,0,0,
      0,0,0,0,
      0,0,0,0,
      0,0,0,0;
  
  ekf_.Init(x,P,F,Q);
  // done initializing, no need to predict or update
  is_initialized_ = true;
  //cout << "Initialized" <<endl;
  return;
  }

  /**
   * Prediction
   */
  //cout << "Processing measurement" <<endl;
	float dt= (measurement_pack.timestamp_-previous_timestamp_)/1000000.0;
	previous_timestamp_=measurement_pack.timestamp_;

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  	ekf_.F_(0,2)=dt;
  	ekf_.F_(1,3)=dt;
    //cout << "ekf_.F set" <<endl;
  //float noise_ax=9;
  //float noise_ay=9;
  //noise_ax, noise_ay are defined as pre-processor directives
  float dt2=dt*dt;
  float dt3=dt2*dt;
  float dt4=dt2*dt2;
  
  	ekf_.Q_<<dt4*noise_ax/4,0,dt3*noise_ax/2,0,
  		0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
  		dt3*noise_ax/2,0,dt2*noise_ax,0,
  		0,dt3*noise_ay/2,0, dt2*noise_ay;

    //cout << "ekf_.Q set" <<endl;

  ekf_.Predict();

  //cout << "Prediction made" <<endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    MatrixXd Hj = Tools::CalculateJacobian(ekf_.x_);
	ekf_.UpdateEKF(measurement_pack.raw_measurements_, R_radar_, Hj);
  } else {
    // TODO: Laser updates
	ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
