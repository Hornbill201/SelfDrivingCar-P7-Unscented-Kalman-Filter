#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

#define EPS 0.001 // Just a small number

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // state dimension
  n_x_ = x_.size();

  // augmented state estimation
  n_aug_ = n_x_ + 2;

  // number of sigma points
  n_sig_ = 2 * n_aug_ + 1;

  // Predicted Sigma Points mateix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

  // sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // weights of sigma points
  weights_  = VectorXd(n_sig_);

  // measurement noise convariance matrix
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
  			  0, std_radphi_ * std_radphi_, 0,
  			  0, 0, std_radrd_ * std_radrd_;

  R_lidar_ = MatrixXd(2,2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
  			  0, std_laspy_ * std_laspy_;


}


UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if (!is_intialized_){
  	P_ << 1, 0, 0, 0, 0,
  		  0, 1, 0, 0, 0,
  		  0, 0, 1, 0, 0,
  		  0, 0, 0, 1, 0,
  		  0, 0, 0, 0, 1;

  	if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
  		// convert radar from polar to cartesian and initialize state
  		float rho = meas_package.raw_measurements_[0]; 
  		float phi = meas_package.raw_measurements_[1];
  		float rho_dot = meas_package.raw_measurements_[2];
  		//convert
  		float px = pho * cos(phi);
  		float py = rho * sin(phi);
  		float vx = rho_dot * cos(phi);
  		float vy = rho_dot * sin(phi);
  		float v = sqrt(vx*vx + vy*vy);
  		x_ << px, py, v, 0, 0;
  	}
  	else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
  		x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
  		// take care of the diminishing case
  		if (fabs(x_(0)) < EPS && fabs(x_(1)) < EPS){
  			x_(0) = EPS;
  			x_(1) = EPS;
  		}
  	}
  	// Initialize weights
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < weights_.size(); i++) {
        weights_(i) = 0.5 / (n_aug_ + lambda_);
    }

    // save the initial timestamp for delta-t calculation
    previous_timestamp_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;

  }

  double dt = meas_package.timestamp_ - previous_timestamp_;

  dt /= 1000000.0;
  previous_timestamp_ = meas_package.timestamp_;
  Prediction(dt);

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
  	UpdateRadar(meas_package);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
  	UpdateLidar(meas_package);
  }



}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
	double delta_t2 = delta_t * delta_t;
	// augmented mean vector
	VectorXd x_aug = VectorXd(n_aug_);
	// augmented state convariance matrix
	MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
	// sigma points matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

	// Fill the matrices
	x_aug.fill(0.0);
	x_aug.head(n_x_) = x_;
	P_aug.fill(0.0);
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_,n_x_) = std_a_ * std_a_;
	P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

	// Square root of P_aug
	MatrixXd L = P_aug.llt().matrixL();
  	//create augmented sigma points
  	Xsig_aug.col(0) = x_aug;
  	for (int i; i<n_aug; i++){
    	Xsig_aug.col(i+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    	Xsig_aug.col(i+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  	}

  	// predict sigma points



}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
