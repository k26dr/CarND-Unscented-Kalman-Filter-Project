#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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
  use_radar_ = false;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 15;

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
  bool is_initialized_ = false;

  n_x_ = 5;
  n_aug_ = 7;

  // Intialize states
  x_ << 0,0,2,M_PI/2,0;
  P_ << MatrixXd::Identity(n_x_,n_x_);
  P_(0,0) = std_laspx_*std_laspx_;
  P_(1,1) = std_laspy_*std_laspy_;
  
  lambda_ = 3 - n_aug_;
  Xsig_pred_ = MatrixXd(2*n_aug_ + 1, n_aug_);
  weights_ = VectorXd(2*n_aug_ + 1);
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
  if (!is_initialized_) {
    cout << "First Measurement" << endl;
    time_us_ = meas_package.timestamp_;
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "Initial Radar" << endl;
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      x_(1) = rho * cos(phi);
      x_(2) = rho * sin(phi);
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        cout << "Initial Laser" << endl;
        float px = meas_package.raw_measurements_[0];
        float py = meas_package.raw_measurements_[1];
        x_(0) = px;
        x_(1) = py;
    }

    cout << meas_package.raw_measurements_ << endl;
    is_initialized_ = true;
    return;
  }

  //////////////////
  // Prediction

  Prediction();

  

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

  // time delta
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  
  // Augmented states
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd X_sig_aug = MatrixXd(2*n_aug_ + 1, n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  x_aug.fill(0.0);

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x_) = P_;
  P_aug(n_x_,n_x_) = std_a_ * std_a_;
  P_aug(n_x_+1, n_x_+1) = std_yawdd_, std_yawdd_;

  // square root matrix
  MatrixXd A = P_aug.llt().matrixL();
    
  // create augmented sigma points
  MatrixXd X2 = sqrt(3) * A;
  X2.colwise() += x_aug;
  MatrixXd X3 = -sqrt(3) * A;
  X3.colwise() += x_aug;
  Xsig_aug << x_aug, X2, X3;
  
  // predict sigma point motion


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
