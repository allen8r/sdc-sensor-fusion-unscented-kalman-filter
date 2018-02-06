#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  time_us_ = 0;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd();
  
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

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "Initialize state x_: " << endl;
    x_ << 1, 1, 1, 1, 1;

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = meas_package.raw_measurements_[0]; 
      float phi = meas_package.raw_measurements_[1];
      x_(0) = cos(phi) * rho; // px
      x_(1) = sin(phi) * rho; // py

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0]; // px
      x_(1) = meas_package.raw_measurements_[1]; // py
    }

    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // Calculate elapsed time 
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(dt);
  


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
  // Generate Sigma Points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  GenerateAugmentedSigmaPoints(&Xsig_aug);

  // Predict Sigma Points
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2*n_aug_+1);
  SigmaPointPrediction(&Xsig_pred, &Xsig_aug, delta_t);

  // Predict State Mean and Covariance matrix
  
   



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

void UKF::GenerateAugmentedSigmaPoints(MatrixXd* Xsig_out) {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  //create augmented mean state
  x_aug.setZero(n_aug_);
  x_aug.head(n_x_) = x_;

  //create augmented covariance matrix
  P_aug.setZero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_aug_-2, n_aug_-2) = std_a_ * std_a_;
  P_aug(n_aug_-1, n_aug_-1) = std_yawdd_ * std_yawdd_;
  
  //std::cout << "P_aug=\n" << P_aug << std::endl;
  
  //create square root matrix
  MatrixXd rt_P_aug = P_aug.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug.setZero(n_aug_, 2 * n_aug_ + 1);
  
  Xsig_aug.col(0) = x_aug;
  
  MatrixXd rt_lambda_P_aug = sqrt(lambda_+n_aug_) * rt_P_aug;
  for (int i = 0; i < n_aug_; i++) {
      Xsig_aug.col(i+1) = x_aug + rt_lambda_P_aug.col(i);
      Xsig_aug.col(n_aug_+i+1) = x_aug - rt_lambda_P_aug.col(i);
  }

  *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(MatrixXd* Xsig_out, MatrixXd* Xsig_aug, double delta_t) {
  //predict sigma points

  //create matrix with predicted sigma points as columns
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2*n_aug_+1);

  VectorXd col_aug = VectorXd(n_aug_);
  VectorXd x = VectorXd(n_x_);
  
  for (int i = 0; i < Xsig_aug->cols(); i++) {
      col_aug = Xsig_aug->col(i);
      x = col_aug.head(n_x_);
      
      //float px = col_aug(0);
      //float py = col_aug(1);
      double v = col_aug(2);
      double psi = col_aug(3);
      double psi_dot = col_aug(4);
      double nu_a = col_aug(5);
      double nu_psi_dotdot = col_aug(6);
      
      VectorXd x1 = VectorXd(n_x_);
      double psi_dot_t = psi_dot * delta_t;
      
      //avoid division by zero
      if (fabs(psi_dot) > 0.0001) {
          
        double v_psi_dot = v / psi_dot;
        double psi_psi_dot_t = psi + psi_dot*delta_t;
        
        x1 << v_psi_dot*(sin(psi_psi_dot_t)-sin(psi)),
              v_psi_dot*(-cos(psi_psi_dot_t)+cos(psi)),
              0,
              psi_dot_t,
              0;
              
      } else {
          
        x1 << v*cos(psi)*delta_t,
              v*sin(psi)*delta_t,
              0,
              psi_dot_t,
              0;
      }
      
      VectorXd noise = VectorXd(n_x_);
      double dt2 = delta_t * delta_t;
      
      noise << 0.5*dt2*cos(psi)*nu_a,
               0.5*dt2*sin(psi)*nu_a,
               delta_t*nu_a,
               0.5*dt2*nu_psi_dotdot,
               delta_t*nu_psi_dotdot;
               
      x += (x1 + noise);
      
    //write predicted sigma points into right column
    Xsig_pred.col(i) = x;
  }
  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred << std::endl;

  //write result
  *Xsig_out = Xsig_pred;
}
