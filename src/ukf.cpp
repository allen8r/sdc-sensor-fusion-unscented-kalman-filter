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
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3;
  
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

  //// Initiailize Laser matrices ////
  // Initial transition matrix F
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << std_laspx_ * std_laspx_, 0,
             0, std_laspy_ * std_laspy_;

  // Initialize measurement noise
  Q_ = MatrixXd(4, 4);
  Q_ << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0;

  // Set acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
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
    cout << "Initialize state x_." << endl;
    

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_ << 1, 1, 1, 1, 1;
      float rho = meas_package.raw_measurements_[0]; 
      float phi = meas_package.raw_measurements_[1];
      x_(0) = cos(phi) * rho; // px
      x_(1) = sin(phi) * rho; // py

    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ = VectorXd(4);
      x_ << 1, 1, 1, 1;
      x_(0) = meas_package.raw_measurements_[0]; // px
      x_(1) = meas_package.raw_measurements_[1]; // py

      // Initialize process noise covariance matrix
      P_ = MatrixXd(4, 4);
      P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;
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

  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    PredictionRadar(dt);
  } else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
    PredictionLidar(dt);
  }
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    UpdateRadar(meas_package);    

  } else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    UpdateLidar(meas_package);
    
  }

  // print the output
  cout << "x_ = \n" << x_ << endl;
  cout << "P_ = \n" << P_ << endl << endl;


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::PredictionRadar(double delta_t) {
  /**
  Estimate the object's location. Predict sigma points, the state, 
  and the state covariance matrix.
  */

  // Generate Sigma Points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
  GenerateAugmentedSigmaPoints(&Xsig_aug);

  // Predict Sigma Points
  SigmaPointPrediction(&Xsig_aug, delta_t);

  // Predict State Mean and Covariance matrix
  PredictMeanAndCovariance();
  
}

void UKF::PredictionLidar(double delta_t) {
  // Integrate time change into the state transition matrix F
  F_(0, 2) = delta_t;
  F_(1, 3) = delta_t;

  // Update process noise covariance matrix Q
  double dt2 = delta_t * delta_t;
  double dt3 = dt2 * delta_t;
  double dt4 = dt2 * dt2;
  double dt4_4 = dt4 / 4;
  double dt4_4_ax = dt4_4 * noise_ax;
  double dt4_4_ay = dt4_4 * noise_ay;
  double dt3_2 = dt3 / 2 ;
  double dt3_2_ax = dt3_2 * noise_ax;
  double dt3_2_ay = dt3_2 * noise_ay;

  Q_ << dt4_4_ax,        0,     dt3_2_ax,            0,
               0, dt4_4_ay,            0,     dt3_2_ay,
        dt3_2_ax,        0, dt2*noise_ax,            0,
               0, dt3_2_ay,            0, dt2*noise_ay;

  // Predict the state
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
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
  VectorXd z = VectorXd(n_x_);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2],
       meas_package.raw_measurements_[3];
  
  UpdateLidarState(z);

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
  VectorXd z_pred = VectorXd(3);
  MatrixXd S_out = MatrixXd(3, 3);
  MatrixXd z_sig = MatrixXd(3, 15);
  PredictRadarMeasurement(&z_pred, &S_out, &z_sig);

  VectorXd z = VectorXd(3);
  z << meas_package.raw_measurements_[0],
       meas_package.raw_measurements_[1],
       meas_package.raw_measurements_[2];
  UpdateRadarState(&z_sig, &z, &z_pred, &S_out);
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

void UKF::SigmaPointPrediction(MatrixXd* Xsig_aug, double delta_t) {
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
      
    //write predicted sigma points into column
    Xsig_pred.col(i) = x;
  }

  //write result
  Xsig_pred_ = Xsig_pred;
}

void UKF::PredictMeanAndCovariance() {
  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
  
  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

  //set weights
  double w = 1 / (lambda_ + n_aug_);
  double w0 = lambda_ * w; // weight for col 0
  double wt = 0.5 * w;     // weight for cols 1 to 2*n_aug_
  
  // sub matrix of Xsig_pred without the first column
  MatrixXd Xsig_pred_sub = Xsig_pred_.block(0, 1, n_x_, 2*n_aug_);
  
  //predict state mean
  x = w0 * Xsig_pred_.col(0) + wt * Xsig_pred_sub.rowwise().sum();
  
  //predict state covariance matrix
  MatrixXd X_diff = Xsig_pred_sub.colwise() - x;
  //normalize angle calcuations for 4th row of x
  for (int i = 0; i < X_diff.cols(); i++) {
      X_diff.col(i)(3) = NormalizeAngle(X_diff.col(i)(3));
  }
  
  P = w0 * (Xsig_pred_.col(0) - x) * (Xsig_pred_.col(0) - x).transpose() +
      wt * (X_diff) * (X_diff).transpose();

  //write result
  x_ = x;
  P_ = P;
}

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out, MatrixXd* z_sig) {

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug_;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
   double weight_0 = lambda/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<weights.size(); i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
  //transform sigma points into measurement space
  Zsig.setZero();
  for (int i = 0; i < 2*n_aug_+1; i++) {
    VectorXd x = Xsig_pred_.col(i);
    double px = x(0);
    double py = x(1);
    double v = x(2);
    double psi = x(3);
    double px2 = px * px;
    double py2 = py * py;
    double px2_py2 = px2 + py2;
    double rt_px2_py2 = sqrt(px2_py2);
    if (fabs(px) > 0.0001 && fabs(rt_px2_py2) > 0.001) {
        double rho = rt_px2_py2;
        double phi = atan2(py, px);
        // Normalize phi
        phi = NormalizeAngle(phi);
        double rho_dot = ((px*cos(psi)*v) + (py*sin(psi)*v)) / rt_px2_py2;
        Zsig.col(i) << rho, phi, rho_dot;
    }
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  //calculate mean predicted measurement
  z_pred.setZero();
  for (int i = 0; i < weights.size(); i++) {
    z_pred += weights(i) * Zsig.col(i); 
  }
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  //calculate innovation covariance matrix S
  S.setZero();
  for (int i = 0; i < weights.size(); i++) {
    MatrixXd Zdiff = Zsig.col(i) - z_pred;
    //Normalize angle phi
    Zdiff(1) = NormalizeAngle(Zdiff(1));

    S += weights(i) * (Zdiff * Zdiff.transpose());
  }
 
  // Add noise covariance matrix R
  MatrixXd R = MatrixXd(n_z, n_z);
  R.setZero();
  R(0, 0) = std_radr_ * std_radr_;
  R(1, 1) = std_radphi_ * std_radphi_;
  R(2, 2) = std_radrd_ * std_radrd_;
  S += R; 
  
  //write result
  *z_out = z_pred;
  *S_out = S;
  *z_sig = Zsig;
}

void UKF::UpdateRadarState(MatrixXd* Zsig, VectorXd* z, VectorXd* z_pred, MatrixXd* S) {
  // Zsig   - matrix with sigma points in measurement space
  // z      - vector of incoming radar measurement
  // z_pred - vector of mean predicted measurement
  // S      - predicted measurement covariance

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1);
   double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.setZero();
  //calculate cross correlation matrix
  for (int i = 0; i < 2*n_aug_+1; i++) {
      // state difference
      VectorXd Xdiff = Xsig_pred_.col(i) - x_;
      Xdiff(3) = NormalizeAngle(Xdiff(3));
      
      // residual
      VectorXd Zdiff = Zsig->col(i) - *z_pred;
      Zdiff(1) = NormalizeAngle(Zdiff(1));
      
      Tc += weights(i) * (Xdiff * Zdiff.transpose());
  }
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S->inverse();
  
  //update state mean
  VectorXd z_diff = *z - *z_pred;
  //Normalize angle phi
  z_diff(1) = NormalizeAngle(z_diff(1));
  x_ = x_ + K * z_diff;
  
  //update covariance matrix
  P_ = P_ - K * *S * K.transpose();

}

void UKF::UpdateLidarState(VectorXd z) {
  //update the state by using Kalman Filter equations

  VectorXd z_pred = H_laser_ * x_;
  VectorXd y = z - z_pred; // error
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;     

  // convert vx, vy (x_[2], x_[3]) to v and yaw
  float vx = x_[2];
  float vy = x_[3];
  float v = 0;
  float yaw = 0;
  if (fabs(vx) > 0.0001) {
    yaw = atan2(vy, vx);
    if (fabs(cos(yaw)) > 0.0001) {
      v = vx / cos(yaw);
    }
  }
  x_[2] = v;
  x_[3] = yaw;
}


double UKF::NormalizeAngle(double a) {
  double normalized = a;
  while (normalized >  M_PI) normalized -= 2*M_PI;
  while (normalized < -M_PI) normalized += 2*M_PI;
  return normalized;
}