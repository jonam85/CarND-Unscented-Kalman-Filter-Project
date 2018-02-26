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
  
  ///* State dimension
  n_x_ = 5;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
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

  ///* Augmented state dimension
  n_aug_ = n_x_ + 2;
  
  ///* Sigma points dimension
  n_sig_ = 2 * n_aug_ + 1;

  ///* Sigma point spreading parameter
  lambda_ = 3 - n_x_;
  
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;
        
  // Initialize weights.
  weights_ = VectorXd(n_sig_);
  weights_.fill(0.5 / (n_aug_ + lambda_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  
  // Initialize measurement noice covarieance matrix
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0, std_radphi_*std_radphi_, 0,
              0, 0,std_radrd_*std_radrd_;
  
  R_lidar_ = MatrixXd(2, 2);
  R_lidar_ << std_laspx_*std_laspx_,0,
              0,std_laspy_*std_laspy_;
              

}

UKF::~UKF() {}


void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
 
  

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
   
    //compute the time elapsed between the current and previous measurements
  float dt = (meas_package.timestamp_ - time_us_) / 1000000.0; //dt - expressed in seconds
  time_us_ = meas_package.timestamp_;
  
  //Check if the delta time is > 3 secs and reset
  if((dt > 3) || (dt < 0))
    is_initialized_ = false;
   
  if (!is_initialized_) {
    
    //Set initial values for P_
    P_ << 1,  0,  0,  0,  0,
          0,  1,  0,  0,  0,
          0,  0,  10,  0,  0,
          0,  0,  0,  0.5,  0,
          0,  0,  0,  0,  0.1;
               
    // first measurement
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) 
    {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho;
      float theta;
      float rho_dot;
      rho = meas_package.raw_measurements_[0];
      theta = meas_package.raw_measurements_[1];
      rho_dot = meas_package.raw_measurements_[2];
      
      double px = rho*cos(theta);
      double py = rho*sin(theta);
      double vx = rho_dot*cos(theta);
      double vy = rho_dot*sin(theta);
      double v = sqrt(vx * vx + vy * vy);
      
      //set the state with the initial location and zero velocity
      x_ << px, py, 0 , 0, 0 ;
      is_initialized_ = true;
    }
    else 
    {

      if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) 
      {
        //set the state with the initial location and zero velocity
        x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
        is_initialized_ = true;
      }

    }
    
    time_us_ = meas_package.timestamp_;

    // done initializing, no need to predict or update
    
    return;
  }


  // Prediction step
  Prediction(dt);


  // Update step
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    UpdateRadar(meas_package);
  }
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    UpdateLidar(meas_package);
  }

  // print the output
  //cout << "x_ = " << x_ << endl;
  //cout << "P_ = " << P_ << endl;
  
  //std::cout << "NIS RADAR: " << NIS_radar_;
  //std::cout << "  NIS LIDAR: " << NIS_lidar_ << std::endl;
}

// Function of Prediction
void UKF::Prediction(double delta_t) {

  MatrixXd Xsig_aug = AugmentedSigmaPoints();
  SigmaPointPrediction(Xsig_aug, delta_t);
  PredictMeanAndCovariance();
  
}

// Function of Lidar Update
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  
  int n_z = 2;
  MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, n_sig_);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < n_sig_; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_lidar_;

  VectorXd z = meas_package.raw_measurements_;

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  Tc.fill(0.0);
  for (int i = 0; i < n_sig_; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //angle normalization    
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;    
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;  

    Tc = Tc + weights_(i) * (x_diff * z_diff.transpose());
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  //NIS Lidar Update
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  //print result  
  //std::cout << "Updated state x: " << std::endl << x_ << std::endl;  
  //std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;  

  
}

// Function of Radar Update
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  
  //set measurement dimension, radar can measure r, phi, and r_dot  
  int n_z = 3;  
  
  VectorXd z = meas_package.raw_measurements_;
  
  //create matrix for sigma points in measurement space  
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  Zsig.fill(0.0);
  
  //transform sigma points into measurement space  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
  
    //2n+1 simga points    
    
    // extract values for better readibility    
    double p_x = Xsig_pred_(0,i);    
    double p_y = Xsig_pred_(1,i);    
    double v   = Xsig_pred_(2,i);    
    double yaw = Xsig_pred_(3,i);   

    double v1 = cos(yaw)*v;    
    double v2 = sin(yaw)*v; 
    
    if (fabs(p_x) < 0.001) 
      p_x = 0.001;   // To avoid divide by zero
    
    // measurement model    
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);        //r    
    Zsig(1,i) = atan2(p_y,p_x);                 //phi    
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);     //r_dot  
  }  
  
  //mean predicted measurement  
  VectorXd z_pred = VectorXd(n_z);  
  z_pred.fill(0.0);  
  
  for (int i=0; i < 2*n_aug_+1; i++) {      
    z_pred = z_pred + weights_(i) * Zsig.col(i);  
  }  

  //innovation covariance matrix S  
  MatrixXd S = MatrixXd(n_z,n_z);  
  S.fill(0.0);  

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //2n+1 simga points    
  
    //residual    
    VectorXd z_diff = Zsig.col(i) - z_pred;    
  
    //angle normalization    
    while (z_diff(1)> M_PI) 
      z_diff(1)-=2.*M_PI;    
    while (z_diff(1)<-M_PI) 
      z_diff(1)+=2.*M_PI; 
         
    S = S + weights_(i) * z_diff * z_diff.transpose();  
  }  


  //add measurement noise covariance matrix  
  S = S + R_radar_;  
  
  //print result  
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;  
  //std::cout << "S: " << std::endl << S << std::endl;  
  
    //create matrix for cross correlation 
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix  
  Tc.fill(0.0);  
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //2n+1 simga points    
    //residual    
    VectorXd z_diff = Zsig.col(i) - z_pred;    
    //angle normalization    
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;    
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;    
    // state difference    
    VectorXd x_diff = Xsig_pred_.col(i) - x_;    
    //angle normalization    
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;    
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;    
    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();  
  }  
  
  //Kalman gain K;  
  MatrixXd K = Tc * S.inverse(); 
  
  //residual  
  VectorXd z_diff = z - z_pred;  
  
  //angle normalization  
  while (z_diff(1)> M_PI) 
    z_diff(1)-=2.*M_PI;  
  while (z_diff(1)<-M_PI) 
    z_diff(1)+=2.*M_PI;  

  //update state mean and covariance matrix  
  x_ = x_ + K * z_diff;  
  P_ = P_ - K*S*K.transpose();
  
  //NIS Update
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;


  //print result  
  //std::cout << "Updated state x: " << std::endl << x_ << std::endl;  
  //std::cout << "Updated state covariance P: " << std::endl << P_ << std::endl;  

}

/*
void UKF::GenerateSigmaPoints(MatrixXd* Xsig_out, VectorXd x, MatrixXd P) {  
  //set state dimension  
  int n_x = x.size(); 
  
  //define spreading parameter  
  //**Pass lambda as factor if alterations required
  double lambda = 3 - n_x;  
  
  //create sigma point matrix  
  MatrixXd Xsig = MatrixXd(n_x, 2 * n_x + 1);  
  
  //calculate square root of P  
  MatrixXd A = P.llt().matrixL();
  
  //set first column of sigma point matrix  
  Xsig.col(0)  = x;  
  
  //set remaining sigma points  
  for (int i = 0; i < n_x; i++)  {    
    Xsig.col(i+1)     = x + sqrt(lambda+n_x) * A.col(i);    
    Xsig.col(i+1+n_x) = x - sqrt(lambda+n_x) * A.col(i);  
  }
  
  //print result  
  //std::cout << "Xsig = " << std::endl << Xsig << std::endl;  
  //write result  
  *Xsig_out = Xsig;

}
*/


// Function of Augmented Sigma points generation
MatrixXd UKF::AugmentedSigmaPoints() {  


  //create augmented mean vector  
  VectorXd x_aug = VectorXd(n_aug_);
    
  //create augmented state covariance  
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
  //create sigma point matrix  
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  //create augmented mean state
  x_aug.fill(0.0);
  x_aug.head(5) = x_;  

  //create augmented covariance matrix  
  P_aug.fill(0.0);  
  P_aug.topLeftCorner(n_x_,n_x_) = P_;  
  P_aug(n_x_,n_x_) = std_a_*std_a_;  
  P_aug(n_x_+1,n_x_+1) = std_yawdd_*std_yawdd_;

  //create square root matrix  
  MatrixXd L = P_aug.llt().matrixL();  

  //create augmented sigma points
  Xsig_aug.fill(0.0);  
  Xsig_aug.col(0)  = x_aug;  

  for (int i = 0; i< n_aug_; i++)  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);  
  }    


  //print result  
  //std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;  
  
  //write result  
  //*Xsig_out = Xsig_aug;
  return Xsig_aug;

}

// Function of Sigma points predication
void UKF::SigmaPointPrediction(MatrixXd Xsig_aug, double delta_t) {  


  //create matrix with predicted sigma points as columns  
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);  

  Xsig_pred_.fill(0.0);

 
  //predict sigma points  
  for (int i = 0; i< 2*n_aug_+1; i++)  {    
  
    //extract values for better readability   
    double p_x = Xsig_aug(0,i);    
    double p_y = Xsig_aug(1,i);    
    double v = Xsig_aug(2,i);   
    double yaw = Xsig_aug(3,i);    
    double yawd = Xsig_aug(4,i);    
    double nu_a = Xsig_aug(5,i);    
    double nu_yawdd = Xsig_aug(6,i);    
    
    //predicted state values    
    double px_p, py_p;   
    
    //avoid division by zero    
    if (fabs(yawd) > 0.001) {        
    
    px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));        
    py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );    
    
    }    
    else {        
    
    px_p = p_x + v*delta_t*cos(yaw);        
    py_p = p_y + v*delta_t*sin(yaw);    
    
    }    
    
    double v_p = v;    
    double yaw_p = yaw + yawd*delta_t;   
    double yawd_p = yawd;    
    
    //add noise    
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);    
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);    
    v_p = v_p + nu_a*delta_t;    
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;   
    yawd_p = yawd_p + nu_yawdd*delta_t;    
    
    //write predicted sigma point into right column    
    Xsig_pred_(0,i) = px_p;    
    Xsig_pred_(1,i) = py_p;    
    Xsig_pred_(2,i) = v_p;   
    Xsig_pred_(3,i) = yaw_p;    
    Xsig_pred_(4,i) = yawd_p; 
    
  }
  
  //print result 
  //std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;  
  
  //write result  
  //*Xsig_out = Xsig_pred;
     
}

// Function of Sigma points Predict Mean and Covariance
void UKF::PredictMeanAndCovariance() 
{  
  
  //create vector for weights  
  weights_ = VectorXd(2*n_aug_+1);    
  
  // set weights  
  double weight_0 = lambda_/(lambda_+n_aug_);  
  
  weights_(0) = weight_0;  
  
  for (int i=1; i<2*n_aug_+1; i++) {  
  
    //2n+1 weights    
    double weight = 0.5/(n_aug_+lambda_);    
    weights_(i) = weight;  
  }  
  
  //predicted state mean  
  x_.fill(0.0);  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //iterate over sigma points    
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);  
  }  

  //predicted state covariance matrix  
  P_.fill(0.0);  

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
    //iterate over sigma points    
    // state difference    
    VectorXd x_diff = Xsig_pred_.col(i) - x_;    
    //angle normalization    
    while (x_diff(3)> M_PI) 
      x_diff(3)-=2.*M_PI;    
    while (x_diff(3)<-M_PI) 
      x_diff(3)+=2.*M_PI;    
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;  
  }
  

  //print result  
  //std::cout << "Predicted state" << std::endl;  
  //std::cout << x_ << std::endl;  
  //std::cout << "Predicted covariance matrix" << std::endl;  
  //std::cout << P_ << std::endl;  

}
