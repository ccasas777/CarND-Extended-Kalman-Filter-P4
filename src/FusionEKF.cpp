#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

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
  
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;	
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  
  ekf_.R_ = MatrixXd(2, 2);
  

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
// create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);
  ekf_.Q_ = MatrixXd(4, 4);

  // state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;


  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  
  noise_ax = 9;
  noise_ay = 9;
  
    
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
    
 	std::cout<<"initializing"<<std::endl;
    
    ekf_.x_ << 1, 1, 1, 1;	   
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float p = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float v =   measurement_pack.raw_measurements_[2];
	  ekf_.x_ << p*cos(theta),
      			 p*sin(theta),
      			v*cos(theta),
      			v*sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	  // read measurements  
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1],
      			 0,
      			 0;
     }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    return;
  }
  
  //Calculate dt
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_; 
  
  //restart
  if(dt>0.2 || dt<-0.2){
    
    std::cout<<"restart"<<std::endl;   
    previous_timestamp_ = measurement_pack.timestamp_;
    
  	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float p = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float v =   measurement_pack.raw_measurements_[2];
	  ekf_.x_ << p*cos(theta),
      			 p*sin(theta),
      			v*cos(theta),
      			v*sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
	  // read measurements  
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
                 measurement_pack.raw_measurements_[1],
      			 0,
      			 0;
     }
    return;
  }
  
  

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.   
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1; 

  // 2. Set the process covariance matrix Q
  
  float dt_4 = 1/4* dt*dt*dt*dt;
  float dt_3 = 1/2* dt*dt*dt;
  float dt_2 = dt*dt;
  
  ekf_.Q_ << dt_4 * noise_ax, 0,  dt_3 * noise_ax, 0,
            0, dt_4 * noise_ay, 0, dt_3 * noise_ay,
            dt_3* noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 * noise_ay, 0, dt_2 * noise_ay;
  
  //Motion Prediction
  ekf_.Predict(); 

  
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
      z = VectorXd(3);
      z << measurement_pack.raw_measurements_[0], 
           measurement_pack.raw_measurements_[1],
    	   measurement_pack.raw_measurements_[2];
    
     ekf_.R_ = R_radar_;
    
     float px = ekf_.x_(0);
     float py = ekf_.x_(1);
     float vx = ekf_.x_(2);
     float vy = ekf_.x_(3); 
     float pxy = sqrt(px*px+py*py);
     if (pxy*pxy<0.0001){
      cout<<"error"<<endl;
     }else{
  
      Hj_ <<      px/pxy      ,       py/pxy    , 0                             ,0,
               -py/(pxy*pxy), px/(pxy*pxy)    , 0                             ,0,
                py*(vx*py-vy*px)/(pxy*pxy*pxy), px*(vy*px-vx*py)/(pxy*pxy*pxy),px/pxy,py/pxy;
      ekf_.H_ =  Hj_ ;
     
	  ekf_.UpdateEKF(z);
     }
  } else {
    // TODO: Laser updates
     z = VectorXd(2);	
     z << measurement_pack.raw_measurements_[0], 
          measurement_pack.raw_measurements_[1];
    
    ekf_.R_ =  R_laser_;
    ekf_.H_ =  H_laser_;
    
 	ekf_.Update(z);
   
  }

}
