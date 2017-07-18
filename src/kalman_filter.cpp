#include <iostream>
#include "kalman_filter.h"
#include <cmath>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**t
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  float rho = sqrt(pow(x_(0),2)+pow(x_(1),2));
  float phi = atan2(x_(1),x_(0));
  float rhodot;
  if (rho == 0){
    cout << "error: dividing by zero" << endl;
  }
  else {
    rhodot = (x_(0)*x_(2)+x_(1)*x_(3))/rho;
  }

  VectorXd hx(3);

  hx << rho, phi, rhodot;

  MatrixXd y = z - hx;

  // normalize the angle output after subtraction 
  y(1) = atan2(sin(y(1)), cos(y(1)));
  //while (y(1) < -M_PI)
    //y(1) += 2 * M_PI;
  //while (y(1) > M_PI)
    //y(1) -= 2 * M_PI;

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  
  x_ = x_ + K * y;
  
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  


}
