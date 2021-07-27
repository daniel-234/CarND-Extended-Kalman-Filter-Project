#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  //MatrixXd Ft = F_.transpose();

  // DELETE
  std::cout << "Initialization in Kalman Filter: " << std::endl;
  std::cout << "x: " << x_ << std::endl;
  std::cout << "P: " << P_ << std::endl;
  std::cout << "F: " << F_ << std::endl;
  std::cout << "F transpose: " << F_.transpose() << std::endl;
  std::cout << "Q: " << Q_ << std::endl;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  // Section 9 of Lesson 24

  // DELETE
  std::cout << "Six inside Predict" << std::endl;
  std::cout << "x: " << x_ << std::endl;
  std::cout << "F: " << F_ << std::endl;
  x_ = F_ * x_;
  std::cout << "x: " << x_ << std::endl;
  MatrixXd Ft = F_.transpose();
  std::cout << "F transpose: " << Ft << std::endl;
  std::cout << "P: " << P_ << std::endl;
  std::cout << "Q: " << Q_ << std::endl;
  P_ = F_ * P_ * Ft + Q_;

  std::cout << "P: " << P_ << std::endl;

  // DELETE
  std::cout << "Six inside Predict after function" << std::endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px*px + py*py);
  float theta = atan2(py, px);
  float ro_dot = (px*vx + py*vy) / rho;

  VectorXd u(4);
  u << 0, 0, 0, 0;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, ro_dot;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


  std::cout << "Seven inside Update before prediction step" << std::endl;
  std::cout << "u: " << u << std::endl;

  // EKF Prediction step
  x_ = F_ * x_ + u;
  std::cout << "x: " << x_ << std::endl;

  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}
