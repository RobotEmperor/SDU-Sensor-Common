/*
 * sensor_filter.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#include "sensor_filter/sensor_filter.h"

LowPassFilter::LowPassFilter()
{
  control_time_ = 0;
  cutoff_frequency_ = 0;
  lambda_ = 0;
  alpha_ = 0;
  raw_data_ = 0;
}

LowPassFilter::LowPassFilter(double control_time_init, double cutoff_frequency_init)
    : control_time_(control_time_init), cutoff_frequency_(cutoff_frequency_init)
{
  this->initialize();
}

LowPassFilter::~LowPassFilter()
{
}

void LowPassFilter::initialize()
{
  lambda_ = 0;
  alpha_ = 0;
  raw_data_ = 0;

  lambda_ = 1 / (2 * M_PI * cutoff_frequency_);
  alpha_ = control_time_ / (lambda_ + control_time_);
}

void LowPassFilter::set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::MatrixXd data)
{
  control_time_ = control_time_init;
  cutoff_frequency_ = cutoff_frequency_init;

  filtered_data_.resize(data.rows(), data.cols());
  pre_filtered_data_.setZero(data.rows(), data.cols());
}

Eigen::MatrixXd LowPassFilter::get_lpf_filtered_data(Eigen::MatrixXd data)
{
  for (int data_num = 0; data_num < data.size(); data_num++)
  {
    filtered_data_(data_num, 0) = alpha_ * data(data_num, 0) + ((1 - alpha_) * pre_filtered_data_(data_num, 0));
    pre_filtered_data_(data_num, 0) = filtered_data_(data_num, 0);
  }
  return filtered_data_;
}

HighPassFilter::HighPassFilter()
{
  lambda_ = 0.0;
  alpha_ = 0.0;
}

HighPassFilter::HighPassFilter(double control_time_init, double cutoff_frequency_init)
    : control_time_(control_time_init), cutoff_frequency_(cutoff_frequency_init)
{
  this->initialize();
}

HighPassFilter::~HighPassFilter()
{
}

void HighPassFilter::initialize()
{
  raw_data_ = 0.0;

  lambda_ = 1 / (2 * M_PI * cutoff_frequency_);
  alpha_ = lambda_ / (lambda_ + control_time_);
}

void HighPassFilter::set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::MatrixXd data)
{
  control_time_ = control_time_init;
  cutoff_frequency_ = cutoff_frequency_init;

  filtered_data_.resize(data.rows(), data.cols());
  pre_filtered_data_.setZero(data.rows(), data.cols());
  pre_input_data_.setZero(data.rows(), data.cols());
}

Eigen::MatrixXd HighPassFilter::get_hpf_filtered_data(Eigen::MatrixXd data)
{
  for (int data_num = 0; data_num < data.size(); data_num++)
  {
    filtered_data_(data_num, 0) =
        alpha_ * pre_filtered_data_(data_num, 0) + alpha_ * (data(data_num, 0) - pre_input_data_(data_num, 0));
    pre_filtered_data_(data_num, 0) = filtered_data_(data_num, 0);
    pre_input_data_(data_num, 0) = data(data_num, 0);
  }
  return filtered_data_;
}

KalmanFilter::KalmanFilter()
{
}

KalmanFilter::~KalmanFilter()
{
}

void KalmanFilter::initialize_system(Eigen::MatrixXd F_init, Eigen::MatrixXd H_init, Eigen::MatrixXd Q_init,
                                     Eigen::MatrixXd R_init, Eigen::MatrixXd B_init, Eigen::MatrixXd U_init,
                                     Eigen::MatrixXd Z_init)
{
  // must be designed by your system model
  F_.resize(F_init.rows(), F_init.rows());
  H_.resize(Z_init.rows(), F_init.rows());
  Q_.resize(F_init.rows(), F_init.rows());
  R_.resize(Z_init.rows(), Z_init.rows());

  B_.resize(F_init.rows(), F_init.rows());
  U_.resize(F_init.rows(), 1);
  Z_.resize(F_init.rows(), 1);

  F_ = F_init;
  H_ = H_init;
  Q_ = Q_init;
  R_ = R_init;
  B_ = B_init;
  U_ = U_init;
  Z_ = Z_init;

  // intial condition
  correction_value_x_.resize(F_init.rows(), 1);
  correction_value_p_.resize(F_init.rows(), F_init.rows());

  correction_value_x_.fill(0);
  correction_value_p_.setIdentity();

  // variables
  prediction_value_x_.resize(F_init.rows(), 1);
  prediction_value_p_.resize(F_init.rows(), F_init.rows());

  prediction_value_x_.fill(0);
  prediction_value_p_.setIdentity();

  previous_correction_value_x_.resize(F_init.rows(), 1);
  previous_correction_value_p_.resize(F_init.rows(), F_init.rows());

  previous_correction_value_x_.fill(0);
  previous_correction_value_p_.setIdentity();

  previous_correction_value_x_ = correction_value_x_;
  previous_correction_value_p_ = correction_value_p_;

  kalman_gain_k_.resize(F_init.rows(), F_init.rows());
  kalman_gain_k_.fill(0);
}

Eigen::MatrixXd KalmanFilter::get_kalman_filtered_data(Eigen::MatrixXd measurement_z)
{
  prediction_value_x_ = F_ * previous_correction_value_x_ + B_ * U_;

  prediction_value_p_ = F_ * correction_value_p_ * F_.transpose() + Q_;

  if ((H_ * prediction_value_p_ * H_.transpose() + R_).determinant() == 0)
  {
    return correction_value_x_;
  }

  kalman_gain_k_ = prediction_value_p_ * H_.transpose() * ((H_ * prediction_value_p_ * H_.transpose() + R_).inverse());

  correction_value_x_ = prediction_value_x_ + kalman_gain_k_ * (measurement_z - (H_ * prediction_value_x_));

  correction_value_p_ = prediction_value_p_ - (kalman_gain_k_ * H_ * prediction_value_p_);

  previous_correction_value_x_ = correction_value_x_;
  previous_correction_value_p_ = correction_value_p_;

  return correction_value_x_;
}

void KalmanFilter::change_noise_value(Eigen::MatrixXd R_init)
{
  R_ = R_init;
}

KalmanBucyFilter::KalmanBucyFilter()
{
}

KalmanBucyFilter::~KalmanBucyFilter()
{
}

void KalmanBucyFilter::initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables)
{
  // must be designed by your system model
  F_.resize(state_variables.rows(), state_variables.rows());
  H_.resize(measurement_variables.rows(), state_variables.rows());
  Q_.resize(state_variables.rows(), state_variables.rows());
  R_.resize(measurement_variables.rows(), measurement_variables.rows());

  F_.setIdentity();
  H_.setIdentity();
  Q_.setIdentity();
  R_.setIdentity();

  // intial condition
  estimated_value_x_.resize(state_variables.rows(), state_variables.cols());
  estimated_value_p_.resize(state_variables.rows(), state_variables.rows());

  estimated_value_x_.fill(0);
  estimated_value_p_.setIdentity();

  // variables
  dt_value_x_.resize(state_variables.rows(), state_variables.cols());
  dt_value_p_.resize(state_variables.rows(), state_variables.rows());

  dt_value_x_.fill(0);
  dt_value_p_.setIdentity();

  pre_dt_value_x_.resize(state_variables.rows(), state_variables.cols());
  pre_dt_value_p_.resize(state_variables.rows(), state_variables.rows());

  pre_dt_value_x_.fill(0);
  pre_dt_value_p_.setIdentity();

  // kalman gain
  kalman_gain_k_.resize(state_variables.rows(), state_variables.rows());
  kalman_gain_k_.fill(0);
}

Eigen::MatrixXd KalmanBucyFilter::kalman_bucy_filtering_processing(Eigen::MatrixXd measurement_z)
{
  kalman_gain_k_ = estimated_value_p_ * H_.transpose() * R_.inverse();

  dt_value_x_ = F_ * estimated_value_x_ + kalman_gain_k_ * (measurement_z - (H_ * estimated_value_x_));
  //             _                   _    _                _
  dt_value_p_ = F_ * estimated_value_p_ + estimated_value_p_ * F_.transpose() -
                (kalman_gain_k_ * R_ * kalman_gain_k_.transpose()) + Q_;
  //             _
  estimated_value_x_ += ((pre_dt_value_x_ + dt_value_x_) * control_time_) / 2;
  estimated_value_p_ += ((pre_dt_value_p_ + dt_value_p_) * control_time_) / 2;  // trapezoidal method integral
  //
  pre_dt_value_x_ = dt_value_x_;
  pre_dt_value_p_ = dt_value_p_;

  return estimated_value_x_;
}
