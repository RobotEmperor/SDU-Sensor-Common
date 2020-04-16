/*
 * sensor_filter.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef SENSOR_FILTER_H_
#define SENSOR_FILTER_H_

#include <math.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include "sdu_math/statistics_math.h"

class LowPassFilter
{
 public:
  LowPassFilter();
  LowPassFilter(double control_time_init, double cutoff_frequency_init);
  ~LowPassFilter();
  void initialize();
  void set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::MatrixXd data);
  Eigen::MatrixXd get_lpf_filtered_data(Eigen::MatrixXd data);  // frq = frequency , ctrl = contro

 private:
  double control_time_;
  double cutoff_frequency_;
  double raw_data_;
  double lambda_;
  double alpha_;

  Eigen::MatrixXd filtered_data_;
  Eigen::MatrixXd pre_filtered_data_;
};

class HighPassFilter
{
 public:
  HighPassFilter();
  HighPassFilter(double control_time_init, double cutoff_frequency_init);
  ~HighPassFilter();
  void initialize();
  void set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::MatrixXd data);
  Eigen::MatrixXd get_hpf_filtered_data(Eigen::MatrixXd data);  // frq = frequency , ctrl = control

 private:
  double control_time_;
  double cutoff_frequency_;
  double raw_data_;
  double lambda_;
  double alpha_;

  Eigen::MatrixXd filtered_data_;
  Eigen::MatrixXd pre_filtered_data_;
  Eigen::MatrixXd pre_input_data_;
};

class KalmanFilter
{
 public:
  KalmanFilter();
  ~KalmanFilter();
  void initialize_system(Eigen::MatrixXd F_init, Eigen::MatrixXd H_init, Eigen::MatrixXd Q_init, Eigen::MatrixXd R_init,
                         Eigen::MatrixXd B_init, Eigen::MatrixXd U_init, Eigen::MatrixXd Z_init);
  void change_noise_value(Eigen::MatrixXd R_init);
  void set_addtional_estimated_y_term(Eigen::MatrixXd add_term);
  void set_system_input_u(Eigen::MatrixXd input_u);
  Eigen::MatrixXd get_estimated_state();
  Eigen::MatrixXd get_output_error();
  double get_contact_force();

  // kalman filter process
  void process_kalman_filtered_data(Eigen::MatrixXd measurement_y);

 private:
  // must be designed by your system model
  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  Eigen::MatrixXd B_;
  Eigen::MatrixXd U_;
  Eigen::MatrixXd Z_;

  // intial condition
  Eigen::MatrixXd correction_value_x_;
  Eigen::MatrixXd correction_value_p_;

  // variables
  Eigen::MatrixXd prediction_value_x_;
  Eigen::MatrixXd prediction_value_p_;

  Eigen::MatrixXd previous_correction_value_x_;
  Eigen::MatrixXd previous_correction_value_p_;

  // kalman gain
  Eigen::MatrixXd kalman_gain_k_;

  // output variables
  Eigen::MatrixXd estimated_y_;
  Eigen::MatrixXd additonal_estimated_y_;
  Eigen::MatrixXd output_error_;

  double raw_data_;
};

class KalmanBucyFilter
{
 public:
  KalmanBucyFilter();
  ~KalmanBucyFilter();
  void initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables);
  Eigen::MatrixXd kalman_bucy_filtering_processing(Eigen::MatrixXd measurement_z);

 private:
  // variables

  double control_time_;

  Eigen::MatrixXd F_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;

  Eigen::MatrixXd estimated_value_x_;
  Eigen::MatrixXd estimated_value_p_;

  Eigen::MatrixXd dt_value_x_;
  Eigen::MatrixXd dt_value_p_;

  Eigen::MatrixXd pre_dt_value_x_;
  Eigen::MatrixXd pre_dt_value_p_;

  // kalman gain
  Eigen::MatrixXd kalman_gain_k_;

  double raw_data_;
};
#endif /* SENSOR_FILTER_H_ */
