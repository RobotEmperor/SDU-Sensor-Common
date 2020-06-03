/*
 * tool_estimation.h
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */

#ifndef TOOL_ESTIMATION_H_
#define TOOL_ESTIMATION_H_

#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "sdu_math/kinematics.h"
#include "sdu_math/statistics_math.h"
#include "sensor_filter/sensor_filter.h"

class ToolEstimation
{
public:
  ToolEstimation();
  ~ToolEstimation();
  void initialize();
  void offset_init(Eigen::MatrixXd data, int desired_sample_num);

  void set_parameters(double control_time_init, double mass_of_tool_init);
  void set_noise_cov_parameters(double q_noise, double r_noise);
  void set_orientation_data(Eigen::MatrixXd tf_base_to_tool);
  void set_gravity_input_data(Eigen::MatrixXd gravity_input);
  void set_acc_input_data(Eigen::MatrixXd linear_acc_input);
  void set_pose_input_data(Eigen::MatrixXd pose);
  void set_speed_input_data(Eigen::MatrixXd speed);

  Eigen::MatrixXd calculate_angular_acc();
  Eigen::MatrixXd get_offset_data();
  Eigen::MatrixXd get_orientation_angle();
  Eigen::MatrixXd get_orientation_vel();
  Eigen::MatrixXd get_orientation_acc();
  Eigen::MatrixXd get_one_axis_inertia_tensor(Eigen::MatrixXd ft_data, std::string axis);

  Eigen::MatrixXd get_estimated_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data); //

private:
  double control_time_;
  double mass_of_tool_;
  double cutoff_frequency_;

  //filter
  std::shared_ptr<LowPassFilter> lpf_filtered_force;
  std::shared_ptr<KalmanFilter> kf_estimated_force;

  std::shared_ptr<Kinematics> tool_kinematics;
  std::shared_ptr<Statistics> tool_statistics_orientation_vel;
  std::shared_ptr<Statistics> tool_statistics_orientation_acc;

  //noise variables
  double r_,q_;

  Eigen::MatrixXd pose_;
  Eigen::MatrixXd orientation_;
  Eigen::MatrixXd pose_vel_;
  Eigen::MatrixXd orientation_vel_;
  Eigen::MatrixXd pose_acc_;
  Eigen::MatrixXd orientation_acc_;
  Eigen::MatrixXd inertia_of_tool_;

  // accelerometer
  Eigen::MatrixXd tool_linear_acc_data_;
  Eigen::MatrixXd tool_linear_acc_offset_data_;

  Eigen::MatrixXd orientation_base_to_tool_;
  Eigen::MatrixXd gravity_;
  Eigen::MatrixXd compensated_acc_;
  Eigen::MatrixXd angular_acceleration_;

  //force observer sensor
  Eigen::MatrixXd f_F_init_;
  Eigen::MatrixXd f_H_init_;
  Eigen::MatrixXd f_Q_init_;
  Eigen::MatrixXd f_R_init_;
  Eigen::MatrixXd f_B_init_;
  Eigen::MatrixXd f_U_init_;
  Eigen::MatrixXd f_Z_init_;

  Eigen::MatrixXd contacted_force_;
  Eigen::MatrixXd pre_contacted_force_;
};
#endif /* TOOL_ESTIMATION_H_ */
