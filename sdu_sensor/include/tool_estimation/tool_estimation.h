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
  void set_acc_input_data(Eigen::MatrixXd linear_acc_input);
  void set_pose_input_data(Eigen::MatrixXd pose);
  void set_speed_input_data(Eigen::MatrixXd speed);
  Eigen::MatrixXd calculate_angular_acc();
  Eigen::MatrixXd get_offset_data();
  Eigen::MatrixXd get_orientation_angle();
  Eigen::MatrixXd get_orientation_vel();
  Eigen::MatrixXd get_orientation_acc();
  Eigen::MatrixXd get_one_axis_inertia_tensor(Eigen::MatrixXd ft_data, std::string axis);
  Eigen::MatrixXd get_contacted_force(Eigen::MatrixXd position_data,
      Eigen::MatrixXd target_position_data,
      Eigen::MatrixXd ft_data,
      Eigen::MatrixXd linear_acc_data); //

private:
  double control_time_;
  double mass_of_tool_;

  //filter
  std::shared_ptr<KalmanFilter> kf_estimated_contact_fx_;
  std::shared_ptr<KalmanFilter> kf_estimated_contact_fy_;
  std::shared_ptr<KalmanFilter> kf_estimated_contact_fz_;

  std::shared_ptr<KalmanFilter> kf_accelerometer;
  std::shared_ptr<Kinematics> tool_kinematics;
  std::shared_ptr<Statistics> tool_statistics_orientation_vel;
  std::shared_ptr<Statistics> tool_statistics_orientation_acc;

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

  Eigen::MatrixXd contacted_force_torque_;

  Eigen::MatrixXd filtered_acc_;
  Eigen::MatrixXd angular_acceleration_;

  //force sensor
  Eigen::MatrixXd f_F_init_;
  Eigen::MatrixXd f_H_init_;
  Eigen::MatrixXd f_Q_init_;
  Eigen::MatrixXd f_R_init_;
  Eigen::MatrixXd f_B_init_;
  Eigen::MatrixXd f_U_init_;
  Eigen::MatrixXd f_Z_init_;

  double c1,c2,c3,c4,c5; // configurable variables

  //estimated y and real y variables
  std::map<int, Eigen::MatrixXd> estimated_output_data_;
  std::map<int, Eigen::MatrixXd> measured_output_data_;
  std::map<int, Eigen::MatrixXd> error_output_data_;

  Eigen::MatrixXd pseoudo_inverse_d_f_;

  //acc
  Eigen::MatrixXd acc_F_init_;
  Eigen::MatrixXd acc_H_init_;
  Eigen::MatrixXd acc_Q_init_;
  Eigen::MatrixXd acc_R_init_;
  Eigen::MatrixXd acc_B_init_;
  Eigen::MatrixXd acc_U_init_;
  Eigen::MatrixXd acc_Z_init_;
};
#endif /* TOOL_ESTIMATION_H_ */
