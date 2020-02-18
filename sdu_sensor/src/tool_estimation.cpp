/*
 * tool_estimation.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */
// Tool estimation
#include "tool_estimation/tool_estimation.h"

ToolEstimation::ToolEstimation()
{
  initialize();
}

ToolEstimation::~ToolEstimation()
{
}

void ToolEstimation::initialize()
{
  kf_estimated_contact_ft = std::make_shared<KalmanFilter>();

  control_time_ = 0;
  mass_of_tool_ = 0;

  tool_linear_acc_offset_data_.resize(4,1);
  tool_linear_acc_data_.resize(4,1);

  inertia_of_tool_.resize(3,3);
  contacted_force_torque_.resize(6,1);
  estimated_data_.resize(6,1);

  //////////////////////////////////
  tool_linear_acc_offset_data_.fill(0);
  tool_linear_acc_data_.fill(0);

  inertia_of_tool_.fill(0);
  contacted_force_torque_.fill(0);
  estimated_data_.fill(0);

  F_init_.resize(6,6);
  H_init_.resize(6,6);
  Q_init_.resize(6,6);
  R_init_.resize(6,6);
  B_init_.resize(6,6);
  U_init_.resize(6,1);
  Z_init_.resize(6,1);

  F_init_.setIdentity();
  H_init_.setIdentity();
  Q_init_.setIdentity();
  R_init_.setIdentity();
  B_init_.setZero();
  U_init_.setZero();
  Z_init_.setZero();

  Q_init_ = Q_init_ * 0.001;
  R_init_ = R_init_ * 20;

  kf_estimated_contact_ft->initialize_system(F_init_,H_init_,Q_init_,R_init_,B_init_,U_init_,Z_init_);
}
void ToolEstimation::set_parameters(double control_time_init, double mass_of_tool_init)
{
  control_time_ = control_time_init;
  mass_of_tool_ = mass_of_tool_init;
}

void ToolEstimation::offset_init(Eigen::MatrixXd data,  int desired_sample_num)
{
  static int sample_num = 1;

  tool_linear_acc_offset_data_ += data;

  if(sample_num == desired_sample_num)
  {
    tool_linear_acc_offset_data_ = tool_linear_acc_offset_data_/(sample_num);
    sample_num = 1;
    return;
  }
  sample_num ++;
}
Eigen::MatrixXd ToolEstimation::get_contacted_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data) // input entire force torque
{
  //calculate contact force
  contacted_force_torque_(0,0) = ft_data(0,0) -(mass_of_tool_ * linear_acc_data(0,0))*-1;
  contacted_force_torque_(1,0) = ft_data(1,0) -(mass_of_tool_ * linear_acc_data(1,0))*-1;
  contacted_force_torque_(2,0) = ft_data(2,0) -(mass_of_tool_ * linear_acc_data(2,0))*-1;

  contacted_force_torque_ = kf_estimated_contact_ft->get_kalman_filtered_data(contacted_force_torque_);

  return contacted_force_torque_;
}
Eigen::MatrixXd ToolEstimation::get_offset_data()
{
  static Eigen::MatrixXd offset_data;

  offset_data = tool_linear_acc_offset_data_;

  return offset_data;
}
