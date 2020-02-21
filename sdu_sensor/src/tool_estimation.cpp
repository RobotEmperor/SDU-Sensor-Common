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
  kf_accelerometer        = std::make_shared<KalmanFilter>();

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

  //filtered_acc_
  filtered_acc_.resize(3,1);
  filtered_acc_.fill(0);

  // ft contact initialize
  ft_F_init_.resize(6,6);
  ft_H_init_.resize(6,6);
  ft_Q_init_.resize(6,6);
  ft_R_init_.resize(6,6);
  ft_B_init_.resize(6,6);
  ft_U_init_.resize(6,1);
  ft_Z_init_.resize(6,1);
  ft_F_init_.setIdentity();
  ft_H_init_.setIdentity();
  ft_Q_init_.setIdentity();
  ft_R_init_.setIdentity();
  ft_B_init_.setZero();
  ft_U_init_.setZero();
  ft_Z_init_.setZero();
  ft_Q_init_ = ft_Q_init_ * 0.001;
  ft_R_init_ = ft_R_init_ * 20;

  kf_estimated_contact_ft->initialize_system(ft_F_init_,ft_H_init_,ft_Q_init_,ft_R_init_,ft_B_init_,ft_U_init_,ft_Z_init_);

  // accelerometer initialize
  acc_R_init_ = ft_R_init_ * 20;

  acc_F_init_.resize(3,3);
  acc_H_init_.resize(3,3);
  acc_Q_init_.resize(3,3);
  acc_R_init_.resize(3,3);
  acc_B_init_.resize(3,3);
  acc_U_init_.resize(3,1);
  acc_Z_init_.resize(3,1);

  acc_F_init_.setIdentity();
  acc_H_init_.setIdentity();
  acc_Q_init_.setIdentity();
  acc_R_init_.setIdentity();
  acc_B_init_.setZero();
  acc_U_init_.setZero();
  acc_Z_init_.setZero();

  acc_Q_init_ = acc_Q_init_ * 0.001;
  acc_R_init_ = acc_R_init_ * 20;

  kf_accelerometer->initialize_system(acc_F_init_,acc_H_init_,acc_Q_init_,acc_R_init_,acc_B_init_,acc_U_init_,acc_Z_init_);
}
void ToolEstimation::set_parameters(double control_time_init, double mass_of_tool_init)
{
  control_time_ = control_time_init;
  mass_of_tool_ = mass_of_tool_init;
}
void ToolEstimation::set_acc_input_data(Eigen::MatrixXd linear_acc_input)
{
  tool_linear_acc_data_ = linear_acc_input;
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
Eigen::MatrixXd ToolEstimation::get_angular_acc()
{
  Eigen::MatrixXd angle; // defines roll pitch yaw
  angle.resize(3,1);
  angle.fill(0);

  filtered_acc_ = kf_accelerometer->get_kalman_filtered_data(tool_linear_acc_data_);

  angle(0,0) = atan(-filtered_acc_(0,1)/(sqrt(pow(filtered_acc_(0,0),2) + pow(filtered_acc_(0,2),2) )));
  angle(0,1) = atan(-filtered_acc_(0,0)/(sqrt(pow(filtered_acc_(0,1),2) + pow(filtered_acc_(0,2),2) ))); // rad

  return angle;
}
Eigen::MatrixXd ToolEstimation::get_one_axis_inertia_tensor(Eigen::MatrixXd ft_data, std::string axis)
{



}
Eigen::MatrixXd ToolEstimation::get_offset_data()
{
  static Eigen::MatrixXd offset_data;

  offset_data = tool_linear_acc_offset_data_;

  return offset_data;
}
