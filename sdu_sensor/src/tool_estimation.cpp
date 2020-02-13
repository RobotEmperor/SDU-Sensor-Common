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

  control_time = 0;
  mass_of_tool = 0;

  tool_linear_acc_offset_data.resize(4,1);
  tool_linear_acc_data.resize(4,1);

  inertia_of_tool.resize(3,3);
  contacted_force_torque.resize(6,1);
  estimated_data.resize(6,1);

  //////////////////////////////////
  tool_linear_acc_offset_data.fill(0);
  tool_linear_acc_data.fill(0);

  inertia_of_tool.fill(0);
  contacted_force_torque.fill(0);
  estimated_data.fill(0);

  F_init.resize(6,6);
  H_init.resize(6,6);
  Q_init.resize(6,6);
  R_init.resize(6,6);
  B_init.resize(6,6);
  U_init.resize(6,1);
  Z_init.resize(6,1);

  F_init.setIdentity();
  H_init.setIdentity();
  Q_init.setIdentity();
  R_init.setIdentity();
  B_init.setZero();
  U_init.setZero();
  Z_init.setZero();

  Q_init = Q_init * 0.001;
  R_init = R_init * 20;

  kf_estimated_contact_ft->initialize_system(F_init,H_init,Q_init,R_init,B_init,U_init,Z_init);
}
void ToolEstimation::set_parameters(double control_time_init, double mass_of_tool_init)
{
  control_time = control_time_init;
  mass_of_tool = mass_of_tool_init;
}

void ToolEstimation::offset_init(Eigen::MatrixXd data,  int desired_sample_num)
{
  static int sample_num = 1;

  tool_linear_acc_offset_data += data;

  if(sample_num == desired_sample_num)
  {
    tool_linear_acc_offset_data = tool_linear_acc_offset_data/(sample_num);
    sample_num = 1;
    return;
  }
  sample_num ++;
}
Eigen::MatrixXd ToolEstimation::get_contacted_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data) // input entire force torque
{
  //calculate contact force
  contacted_force_torque(0,0) = ft_data(0,0) -(mass_of_tool * linear_acc_data(0,0))*-1;
  contacted_force_torque(1,0) = ft_data(1,0) -(mass_of_tool * linear_acc_data(1,0))*-1;
  contacted_force_torque(2,0) = ft_data(2,0) -(mass_of_tool * linear_acc_data(2,0))*-1;

  contacted_force_torque = kf_estimated_contact_ft->get_kalman_filtered_data(contacted_force_torque);

  return contacted_force_torque;
}
Eigen::MatrixXd ToolEstimation::get_offset_data()
{
  static Eigen::MatrixXd offset_data;

  offset_data = tool_linear_acc_offset_data;

  return offset_data;
}
