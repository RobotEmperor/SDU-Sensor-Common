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
  control_time = 0;
  mass_of_tool = 0;

  tool_linear_acc_offset_data.resize(3,1);
  tool_linear_acc_data.resize(3,1);

  inertia_of_tool.resize(3,3);
  contacted_force_torque.resize(3,1);
  estimated_data.resize(6,1);

  //////////////////////////////////
  tool_linear_acc_offset_data.fill(0);
  tool_linear_acc_data.fill(0);

  inertia_of_tool.fill(0);
  contacted_force_torque.fill(0);
  estimated_data.fill(0);
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


void ToolEstimation::estimation_processing(Eigen::MatrixXd data) // input entire force torque
{
  // tool_linear_acc_data = tool_linear_acc_data - offset_data;
  //filtered_data = kalman_filter_linear_acc->kalman_filtering_processing(tool_linear_acc_data);

  //calculate contact force
  contacted_force_torque(0,0) = data(0,0) -(mass_of_tool * tool_linear_acc_data(0,0))*-1;
  contacted_force_torque(1,0) = data(1,0) -(mass_of_tool * tool_linear_acc_data(1,0))*-1;
  contacted_force_torque(2,0) = data(2,0) -(mass_of_tool * tool_linear_acc_data(2,0))*-1;
  //
  //  filtered_data = contacted_force_torque;
}

std::vector<double> ToolEstimation::get_contact_force_data()
{
  std::vector<double> tool_contact_force_data_vector;
  tool_contact_force_data_vector.clear();

  for(int num = 0; num < 3; num ++)
  {
    tool_contact_force_data_vector.push_back(contacted_force_torque(num,0));
  }
  return tool_contact_force_data_vector;
}
