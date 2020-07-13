/*
 * tool_estimation.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */
#include "tool_estimation/tool_estimation.h"

ToolEstimation::ToolEstimation()
{
  initialize();
}

ToolEstimation::~ToolEstimation()
{
  delete kf_estimated_force;
}

void ToolEstimation::initialize()
{
  control_time_ = 0.002;
  mass_of_tool_ = 2.52;
  cutoff_frequency_ = 10;

  r_ = 1000;
  q_ = 0.1;

  kf_estimated_force = new KalmanFilter;

  tool_linear_acc_offset_data_.resize(4, 1);
  tool_linear_acc_data_.resize(4, 1);

  inertia_of_tool_.resize(3, 3);
  contacted_force_.resize(3, 1);
  pre_contacted_force_.resize(3,1);

  pose_.resize(3,1);
  orientation_.resize(3,1);

  pose_vel_.resize(3,1);
  orientation_vel_.resize(3,1);

  pose_acc_.resize(3,1);
  orientation_acc_.resize(3,1);

//  tool_statistics_orientation_vel->set_dimension(orientation_);
//  tool_statistics_orientation_acc->set_dimension(orientation_);

  orientation_base_to_tool_.resize(3,3);
  orientation_base_to_tool_.fill(0);

  //////////////////////////////////
  tool_linear_acc_offset_data_.fill(0);
  tool_linear_acc_data_.fill(0);

  inertia_of_tool_.fill(0);
  contacted_force_.fill(0);
  pre_contacted_force_.fill(0);

  pose_.fill(0);
  orientation_.fill(0);

  pose_vel_.fill(0);
  orientation_vel_.fill(0);

  pose_acc_.fill(0);
  orientation_acc_.fill(0);

  //filtered_acc_

  gravity_.resize(3,1);
  gravity_.fill(0);

  compensated_acc_.resize(3,1);
  compensated_acc_.fill(0);

  angular_acceleration_.resize(3, 1);
  angular_acceleration_.fill(0);

  // force contact model design initialize
  f_F_init_.resize(6, 6);
  f_H_init_.resize(6, 6);
  f_Q_init_.resize(6, 6);
  f_R_init_.resize(6, 6);
  f_B_init_.resize(6, 6);
  f_U_init_.resize(6, 1);
  f_Z_init_.resize(6, 1);

  f_F_init_.setIdentity();
  f_H_init_.setIdentity();

  f_B_init_.setZero();
  f_Q_init_.setIdentity();
  f_R_init_.setIdentity();
  f_Z_init_.setZero();
  f_U_init_.setZero();
  f_Q_init_ = f_Q_init_ * q_;
  f_R_init_ = f_R_init_ * r_;

  kf_estimated_force->initialize_system(f_F_init_, f_H_init_, f_Q_init_, f_R_init_, f_B_init_, f_U_init_, f_Z_init_);
}

void ToolEstimation::set_parameters(double control_time_init, double mass_of_tool_init)
{
  control_time_ = control_time_init;
  mass_of_tool_ = mass_of_tool_init;
}
void ToolEstimation::set_noise_cov_parameters(double q_noise, double r_noise)
{
  q_ = q_noise;
  r_ = r_noise;
}

void ToolEstimation::set_orientation_data(Eigen::MatrixXd tf_base_to_tool)
{
  orientation_base_to_tool_ = tf_base_to_tool.block(0,0,3,3);

}
void ToolEstimation::set_gravity_input_data(Eigen::MatrixXd gravity_input)
{

  gravity_ = orientation_base_to_tool_*gravity_input;
}

void ToolEstimation::set_acc_input_data(Eigen::MatrixXd linear_acc_input)
{
  tool_linear_acc_data_ = linear_acc_input;
}
void ToolEstimation::set_pose_input_data(Eigen::MatrixXd pose)
{
  for(int num = 0; num < 3 ; num ++)
  {
    pose_(num,0) = pose(num,0);
  }

  //orientation_ = tool_kinematics->get_axis_to_euler_angle(pose(3,0),pose(4,0),pose(5,0));
}
void ToolEstimation::set_speed_input_data(Eigen::MatrixXd speed)
{
  for(int num = 0; num < 3 ; num ++)
  {
    pose_vel_(num,0) = speed(num,0);
  }

  //orientation_vel_ = tool_kinematics->get_axis_to_euler_angle(speed(3,0),speed(4,0),speed(5,0));

}
void ToolEstimation::offset_init(Eigen::MatrixXd data,  int desired_sample_num)
{
  static int sample_num = 1;

  tool_linear_acc_offset_data_ += data;

  if (sample_num == desired_sample_num)
  {
    tool_linear_acc_offset_data_ = tool_linear_acc_offset_data_ / (sample_num);
    sample_num = 1;
    return;
  }
  sample_num++;
}

Eigen::MatrixXd ToolEstimation::get_estimated_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data)  // input entire force torque
{

  if(orientation_base_to_tool_.determinant() == 0) // inverse check
    return contacted_force_;

  compensated_acc_ = linear_acc_data - (orientation_base_to_tool_.inverse()*gravity_);

  compensated_acc_ = orientation_base_to_tool_ * compensated_acc_;

  ft_data(0,0) =  ft_data(0,0) - (mass_of_tool_*compensated_acc_)(0,0);
  ft_data(1,0) =  ft_data(1,0) - (mass_of_tool_*compensated_acc_)(1,0);
  ft_data(2,0) =  ft_data(2,0) - (mass_of_tool_*compensated_acc_)(2,0);




  kf_estimated_force->process_kalman_filtered_data(ft_data);

  contacted_force_ = kf_estimated_force->get_estimated_state();


  contacted_force_ = contacted_force_*0.01 + pre_contacted_force_*0.99;

  pre_contacted_force_ = contacted_force_;

  //contacted_force_ = lpf_filtered_force->get_lpf_filtered_data(contacted_force_);

  return contacted_force_;
}

Eigen::MatrixXd ToolEstimation::calculate_angular_acc()
{
  Eigen::MatrixXd angle;  // defines roll pitch yaw
  angle.resize(3, 1);
  angle.fill(0);

  //filtered_acc_ = kf_accelerometer->get_kalman_filtered_data(tool_linear_acc_data_);

  // angle(0,0) = atan(-filtered_acc_(1,0)/(sqrt(pow(filtered_acc_(0,0),2) + pow(filtered_acc_(2,0),2) )));
  // angle(1,0) = atan(-filtered_acc_(0,0)/(sqrt(pow(filtered_acc_(1,0),2) + pow(filtered_acc_(2,0),2) ))); // rad


  //angle(0,0) = filtered_acc_(0,0);
  //angle(1,0) = filtered_acc_(1,0);


  //pose_vel_(num,0) = calculate_diff(pose_(num,0), control_time_);
  //orientation_vel_ = tool_statistics_orientation_vel->calculate_diff(orientation_, control_time_);

  //pose_acc_(num,0) = calculate_diff(pose_vel_(num,0), control_time_);
 // orientation_acc_ = tool_statistics_orientation_acc->calculate_diff(orientation_vel_, control_time_);

  angular_acceleration_ = orientation_acc_;


  //orientation_vel_(0,0) = calculate_diff(orientation_(0,0), control_time_);
  return orientation_acc_;
}

Eigen::MatrixXd ToolEstimation::get_one_axis_inertia_tensor(Eigen::MatrixXd ft_data, std::string axis)
{
  if (axis.compare("x") == 0)
  {

    if(angular_acceleration_(0,0) == 0)
    {
      inertia_of_tool_(0, 0) = 0;
      inertia_of_tool_(1, 0) = 0;
      inertia_of_tool_(2, 0) = 0;
    }
    else
    {
      inertia_of_tool_(0, 0) = ft_data(3, 0) / angular_acceleration_(0, 0);
      inertia_of_tool_(1, 0) = ft_data(4, 0) / angular_acceleration_(0, 0);
      inertia_of_tool_(2, 0) = ft_data(5, 0) / angular_acceleration_(0, 0);
    }
    return inertia_of_tool_;
  }
  if (axis.compare("y") == 0)
  {
    if (angular_acceleration_(1, 0) == 0)
    {
      inertia_of_tool_(0, 1) = 0;
      inertia_of_tool_(1, 1) = 0;
      inertia_of_tool_(2, 1) = 0;
    }
    else
    {
      inertia_of_tool_(0, 1) = ft_data(3, 0) / angular_acceleration_(1, 0);
      inertia_of_tool_(1, 1) = ft_data(4, 0) / angular_acceleration_(1, 0);
      inertia_of_tool_(2, 1) = ft_data(5, 0) / angular_acceleration_(1, 0);
    }
    return inertia_of_tool_;
  }
  if (axis.compare("z") == 0)
  {
    if (angular_acceleration_(2, 0) == 0)
    {
      inertia_of_tool_(0, 2) = 0;
      inertia_of_tool_(1, 2) = 0;
      inertia_of_tool_(2, 2) = 0;
    }
    else
    {
      inertia_of_tool_(0, 2) = ft_data(3, 0) / angular_acceleration_(2, 0);
      inertia_of_tool_(1, 2) = ft_data(4, 0) / angular_acceleration_(2, 0);
      inertia_of_tool_(2, 2) = ft_data(5, 0) / angular_acceleration_(2, 0);
    }
    return inertia_of_tool_;
  }
}

Eigen::MatrixXd ToolEstimation::get_offset_data()
{
  static Eigen::MatrixXd offset_data;

  offset_data = tool_linear_acc_offset_data_;

  return offset_data;
}
Eigen::MatrixXd ToolEstimation::get_orientation_angle()
{
  static Eigen::MatrixXd euler_angle;

  euler_angle = orientation_;

  return euler_angle;

}
Eigen::MatrixXd ToolEstimation::get_orientation_vel()
{
  static Eigen::MatrixXd orientation_vel;

  orientation_vel = orientation_vel_;

  return orientation_vel;
}
Eigen::MatrixXd ToolEstimation::get_orientation_acc()
{
  static Eigen::MatrixXd orientation_acc;

  orientation_acc = orientation_acc_;

  return orientation_acc;

}
