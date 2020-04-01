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
}

void ToolEstimation::initialize()
{
  kf_estimated_force_torque = std::make_shared<KalmanFilter>();

  kf_estimated_contact_fx_ = std::make_shared<KalmanFilter>();
  kf_estimated_contact_fy_ = std::make_shared<KalmanFilter>();
  kf_estimated_contact_fz_ = std::make_shared<KalmanFilter>();

  kf_accelerometer         = std::make_shared<KalmanFilter>();
  tool_kinematics          = std::make_shared<Kinematics>();
  tool_statistics_orientation_vel = std::make_shared<Statistics>();
  tool_statistics_orientation_acc = std::make_shared<Statistics>();


  control_time_ = 0.002;
  mass_of_tool_ = 1;

  c1 = 1;
  c2 = 1;
  c3 = 1;
  c4 = 1.7;
  c5 = 1;

  tool_linear_acc_offset_data_.resize(4, 1);
  tool_linear_acc_data_.resize(4, 1);

  inertia_of_tool_.resize(3, 3);
  contacted_force_torque_.resize(6, 1);

  pose_.resize(3,1);
  orientation_.resize(3,1);

  pose_vel_.resize(3,1);
  orientation_vel_.resize(3,1);

  pose_acc_.resize(3,1);
  orientation_acc_.resize(3,1);

  tool_statistics_orientation_vel->set_dimension(orientation_);
  tool_statistics_orientation_acc->set_dimension(orientation_);

  //////////////////////////////////
  tool_linear_acc_offset_data_.fill(0);
  tool_linear_acc_data_.fill(0);

  inertia_of_tool_.fill(0);
  contacted_force_torque_.fill(0);

  pose_.fill(0);
  orientation_.fill(0);

  pose_vel_.fill(0);
  orientation_vel_.fill(0);

  pose_acc_.fill(0);
  orientation_acc_.fill(0);

  //filtered_acc_
  filtered_acc_.resize(3,1);
  filtered_acc_.fill(0);

  angular_acceleration_.resize(3, 1);
  angular_acceleration_.fill(0);

  // force contact model design initialize
  f_F_init_.resize(3, 3);
  f_H_init_.resize(4, 3);
  f_Q_init_.resize(3, 3);
  f_R_init_.resize(4, 4);
  f_B_init_.resize(3, 3);
  f_U_init_.resize(3, 1);
  f_Z_init_.resize(3, 1);

  f_F_init_<<1,control_time_,pow(control_time_,2),
      0,1,control_time_,
      0,0,1;

  f_H_init_.setZero();
  f_H_init_(0,0) = c1;

  f_B_init_<<1,0,pow(control_time_,2)/mass_of_tool_,
      0,0,control_time_/mass_of_tool_,
      0,0,1/mass_of_tool_;

  f_Q_init_.setIdentity();
  f_R_init_.setIdentity();
  f_Z_init_.setZero();
  f_U_init_.setZero();
  f_Q_init_ = f_Q_init_ * 100;
  f_R_init_ = f_R_init_ * 100;

  kf_estimated_contact_fx_->initialize_system(f_F_init_, f_H_init_, f_Q_init_, f_R_init_, f_B_init_, f_U_init_, f_Z_init_);
  kf_estimated_contact_fy_->initialize_system(f_F_init_, f_H_init_, f_Q_init_, f_R_init_, f_B_init_, f_U_init_, f_Z_init_);
  kf_estimated_contact_fz_->initialize_system(f_F_init_, f_H_init_, f_Q_init_, f_R_init_, f_B_init_, f_U_init_, f_Z_init_);

  // observer output define
  pseoudo_inverse_d_f_.resize(1,4);
  pseoudo_inverse_d_f_ << 0,0,0.4698,-0.6711; // must be configurable.

  for(int num = 0; num < 3; num ++)
  {
    estimated_output_data_[num].resize(4, 1);//(position, target_position, external_force, acceleration) sensor fusion estimated output
    measured_output_data_[num].resize(4, 1);
    error_output_data_[num].resize(4, 1);

    estimated_output_data_[num].fill(0);
    measured_output_data_[num].fill(0);
    error_output_data_[num].fill(0);
  }

  // accelerometer initialize
  acc_F_init_.resize(3, 3);
  acc_H_init_.resize(3, 3);
  acc_Q_init_.resize(3, 3);
  acc_R_init_.resize(3, 3);
  acc_B_init_.resize(3, 3);
  acc_U_init_.resize(3, 1);
  acc_Z_init_.resize(3, 1);

  acc_F_init_.setIdentity();
  acc_H_init_.setIdentity();
  acc_Q_init_.setIdentity();
  acc_R_init_.setIdentity();
  acc_B_init_.setZero();
  acc_U_init_.setZero();
  acc_Z_init_.setZero();

  acc_Q_init_ = acc_Q_init_ * 0.001;
  acc_R_init_ = acc_R_init_ * 500;

  kf_accelerometer->initialize_system(acc_F_init_, acc_H_init_, acc_Q_init_, acc_R_init_, acc_B_init_, acc_U_init_,
      acc_Z_init_);
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
void ToolEstimation::set_pose_input_data(Eigen::MatrixXd pose)
{
  for(int num = 0; num < 3 ; num ++)
  {
    pose_(num,0) = pose(num,0);
  }

  orientation_ = tool_kinematics->get_axis_to_euler_angle(pose(3,0),pose(4,0),pose(5,0));
}
void ToolEstimation::set_speed_input_data(Eigen::MatrixXd speed)
{
  for(int num = 0; num < 3 ; num ++)
  {
    pose_vel_(num,0) = speed(num,0);
  }

  orientation_vel_ = tool_kinematics->get_axis_to_euler_angle(speed(3,0),speed(4,0),speed(5,0));

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

Eigen::MatrixXd ToolEstimation::get_contacted_force(Eigen::MatrixXd position_data,
    Eigen::MatrixXd target_position_data,
    Eigen::MatrixXd ft_data,
    Eigen::MatrixXd linear_acc_data)  // input entire force torque
{
  kf_accelerometer->process_kalman_filtered_data(linear_acc_data);

  linear_acc_data = kf_accelerometer->get_estimated_state();

  //output update
  for(int num = 0; num < 3 ; num ++)
  {
    measured_output_data_[num] << c1*position_data(num,0),
        c2*target_position_data(num,0),
        c3*linear_acc_data(num,0),
        c5*linear_acc_data(num,0);
    estimated_output_data_[num]<< 0,
        c2*target_position_data(num,0),
        (c3/mass_of_tool_)*ft_data(num,0),
        (c5/mass_of_tool_)*ft_data(num,0);

    f_U_init_(num,0) = ft_data(num,0);
    //error_output_data_[num] = measured_output_data_[num] - estimated_output_data_[num];
  }

  //calculate contact force
  f_U_init_ << 0,0,ft_data(0,0);
  kf_estimated_contact_fx_->set_system_input_u(f_U_init_);

  f_U_init_ << 0,0,ft_data(1,0);
  kf_estimated_contact_fy_->set_system_input_u(f_U_init_);

  f_U_init_ << 0,0,ft_data(2,0);
  kf_estimated_contact_fz_->set_system_input_u(f_U_init_);

  kf_estimated_contact_fx_->set_addtional_estimated_y_term(estimated_output_data_[0]);
  kf_estimated_contact_fy_->set_addtional_estimated_y_term(estimated_output_data_[1]);
  kf_estimated_contact_fz_->set_addtional_estimated_y_term(estimated_output_data_[2]);

  kf_estimated_contact_fx_->process_kalman_filtered_data(measured_output_data_[0]);
  kf_estimated_contact_fy_->process_kalman_filtered_data(measured_output_data_[1]);
  kf_estimated_contact_fz_->process_kalman_filtered_data(measured_output_data_[2]);

  contacted_force_torque_(0, 0) = (pseoudo_inverse_d_f_* kf_estimated_contact_fx_->get_output_error())(0,0);
  contacted_force_torque_(1, 0) = (pseoudo_inverse_d_f_* kf_estimated_contact_fy_->get_output_error())(0,0);
  contacted_force_torque_(2, 0) = (pseoudo_inverse_d_f_* kf_estimated_contact_fz_->get_output_error())(0,0);

  //contacted_force_torque_(0, 0) = (pseoudo_inverse_d_f_*kf_estimated_contact_fx_->get_output_error())(0,0);


  //contacted_force_torque_(3,0) = (inertia_of_tool_(0,0) * angular_acceleration_(0,0) + inertia_of_tool_(0,1) * angular_acceleration_(1,0) + inertia_of_tool_(0,2) * angular_acceleration_(2,0));
  //contacted_force_torque_(4,0) = (inertia_of_tool_(1,0) * angular_acceleration_(0,0) + inertia_of_tool_(1,1) * angular_acceleration_(1,0) + inertia_of_tool_(1,2) * angular_acceleration_(2,0));
  //contacted_force_torque_(5,0) = (inertia_of_tool_(2,0) * angular_acceleration_(0,0) + inertia_of_tool_(2,1) * angular_acceleration_(1,0) + inertia_of_tool_(2,2) * angular_acceleration_(2,0));

  //contacted_force_torque_ = kf_estimated_contact_ft->get_kalman_filtered_data(ft_data);

  return contacted_force_torque_;
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
  orientation_acc_ = tool_statistics_orientation_acc->calculate_diff(orientation_vel_, control_time_);

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
