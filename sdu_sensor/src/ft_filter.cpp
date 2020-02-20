/*
 * ft_sensor.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */

#include "ft_filter/ft_filter.h"

#define DEBUG_OUTPUT false

#if DEBUG_OUTPUT
#define DEBUG(a)                                                \
  {                                                             \
  std::cout << "FTfilter:" << __LINE__ << ": " << a << std::endl; \
  }
#else
#define DEBUG(a) \
  {              \
  }
#endif

FTfilter::FTfilter()
{
  low_pass_filter_ft  = std::make_shared<LowPassFilter>();
  high_pass_filter_ft = std::make_shared<HighPassFilter>();

  kalman_filter_force_torque = std::make_shared<KalmanFilter>();

  control_time_ = 0;
  mass_of_tool_ = 0;
  lpf_force_cutoff_frequency_ = 0;
  lpf_torque_cutoff_frequency_ = 0;
  hpf_force_cutoff_frequency_  = 0;
  hpf_torque_cutoff_frequency_ = 0;

  gain_q_ = 0;
  gain_r_low_frequency_ = 0;
  gain_r_high_frequency_ = 0;

  gain_r_torque_low_frequency_ = 0;
  gain_r_torque_high_frequency_ = 0;

  limit_low_rate_of_change_ = 0;
  limit_high_rate_of_change_ = 0;

  fx_detection_ = 0; fx_k_ = 0; fx_high_limit_ = 0; fx_low_limit_ = 0;
  fy_detection_ = 0; fy_k_ = 0; fy_high_limit_ = 0; fy_low_limit_ = 0;
  fz_detection_ = 0; fz_k_ = 0; fz_high_limit_ = 0; fz_low_limit_ = 0;
  tx_detection_ = 0; tx_k_ = 0; tx_high_limit_ = 0; tx_low_limit_ = 0;
  ty_detection_ = 0; ty_k_ = 0; ty_high_limit_ = 0; ty_low_limit_ = 0;
  tz_detection_ = 0; tz_k_ = 0; tz_high_limit_ = 0; tz_low_limit_ = 0;
}

FTfilter::~FTfilter()
{
}
void FTfilter::parse_init_data(const std::string &path)
{
  YAML::Node doc; // YAML file class declare
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str()); // write the directory of file and load file

  }
  catch(const std::exception& e) // check error
  {
    printf("Fail to load yaml file!");
    return;
  }
  control_time_ = doc["control_time"].as<double>();
  mass_of_tool_ = doc["mass_of_tool"].as<double>();
  lpf_force_cutoff_frequency_ = doc["lpf_force_cutoff_frequency"].as<double>();
  lpf_torque_cutoff_frequency_ = doc["lpf_torque_cutoff_frequency"].as<double>();

  hpf_force_cutoff_frequency_ = doc["hpf_force_cutoff_frequency"].as<double>();
  hpf_torque_cutoff_frequency_ = doc["hpf_torque_cutoff_frequency"].as<double>();

  gain_q_ = doc["gain_Q"].as<double>();
  gain_r_low_frequency_ = doc["gain_R_low_frequency"].as<double>();
  gain_r_high_frequency_ = doc["gain_R_high_frequency"].as<double>();

  gain_r_torque_low_frequency_ = doc["gain_R_torque_low_frequency"].as<double>();
  gain_r_torque_high_frequency_ = doc["gain_R_torque_high_frequency"].as<double>();

  limit_low_rate_of_change_ = doc["limit_low_rate_of_change"].as<double>();
  limit_high_rate_of_change_ = doc["limit_high_rate_of_change"].as<double>();

  fx_k_ = doc["fx_k"].as<double>();
  fy_k_ = doc["fy_k"].as<double>();
  fz_k_ = doc["fz_k"].as<double>();
  tx_k_ = doc["tx_k"].as<double>();
  ty_k_ = doc["ty_k"].as<double>();
  tz_k_ = doc["tz_k"].as<double>();

  fx_high_limit_ = doc["fx_high_limit"].as<double>();
  fy_high_limit_ = doc["fy_high_limit"].as<double>();
  fz_high_limit_ = doc["fz_high_limit"].as<double>();
  tx_high_limit_ = doc["tx_high_limit"].as<double>();
  ty_high_limit_ = doc["ty_high_limit"].as<double>();
  tz_high_limit_ = doc["tz_high_limit"].as<double>();

  fx_low_limit_ = doc["fx_low_limit"].as<double>();
  fy_low_limit_ = doc["fy_low_limit"].as<double>();
  fz_low_limit_ = doc["fz_low_limit"].as<double>();
  tx_low_limit_ = doc["tx_low_limit"].as<double>();
  ty_low_limit_ = doc["ty_low_limit"].as<double>();
  tz_low_limit_ = doc["tz_low_limit"].as<double>();
}

void FTfilter::initialize(const std::string &path)
{
  parse_init_data(path);

  ft_raw_data_.resize(6,1);
  ft_raw_data_.fill(0);

  ft_filtered_data_.resize(6, 1);
  ft_filtered_data_.fill(0);

  pre_ft_filtered_data_.resize(6,1);
  pre_ft_filtered_data_.fill(0);

  rate_of_change_ft_filtered_data_.resize(6,1);
  rate_of_change_ft_filtered_data_.fill(0);

  ft_offset_data_.resize(6,1);
  ft_offset_data_.fill(0);

  collision_detection_.resize(6,1);
  collision_detection_.fill(0);

  low_pass_filter_ft->set_parameters(control_time_, lpf_force_cutoff_frequency_,ft_raw_data_);
  high_pass_filter_ft->set_parameters(control_time_, hpf_force_cutoff_frequency_,ft_raw_data_);

  low_pass_filter_ft->initialize();
  high_pass_filter_ft->initialize();

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

  Q_init_ = Q_init_ * gain_q_;
  R_init_ = R_init_ * gain_r_low_frequency_;

  kalman_filter_force_torque->initialize_system(F_init_,H_init_,Q_init_,R_init_,B_init_,U_init_,Z_init_);
}
void FTfilter::offset_init(Eigen::MatrixXd data, int desired_sample_num)
{
  static int sample_num = 1;

  ft_offset_data_ += data;

  if(sample_num == desired_sample_num)
  {
    ft_offset_data_ = ft_offset_data_/(sample_num);
    sample_num = 1;
    return;
  }
  sample_num ++;
}

void FTfilter::filter_processing(Eigen::MatrixXd data)
{
  //kalman filer

  data = data - ft_offset_data_;
  ft_filtered_data_ = kalman_filter_force_torque->get_kalman_filtered_data(data);
  collision_detection_processing(data);

  rate_of_change_ft_filtered_data_ = (ft_filtered_data_ - pre_ft_filtered_data_)*(1/control_time_);

  pre_ft_filtered_data_ = ft_filtered_data_;

  rate_of_change_ft_filtered_data_ = low_pass_filter_ft->get_lpf_filtered_data(rate_of_change_ft_filtered_data_);

  for(int num = 0; num < 3; num ++)
  {
    if(rate_of_change_ft_filtered_data_(num,0) >  limit_high_rate_of_change_
      || rate_of_change_ft_filtered_data_(num,0) < limit_low_rate_of_change_)
    {
      R_init_(num,num) = gain_r_high_frequency_;
      kalman_filter_force_torque->change_noise_value(R_init_);
    }
    else
    {
      R_init_(num,num) = gain_r_low_frequency_;
      kalman_filter_force_torque->change_noise_value(R_init_);
    }
  }
  for(int num = 3; num < 6; num ++)
  {
    if(rate_of_change_ft_filtered_data_(num,0) >  limit_high_rate_of_change_*0.15
      || rate_of_change_ft_filtered_data_(num,0) < limit_low_rate_of_change_*0.15)
    {
      R_init_(num,num) = gain_r_torque_high_frequency_;
      kalman_filter_force_torque->change_noise_value(R_init_);
    }
    else
    {
      R_init_(num,num) = gain_r_torque_low_frequency_;
      kalman_filter_force_torque->change_noise_value(R_init_);
    }
  }
}

// collision detection
void FTfilter::collision_detection_processing(Eigen::MatrixXd data)
{
  fx_detection_ = calculate_cusum(data(0,0),fx_k_,fx_high_limit_,fx_low_limit_); // how to decide k, limit
  fy_detection_ = calculate_cusum(data(1,0),fy_k_,fy_high_limit_,fy_low_limit_);
  fz_detection_ = calculate_cusum(data(2,0),fz_k_,fz_high_limit_,fz_low_limit_);
  tx_detection_ = calculate_cusum(data(3,0),tx_k_,tx_high_limit_,tx_low_limit_);
  ty_detection_ = calculate_cusum(data(4,0),ty_k_,ty_high_limit_,ty_low_limit_);
  tz_detection_ = calculate_cusum(data(5,0),tz_k_,tz_high_limit_,tz_low_limit_);
}
Eigen::MatrixXd FTfilter::get_filtered_data()
{
  return ft_filtered_data_;
}
Eigen::MatrixXd FTfilter::get_offset_data()
{
  return ft_offset_data_;
}
Eigen::MatrixXd FTfilter::get_collision_detection_data()
{
  collision_detection_(0,0) = fx_detection_;
  collision_detection_(1,0) = fy_detection_;
  collision_detection_(2,0) = fz_detection_;
  collision_detection_(3,0) = tx_detection_;
  collision_detection_(4,0) = ty_detection_;
  collision_detection_(5,0) = tz_detection_;

  return collision_detection_;
}






