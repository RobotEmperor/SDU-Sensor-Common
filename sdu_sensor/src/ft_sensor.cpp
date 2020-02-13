/*
 * ft_sensor.cpp
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */

#include "ft_sensor/ft_sensor.h"

#define DEBUG_OUTPUT false

#if DEBUG_OUTPUT
#define DEBUG(a)                                                \
  {                                                             \
  std::cout << "FTsensor:" << __LINE__ << ": " << a << std::endl; \
  }
#else
#define DEBUG(a) \
  {              \
  }
#endif

FTsensor::FTsensor()
{
  low_pass_filter_ft  = std::make_shared<LowPassFilter>();
  high_pass_filter_ft = std::make_shared<HighPassFilter>();

  kalman_filter_force_torque = std::make_shared<KalmanFilter>();

  control_time = 0;
  mass_of_tool = 0;
  lpf_force_cutoff_frequency = 0;
  lpf_torque_cutoff_frequency = 0;
  hpf_force_cutoff_frequency  = 0;
  hpf_torque_cutoff_frequency = 0;

  gain_q = 0;
  gain_r_low_frequency = 0;
  gain_r_high_frequency = 0;

  gain_r_torque_low_frequency = 0;
  gain_r_torque_high_frequency = 0;

  limit_low_rate_of_change = 0;
  limit_high_rate_of_change = 0;

  fx_detection = 0; fx_k = 0; fx_high_limit = 0; fx_low_limit = 0;
  fy_detection = 0; fy_k = 0; fy_high_limit = 0; fy_low_limit = 0;
  fz_detection = 0; fz_k = 0; fz_high_limit = 0; fz_low_limit = 0;
  tx_detection = 0; tx_k = 0; tx_high_limit = 0; tx_low_limit = 0;
  ty_detection = 0; ty_k = 0; ty_high_limit = 0; ty_low_limit = 0;
  tz_detection = 0; tz_k = 0; tz_high_limit = 0; tz_low_limit = 0;
}

FTsensor::~FTsensor()
{
}
void FTsensor::parse_init_data(const std::string &path)
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
  control_time = doc["control_time"].as<double>();
  mass_of_tool = doc["mass_of_tool"].as<double>();
  lpf_force_cutoff_frequency = doc["lpf_force_cutoff_frequency"].as<double>();
  lpf_torque_cutoff_frequency = doc["lpf_torque_cutoff_frequency"].as<double>();

  hpf_force_cutoff_frequency = doc["hpf_force_cutoff_frequency"].as<double>();
  hpf_torque_cutoff_frequency = doc["hpf_torque_cutoff_frequency"].as<double>();

  gain_q = doc["gain_Q"].as<double>();
  gain_r_low_frequency = doc["gain_R_low_frequency"].as<double>();
  gain_r_high_frequency = doc["gain_R_high_frequency"].as<double>();

  gain_r_torque_low_frequency = doc["gain_R_torque_low_frequency"].as<double>();
  gain_r_torque_high_frequency = doc["gain_R_torque_high_frequency"].as<double>();

  limit_low_rate_of_change = doc["limit_low_rate_of_change"].as<double>();
  limit_high_rate_of_change = doc["limit_high_rate_of_change"].as<double>();

  fx_k = doc["fx_k"].as<double>();
  fy_k = doc["fy_k"].as<double>();
  fz_k = doc["fz_k"].as<double>();
  tx_k = doc["tx_k"].as<double>();
  ty_k = doc["ty_k"].as<double>();
  tz_k = doc["tz_k"].as<double>();

  fx_high_limit = doc["fx_high_limit"].as<double>();
  fy_high_limit = doc["fy_high_limit"].as<double>();
  fz_high_limit = doc["fz_high_limit"].as<double>();
  tx_high_limit = doc["tx_high_limit"].as<double>();
  ty_high_limit = doc["ty_high_limit"].as<double>();
  tz_high_limit = doc["tz_high_limit"].as<double>();

  fx_low_limit = doc["fx_low_limit"].as<double>();
  fy_low_limit = doc["fy_low_limit"].as<double>();
  fz_low_limit = doc["fz_low_limit"].as<double>();
  tx_low_limit = doc["tx_low_limit"].as<double>();
  ty_low_limit = doc["ty_low_limit"].as<double>();
  tz_low_limit = doc["tz_low_limit"].as<double>();
}

void FTsensor::initialize(const std::string &path)
{
  parse_init_data(path);

  ft_raw_data.resize(6,1);
  ft_raw_data.fill(0);

  ft_filtered_data.resize(6, 1);
  ft_filtered_data.fill(0);

  pre_ft_filtered_data.resize(6,1);
  pre_ft_filtered_data.fill(0);

  rate_of_change_ft_filtered_data.resize(6,1);
  rate_of_change_ft_filtered_data.fill(0);

  ft_offset_data.resize(6,1);
  ft_offset_data.fill(0);

  collision_detection.resize(6,1);
  collision_detection.fill(0);

  low_pass_filter_ft->set_parameters(control_time, lpf_force_cutoff_frequency,ft_raw_data);
  high_pass_filter_ft->set_parameters(control_time, hpf_force_cutoff_frequency,ft_raw_data);

  low_pass_filter_ft->initialize();
  high_pass_filter_ft->initialize();

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

  Q_init = Q_init * gain_q;
  R_init = R_init * gain_r_low_frequency;

  kalman_filter_force_torque->initialize_system(F_init,H_init,Q_init,R_init,B_init,U_init,Z_init);
}
void FTsensor::offset_init(Eigen::MatrixXd data, int desired_sample_num)
{
  static int sample_num = 1;

  ft_offset_data += data;

  if(sample_num == desired_sample_num)
  {
    ft_offset_data = ft_offset_data/(sample_num);
    sample_num = 1;
    return;
  }
  sample_num ++;
}

void FTsensor::filter_processing(Eigen::MatrixXd data)
{
  //kalman filer

  data = data - ft_offset_data;
  ft_filtered_data = kalman_filter_force_torque->get_kalman_filtered_data(data);
  collision_detection_processing(data);

  rate_of_change_ft_filtered_data = (ft_filtered_data - pre_ft_filtered_data)*(1/control_time);

  pre_ft_filtered_data = ft_filtered_data;

  rate_of_change_ft_filtered_data = low_pass_filter_ft->get_lpf_filtered_data(rate_of_change_ft_filtered_data);

  for(int num = 0; num < 3; num ++)
  {
    if(rate_of_change_ft_filtered_data(num,0) >  limit_high_rate_of_change
      || rate_of_change_ft_filtered_data(num,0) < limit_low_rate_of_change)
    {
      R_init(num,num) = gain_r_high_frequency;
      kalman_filter_force_torque->change_noise_value(R_init);
    }
    else
    {
      R_init(num,num) = gain_r_low_frequency;
      kalman_filter_force_torque->change_noise_value(R_init);
    }
  }
  for(int num = 3; num < 6; num ++)
  {
    if(rate_of_change_ft_filtered_data(num,0) >  limit_high_rate_of_change*0.15
      || rate_of_change_ft_filtered_data(num,0) < limit_low_rate_of_change*0.15)
    {
      R_init(num,num) = gain_r_torque_high_frequency;
      kalman_filter_force_torque->change_noise_value(R_init);
    }
    else
    {
      R_init(num,num) = gain_r_torque_low_frequency;
      kalman_filter_force_torque->change_noise_value(R_init);
    }
  }
}

// collision detection
void FTsensor::collision_detection_processing(Eigen::MatrixXd data)
{
  fx_detection = calculate_cusum(data(0,0),fx_k,fx_high_limit,fx_low_limit); // how to decide k, limit
  fy_detection = calculate_cusum(data(1,0),fy_k,fy_high_limit,fy_low_limit);
  fz_detection = calculate_cusum(data(2,0),fz_k,fz_high_limit,fz_low_limit);
  tx_detection = calculate_cusum(data(3,0),tx_k,tx_high_limit,tx_low_limit);
  ty_detection = calculate_cusum(data(4,0),ty_k,ty_high_limit,ty_low_limit);
  tz_detection = calculate_cusum(data(5,0),tz_k,tz_high_limit,tz_low_limit);
}
Eigen::MatrixXd FTsensor::get_filtered_data()
{
  return ft_filtered_data;
}
Eigen::MatrixXd FTsensor::get_offset_data()
{
  return ft_offset_data;
}
Eigen::MatrixXd FTsensor::get_collision_detection_data()
{
  collision_detection(0,0) = fx_detection;
  collision_detection(1,0) = fy_detection;
  collision_detection(2,0) = fz_detection;
  collision_detection(3,0) = tx_detection;
  collision_detection(4,0) = ty_detection;
  collision_detection(5,0) = tz_detection;

  return collision_detection;
}






