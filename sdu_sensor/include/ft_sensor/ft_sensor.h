/*
 * ft_sensor.h
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */
#ifndef FT_SENSOR_H_
#define FT_SENSOR_H_

#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>


#include "sdu_sensor/sensor_filter.h"
#include "sdu_sensor/tool_estimation.h"
#include "sdu_math/kinematics.h"
#include "sdu_math/statistics_math.h"

class FTsensor
{
  public:
    FTsensor();
    ~FTsensor();
    void initialize(const std::string &path);
    void offset_init(Eigen::MatrixXd data, int desired_sample_num);
    void filter_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz
    void parse_init_data(const std::string &path);
    void collision_detection_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz

    std::vector<double> get_filtered_data();
    std::vector<double> get_offset_data();
    std::vector<double> get_collision_detection_data();
    std::vector<double> get_contact_force_data();

    //filter will be added
    std::shared_ptr<LowPassFilter>  low_pass_filter_ft;
    std::shared_ptr<HighPassFilter> high_pass_filter_ft;

    std::shared_ptr<KalmanFilter> kalman_filter_force_torque;
    std::shared_ptr<KalmanFilter> ft_contact;

    std::shared_ptr<ToolEstimation> tool_estimation;

  private:

    double control_time;
    double mass_of_tool;
    double lpf_force_cutoff_frequency;
    double lpf_torque_cutoff_frequency;

    double hpf_force_cutoff_frequency;
    double hpf_torque_cutoff_frequency;

    double fx_detection, fx_k, fx_high_limit, fx_low_limit;
    double fy_detection, fy_k, fy_high_limit, fy_low_limit;
    double fz_detection, fz_k, fz_high_limit, fz_low_limit;
    double tx_detection, tx_k, tx_high_limit, tx_low_limit;
    double ty_detection, ty_k, ty_high_limit, ty_low_limit;
    double tz_detection, tz_k, tz_high_limit, tz_low_limit;

    double gain_q;
    double gain_r_low_frequency;
    double gain_r_high_frequency;

    double gain_r_torque_low_frequency;
    double gain_r_torque_high_frequency;

    double limit_low_rate_of_change;
    double limit_high_rate_of_change;


    Eigen::MatrixXd ft_filtered_data;
    Eigen::MatrixXd ft_offset_data;
    Eigen::MatrixXd ft_raw_data;

    Eigen::MatrixXd pre_ft_filtered_data;
    Eigen::MatrixXd rate_of_change_ft_filtered_data;

    Eigen::MatrixXd F_init;
    Eigen::MatrixXd H_init;
    Eigen::MatrixXd Q_init;
    Eigen::MatrixXd R_init;
    Eigen::MatrixXd B_init;
    Eigen::MatrixXd U_init;
    Eigen::MatrixXd Z_init;
};



#endif /* FT_SENSOR_H_ */



