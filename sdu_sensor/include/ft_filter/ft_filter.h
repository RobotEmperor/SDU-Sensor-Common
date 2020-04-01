/*
 * ft_filter.h
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */
#ifndef FT_FILTER_H_
#define FT_FILTER_H_

#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "sdu_sensor/sensor_filter.h"
#include "sdu_math/kinematics.h"
#include "sdu_math/statistics_math.h"

class FTfilter
{
  public:
    FTfilter();
    ~FTfilter();
    void initialize(const std::string &path);
    void offset_init(Eigen::MatrixXd data, int desired_sample_num);
    void filter_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz
    void parse_init_data(const std::string &path);
    void collision_detection_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz

    Eigen::MatrixXd get_filtered_data();
    Eigen::MatrixXd get_offset_data();
    Eigen::MatrixXd get_collision_detection_data();
    Eigen::MatrixXd get_contact_force_data();

    //filter will be added
    std::shared_ptr<LowPassFilter>  low_pass_filter_ft;
    std::shared_ptr<HighPassFilter> high_pass_filter_ft;

    std::shared_ptr<KalmanFilter> kalman_filter_force_torque;

  private:

    double control_time_;
    double mass_of_tool_;
    double lpf_force_cutoff_frequency_;
    double lpf_torque_cutoff_frequency_;

    double hpf_force_cutoff_frequency_;
    double hpf_torque_cutoff_frequency_;

    double fx_detection_, fx_k_, fx_high_limit_, fx_low_limit_;
    double fy_detection_, fy_k_, fy_high_limit_, fy_low_limit_;
    double fz_detection_, fz_k_, fz_high_limit_, fz_low_limit_;
    double tx_detection_, tx_k_, tx_high_limit_, tx_low_limit_;
    double ty_detection_, ty_k_, ty_high_limit_, ty_low_limit_;
    double tz_detection_, tz_k_, tz_high_limit_, tz_low_limit_;

    double gain_q_;
    double gain_r_low_frequency_;
    double gain_r_high_frequency_;

    double gain_r_torque_low_frequency_;
    double gain_r_torque_high_frequency_;

    double limit_low_rate_of_change_;
    double limit_high_rate_of_change_;


    Eigen::MatrixXd ft_filtered_data_;
    Eigen::MatrixXd ft_offset_data_;
    Eigen::MatrixXd ft_raw_data_;

    Eigen::MatrixXd collision_detection_;

    Eigen::MatrixXd pre_ft_filtered_data_;
    Eigen::MatrixXd rate_of_change_ft_filtered_data_;

    Eigen::MatrixXd F_init_;
    Eigen::MatrixXd H_init_;
    Eigen::MatrixXd Q_init_;
    Eigen::MatrixXd R_init_;
    Eigen::MatrixXd B_init_;
    Eigen::MatrixXd U_init_;
    Eigen::MatrixXd Z_init_;
};
#endif /* FT_FILTER_H_ */



