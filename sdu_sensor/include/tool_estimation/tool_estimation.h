/*
 * tool_estimation.h
 *
 *  Created on: Feb 12, 2020
 *      Author: yik
 */

#ifndef TOOL_ESTIMATION_H_
#define TOOL_ESTIMATION_H_

#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "sdu_sensor/sensor_filter.h"
#include "sdu_math/kinematics.h"
#include "sdu_math/statistics_math.h"

class ToolEstimation
{
  public:
    ToolEstimation();
    ~ToolEstimation();
    void initialize();
    void offset_init(Eigen::MatrixXd data, int desired_sample_num);
    void set_parameters(double control_time_init, double mass_of_tool_init);
    void set_acc_input_data(Eigen::MatrixXd linear_acc_input);
    Eigen::MatrixXd get_offset_data();
    Eigen::MatrixXd get_angular_acc();
    Eigen::MatrixXd get_one_axis_inertia_tensor(Eigen::MatrixXd ft_data, std::string axis);

    Eigen::MatrixXd get_contacted_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data); // 6*1 data fx fy fz tx ty tz

    //filter
    std::shared_ptr<KalmanFilter> kf_estimated_contact_ft;

    std::shared_ptr<KalmanFilter> kf_accelerometer;




  private:
    double control_time_;
    double mass_of_tool_;

    Eigen::MatrixXd inertia_of_tool_;

    Eigen::MatrixXd tool_linear_acc_data_;
    Eigen::MatrixXd tool_linear_acc_offset_data_;

    Eigen::MatrixXd contacted_force_torque_;
    Eigen::MatrixXd estimated_data_;

    Eigen::MatrixXd filtered_acc_;


    //ft sensor
    Eigen::MatrixXd ft_F_init_;
    Eigen::MatrixXd ft_H_init_;
    Eigen::MatrixXd ft_Q_init_;
    Eigen::MatrixXd ft_R_init_;
    Eigen::MatrixXd ft_B_init_;
    Eigen::MatrixXd ft_U_init_;
    Eigen::MatrixXd ft_Z_init_;

    //acc
    Eigen::MatrixXd acc_F_init_;
    Eigen::MatrixXd acc_H_init_;
    Eigen::MatrixXd acc_Q_init_;
    Eigen::MatrixXd acc_R_init_;
    Eigen::MatrixXd acc_B_init_;
    Eigen::MatrixXd acc_U_init_;
    Eigen::MatrixXd acc_Z_init_;
};
#endif /* TOOL_ESTIMATION_H_ */
