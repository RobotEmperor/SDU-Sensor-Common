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
    void set_input_data(Eigen::MatrixXd linear_acc_input);
    Eigen::MatrixXd get_offset_data();
    Eigen::MatrixXd get_contacted_force(Eigen::MatrixXd ft_data, Eigen::MatrixXd linear_acc_data); // 6*1 data fx fy fz tx ty tz

    //filter
    std::shared_ptr<KalmanFilter> kf_estimated_contact_ft;

  private:
    double control_time;
    double mass_of_tool;

    Eigen::MatrixXd inertia_of_tool;

    Eigen::MatrixXd tool_linear_acc_data;
    Eigen::MatrixXd tool_linear_acc_offset_data;

    Eigen::MatrixXd contacted_force_torque;
    Eigen::MatrixXd estimated_data;

    Eigen::MatrixXd F_init;
    Eigen::MatrixXd H_init;
    Eigen::MatrixXd Q_init;
    Eigen::MatrixXd R_init;
    Eigen::MatrixXd B_init;
    Eigen::MatrixXd U_init;
    Eigen::MatrixXd Z_init;
};
#endif /* TOOL_ESTIMATION_H_ */
