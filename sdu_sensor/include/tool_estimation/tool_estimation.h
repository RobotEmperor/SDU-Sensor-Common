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
    void estimation_processing(Eigen::MatrixXd data); // 6*1 data fx fy fz tx ty tz
    void set_parameters(double control_time_init, double mass_of_tool_init);

    std::vector<double> get_contact_force_data();

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
};



#endif /* TOOL_ESTIMATION_H_ */
