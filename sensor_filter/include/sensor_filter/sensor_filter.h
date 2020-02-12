/*
 * sensor_filter.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef SDU_UR10E_SENSOR_SENSOR_FILTER_INCLUDE_SENSOR_FILTER_SENSOR_FILTER_H_
#define SDU_UR10E_SENSOR_SENSOR_FILTER_INCLUDE_SENSOR_FILTER_SENSOR_FILTER_H_

#include <stdio.h>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include "sdu_math/statistics_math.h"


class LowPassFilter // lpf
{
  public:
    LowPassFilter();
    LowPassFilter(double control_time_init, double cutoff_frequency_init);
    ~LowPassFilter();
    void initialize();
    void set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::MatrixXd data);
    Eigen::MatrixXd get_lpf_filtered_data(Eigen::MatrixXd data); // frq = frequency , ctrl = control

  private:
    double control_time;
    double cutoff_frequency;
    double raw_data;
    double lambda;
    double alpha;

    Eigen::MatrixXd filtered_data;
    Eigen::MatrixXd pre_filtered_data;
};

class HighPassFilter // hpf
{
  public:
    HighPassFilter();
    HighPassFilter(double control_time_init, double cutoff_frequency_init);
    ~HighPassFilter();
    void initialize();
    void set_parameters(double control_time_init, double cutoff_frequency_init, Eigen::MatrixXd data);
    Eigen::MatrixXd get_hpf_filtered_data(Eigen::MatrixXd data); // frq = frequency , ctrl = control

  private:
    double control_time;
    double cutoff_frequency;
    double raw_data;
    double lambda;
    double alpha;

    Eigen::MatrixXd filtered_data;
    Eigen::MatrixXd pre_filtered_data;
    Eigen::MatrixXd pre_input_data;
};



class KalmanFilter // kf
{
  public:
    KalmanFilter();
    ~KalmanFilter();
    void initialize_system(Eigen::MatrixXd F_init, Eigen::MatrixXd H_init,
      Eigen::MatrixXd Q_init, Eigen::MatrixXd R_init,
      Eigen::MatrixXd B_init, Eigen::MatrixXd U_init,
      Eigen::MatrixXd Z_init);

    void change_noise_value(Eigen::MatrixXd R_init);

    //kalman filter process
    Eigen::MatrixXd get_kalman_filtered_data(Eigen::MatrixXd measurement_z);

  private:
    // must be designed by your system model
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    Eigen::MatrixXd B;
    Eigen::MatrixXd U;
    Eigen::MatrixXd Z;

    //intial condition
    Eigen::MatrixXd correction_value_x;
    Eigen::MatrixXd correction_value_p;


    //variables
    Eigen::MatrixXd prediction_value_x;
    Eigen::MatrixXd prediction_value_p;

    Eigen::MatrixXd previous_correction_value_x;
    Eigen::MatrixXd previous_correction_value_p;

    //kalman gain
    Eigen::MatrixXd kalman_gain_k;

    double raw_data;

};

class KalmanBucyFilter // kbf
{
  public:
    KalmanBucyFilter();
    ~KalmanBucyFilter();
    void initialize(Eigen::MatrixXd state_variables, Eigen::MatrixXd measurement_variables);
    // must be designed by your system model

    double control_time;

    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    Eigen::MatrixXd estimated_value_x;
    Eigen::MatrixXd estimated_value_p;


    Eigen::MatrixXd kalman_bucy_filtering_processing(Eigen::MatrixXd measurement_z);

  private:
    //variables

    Eigen::MatrixXd dt_value_x;
    Eigen::MatrixXd dt_value_p;

    Eigen::MatrixXd pre_dt_value_x;
    Eigen::MatrixXd pre_dt_value_p;

    //kalman gain
    Eigen::MatrixXd kalman_gain_k;

    double raw_data;

};
#endif /* SDU_UR10E_SENSOR_SENSOR_FILTER_INCLUDE_SENSOR_FILTER_SENSOR_FILTER_H_ */
