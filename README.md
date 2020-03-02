# SDU-Sensor-Common
This package provides sensor filtering and signal processing libraries for sensors used at SDU.

## Introduction ##
SDU-Sensor-Common/sensor_filter includes the following filters available: 

* 1st-order Low Pass Filter 

* 1st-order High Pass Filter

* Kalman Filter

* Kalman Bucy Filter 

SDU-Sensor-Common/ft_sensor includes the following functions available:

* Signal Processing (Kalman filter)
* Adaptive gain algorithm (can change the kalman filter sensor noise gain R depending on the rate of changes of raw sensor data)
* Offset Tunner
* A filter gain file load using YAML 
* Collision Detection

SDU-Sensor-Common/tool_estimation includes the following functions available:

* Tool's pose, force and torque estimation (Future works) 

  Generally, we need to filter a sensor's value when we want to control a robot stably. These libraries offer for you to use basic sensor filters easily. In addition, the sdu_sensor library is suitable for any F/T sensor to filter and process sensor signals. The library also includes various basic algorithms (for example, collision detection, tool's pose estimation) to do industrial tasks. The final point is that you can create filter/estimation algorithms and easily verify the algorithm by using these libraries.

## Dependencies ##
* [Eigen3] (http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [YAML] (https://yaml.org/)
* [SDU-Math-Common] (https://github.com/RobotEmperor/SDU-Math-Common.git)

## Compatible Operating Systems ##
  Currently tested on:

* Ubuntu 16.04 (Xenial Xerus)
* Ubuntu 18.04 (Bionic Beaver)

## Build ##
  Currently tested on:

* Cmake 

## Installation (Build Instruction) ##

  SDU-Sensor-Common

    git clone https://github.com/RobotEmperor/SDU-Sensor-Common.git
    cd SDU-Sensor-Common
    mkdir build
    cd build
    cmake ..
    make 
    sudo make install 
    
  These libraries are installed in /usr/local/lib and the headers are in /usr/local/include. The python bindings are also installed to your python site-package folder when running sudo make install.
  
## Cmake Example ## 

    cmake_minimum_required(VERSION 3.5)
    project(your_project)
    
    link_directories(/usr/local/lib)
    add_executable(your_project main.cpp)
    target_link_libraries(your_project sdu_sensor sdu_math)


## Classes and Functions ##

  ### sensor_filter ###

* LowPassFilter & HighPassFilter 

  Firstly, declare a new filter class. You have to set control_time and cutoff_frequency variables in these two filter classes. And then just call the following function.

For example(for LowPassFilter), 

    std::shared_ptr<LowPassFilter> lpf;
    lpf = std::make_shared<LowPassFilter>();
    low_pass_filter_ft->set_parameters(0.002, 30, raw_data)//(control_time, lpf_force_cutoff_frequency, raw_data);
    //raw_data in form of Eigen::MatrixXd --> This instance is for using to fit internal matrix dimension.
    
    //in the control loop 
    low_pass_filter_ft->get_lpf_filtered_data(data that you want to filter in form of Eigen::MatrixXd); // It returns Eigen::MartixXd and you can use it. 
    
    

* KalmanFilter & KalmanBucyFilter

  To use the class, you must design your system model in state space equation and then define matrix F,H,Q,R. And then define state_variables and measurement_variables in matrix form by using Eigen::MatrixXd. 
  
For example(for KalmanFilter), 

    std::shared_ptr<KalmanFilter> kalman_filter;
    kalman_filter = std::make_shared<KalmanFilter>();
    kalman_filter->initialize(state_variables, measurement_variables); 
    
    //this example is about only force torque sensor itself, so F,H,Q set identity matrix.
    // design system
    Eigen::MatrixXd F_init;
    Eigen::MatrixXd H_init;
    Eigen::MatrixXd Q_init;
    Eigen::MatrixXd R_init;
    Eigen::MatrixXd B_init;
    Eigen::MatrixXd U_init;
    Eigen::MatrixXd Z_init;
    
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
    
    kalman_filter_force_torque->initialize_system(F_init,H_init,Q_init,R_init,B_init,U_init,Z_init);
    
    // ** if you have specific system model, you have to design F H Q R variables in matrix form. 
    
    //in the control loop 
    KalmanFilter->get_kalman_filtered_data(data that you want to filter); // this function returns to Eigen::MatrixXd
    // output filtered data in matrix form
    // in here, the output is force X, force Y, Force Z, Torque X, Torque Y, Torque Z in 6 X 1 matrix form
    
    
  ### ft_filter ###

  FTfilter
  
  This library is for using F/T sensor easily. It uses the sensor_filter library to filter raw force/torque data. It also offers important data which can be used in control algorithm (for example, collision detection and tool estimation) 
  
  * parse_init_data(const std::string &path)
  
    To load sensor filter gains, you can insert the path of init_data.yaml.
  
  * initialize()
    
    Variables are initialized in this function. 
    
  * offset_init(Eigen::MatrixXd data, bool time_check)
  
    This function is to get offest value by sampling raw data of FT sensor for a certain period of time.
    
  * filter_processing(Eigen::MatrixXd data) 
  
    Filter algorithms are included.(adaptive gain algorithm)
    
  * collision_detection(Eigen::MatrixXd data)
  
    This function can detect collision from raw force torque data by using CUSUM method (it was included in sdu_math library). It returns int value 1 and -1 when collision is detected. (Value 0 is default and non-contact)
    
  * std::vector<double> FTfilter::get_filtered_data()
    
    you can get filtered data from the kalman filter system which designed by user.
    
  * std::vector<double> FTfilter::get_offset_data()
  
    it is for getting ft sensor offset data.
  
  * std::vector<double> FTfilter::get_collision_detection_data()
  
    it is for getting tool's collision detection signal (1 -> collision occurred/ 0 -> collision does not occur)
  
  * std::vector<double> FTfilter::get_contact_force_data()
  
    it is for getting contact force data. 
    
For example (How to use the library)

    std::shared_ptr<FTfilter> ft_filter;
    ft_filter = std::make_shared<FTfilter>();
    
    std::string init_data_path; // it is to load config file.
    init_data_path = "../config/init_data.yaml"; // it must be in your project.
    
    ft_filter->initialize(init_data_path);
    
    
    //in the control loop
  
    ft_filter->filter_processing(raw_force_torque_data); //
    ft_filter->get_offset_data(); //you can get filtered data
    
For example (Load gain file YAML)

    std::shared_ptr<FTfilter> ft_filter;
    ft_filter = std::make_shared<FTfilter>();
    
    std::string init_data_path; // it is to load config file.
    init_data_path = "../config/init_data.yaml"; // it must be in your project.
   
    ft_filter->initialize(init_data_path); 
