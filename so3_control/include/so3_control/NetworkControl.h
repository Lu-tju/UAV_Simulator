#ifndef NETWORK_CONTROL_H_
#define NETWORK_CONTROL_H_

#include <Eigen/Eigen>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <so3_control/SO3Control.h>
#include "so3_control/HGDO.h"
#include <string>
#include <iostream>
#include <fstream>

#define ONE_G 9.81

class NetworkControl
{
public:
    NetworkControl(ros::NodeHandle &node){
        nh_ = node;

        so3_controller_.setMass(mass_);
        disturbance_observer_ = HGDO(control_dt_);
        
        nh_.param("use_disturbance_observer", use_disturbance_observer_, false);
        nh_.param("init_state_x", init_x_, 0.0);
        nh_.param("init_state_y", init_y_, 0.0);
        nh_.param("init_state_z", init_z_, 2.0);
        nh_.param("record_log", record_log_, false);
        nh_.param("logger_file_name", logger_file_name, std::string("/home/lu/"));

        so3_command_pub_ = nh_.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
        position_cmd_sub_ = nh_.subscribe("position_cmd", 1, &NetworkControl::network_cmd_callback, this, ros::TransportHints().tcpNoDelay());
        odom_sub_ = nh_.subscribe("odom", 1, &NetworkControl::odom_callback, this, ros::TransportHints().tcpNoDelay());
        imu_sub_ = nh_.subscribe("imu", 1, &NetworkControl::imu_callback, this, ros::TransportHints().tcpNoDelay());

        if (record_log_)
            initLogRecorder();
    };

    ~NetworkControl(){};

private:
    ros::NodeHandle nh_;
    ros::Publisher so3_command_pub_;
    ros::Subscriber position_cmd_sub_, odom_sub_, imu_sub_;

    double mass_ = 0.98;
    double control_dt_ = 0.02;
    double cur_yaw_ = 0;

    Eigen::Vector3d cur_vel_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d cur_acc_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d dis_acc_ = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d last_des_acc_ = Eigen::Vector3d(0, 0, 0);

    bool position_cmd_init_ = false;
    bool takeoff_cmd_init_ = false;
    bool use_disturbance_observer_ = false;
    
    double init_x_, init_y_, init_z_;
    SO3Control so3_controller_;
    HGDO disturbance_observer_;
    bool record_log_ = false;
    std::ofstream logger;
    std::string logger_file_name;

    void initLogRecorder();

    void recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw);

    Eigen::Vector3d publishHoverSO3Command(Eigen::Vector3d des_pos_, Eigen::Vector3d des_vel_, Eigen::Vector3d des_acc_, double des_yaw_, double des_yaw_dot_);

    void get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des);

    void pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw);

    void network_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd);

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom);

    void imu_callback(const sensor_msgs::Imu &imu);
};

#endif