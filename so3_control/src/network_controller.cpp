/*
    神经网络直接预测期望加速度，这个程序把他转成期望姿态+推力，直接发给仿真器，目前看来并没大区别，但可能加上阻力就变不好了
    注意规划器要从实际状态规划，不能从期望了,否则会逐渐飘走：现在我是位置速度从实际的，加速度从期望的
*/
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

// 他这个控制器收的是绝对的力，px4收的是油门百分比，例如只有重力g时就是0.4（假设40%油门悬停）
double mass_ = 0.98;
double control_dt_ = 0.02;
double cur_yaw_ = 0;
Eigen::Vector3d cur_vel_(0, 0, 0);
Eigen::Vector3d cur_acc_(0, 0, 0);
Eigen::Vector3d dis_acc_(0, 0, 0);
Eigen::Vector3d last_des_acc_(0, 0, 0);
bool position_cmd_init_ = false;
bool takeoff_cmd_init_ = false;
ros::Publisher so3_command_pub_;
ros::Subscriber position_cmd_sub_, odom_sub_, imu_sub_;
double init_x_, init_y_, init_z_;
bool use_disturbance_observer_ = false;
SO3Control so3_controller_;
HGDO disturbance_observer(control_dt_);
bool record_log_ = false;
std::ofstream logger;
std::string logger_file_name;

void initLogRecorder()
{
    std::cout << "logger_file_name: " << logger_file_name << std::endl;
    std::string temp_file_name = logger_file_name + "Net_logger_";
    time_t timep;
    timep = time(0);
    char tmp[64];
    strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
    temp_file_name += tmp;
    temp_file_name += ".csv";
    if (logger.is_open())
    {
        logger.close();
    }
    logger.open(temp_file_name.c_str(), std::ios::out);
    std::cout << "logger: " << temp_file_name << std::endl;
    if (!logger.is_open())
    {
        std::cout << "cannot open the logger." << std::endl;
    }
    else
    {
        logger << "timestamp" << ',';
        logger << "cur_vx" << ',';
        logger << "cur_vy" << ',';
        logger << "cur_vz" << ',';
        logger << "cur_ax" << ',';
        logger << "cur_ay" << ',';
        logger << "cur_az" << ',';
        logger << "des_ax" << ',';
        logger << "des_ay" << ',';
        logger << "des_az" << ',';
        logger << "dis_ax" << ',';
        logger << "dis_ay" << ',';
        logger << "dis_az" << ',';
        logger << "cur_yaw" << ',';
        logger << "des_yaw" << std::endl;
    }
}

void recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a, Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw, double des_yaw)
{
    if (logger.is_open())
    {
        logger << ros::Time::now().toNSec() << ',';
        logger << cur_v(0) << ',';
        logger << cur_v(1) << ',';
        logger << cur_v(2) << ',';
        logger << cur_a(0) << ',';
        logger << cur_a(1) << ',';
        logger << cur_a(2) << ',';
        logger << des_a(0) << ',';
        logger << des_a(1) << ',';
        logger << des_a(2) << ',';
        logger << dis_a(0) << ',';
        logger << dis_a(1) << ',';
        logger << dis_a(2) << ',';
        logger << cur_yaw << ',';
        logger << des_yaw << std::endl;
    }
}

Eigen::Vector3d publishHoverSO3Command(Eigen::Vector3d des_pos_, Eigen::Vector3d des_vel_, Eigen::Vector3d des_acc_, double des_yaw_, double des_yaw_dot_)
{
    Eigen::Vector3d kx_(5.7, 5.7, 6.2);
    Eigen::Vector3d kv_(3.4, 3.4, 4.0);
    so3_controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, des_yaw_dot_, kx_, kv_);

    const Eigen::Vector3d &force = so3_controller_.getComputedForce();
    const Eigen::Quaterniond &orientation = so3_controller_.getComputedOrientation();

    quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command); //! @note memory leak?
    so3_command->header.stamp = ros::Time::now();
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = orientation.x();
    so3_command->orientation.y = orientation.y();
    so3_command->orientation.z = orientation.z();
    so3_command->orientation.w = orientation.w();
    so3_command->kR[0] = 1.5;
    so3_command->kR[1] = 1.5;
    so3_command->kR[2] = 1.0;
    so3_command->kOm[0] = 0.13;
    so3_command->kOm[1] = 0.13;
    so3_command->kOm[2] = 0.1;
    so3_command->aux.current_yaw = cur_yaw_;
    so3_command->aux.enable_motors = true;
    so3_command_pub_.publish(so3_command);

    double thrust = force.norm() / mass_;
    Eigen::Matrix3d Cbn;    
    get_dcm_from_q(Cbn, orientation);
    Eigen::Vector3d att_acc = Eigen::Vector3d(0, 0, thrust);
    att_acc = Cbn * att_acc;
    att_acc(2) -= ONE_G;
    // std::cout<<"att_acc"<<att_acc.transpose()<<std::endl;
    return att_acc;
}

void get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw, Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des)
{
    Eigen::Vector3d force_ = mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    force_.noalias() += mass_ * ref_acc;

    // Limit control angle to theta degree
    double theta = M_PI / 6;
    double c = cos(theta);
    Eigen::Vector3d f;
    f.noalias() = force_ - mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
    {
        double nf = f.norm();
        double A = c * c * nf * nf - f(2) * f(2);
        double B = 2 * (c * c - 1) * f(2) * mass_ * ONE_G;
        double C = (c * c - 1) * mass_ * mass_ * ONE_G * ONE_G;
        double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        force_.noalias() = s * f + mass_ * ONE_G * Eigen::Vector3d(0, 0, 1);
    }

    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(ref_yaw), sin(ref_yaw), 0);

    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    quat_des = Eigen::Quaterniond(R);
    force_des = force_;

}

// 世界系的期望加速度：ref_acc（加上g）、期望yaw：ref_yaw
void pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw)
{
    Eigen::Vector3d force;
    Eigen::Quaterniond quat_des;
    get_Q_from_ACC(ref_acc, ref_yaw, quat_des, force);
    quadrotor_msgs::SO3Command::Ptr so3_command(new quadrotor_msgs::SO3Command);
    so3_command->header.stamp = ros::Time::now();
    so3_command->force.x = force(0);
    so3_command->force.y = force(1);
    so3_command->force.z = force(2);
    so3_command->orientation.x = quat_des.x();
    so3_command->orientation.y = quat_des.y();
    so3_command->orientation.z = quat_des.z();
    so3_command->orientation.w = quat_des.w();
    so3_command->kR[0] = 1.5;
    so3_command->kR[1] = 1.5;
    so3_command->kR[2] = 1.0;
    so3_command->kOm[0] = 0.13;
    so3_command->kOm[1] = 0.13;
    so3_command->kOm[2] = 0.1;
    so3_command->aux.current_yaw = cur_yaw;
    so3_command->aux.enable_motors = true;
    so3_command_pub_.publish(so3_command);
}

void network_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr &cmd)
{
    Eigen::Vector3d des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    double des_yaw = cmd->yaw;
    if(position_cmd_init_ && use_disturbance_observer_){
        disturbance_observer.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
        std::cout<<"dis_acc: "<<dis_acc_.transpose()<<std::endl;
    }
    Eigen::Vector3d att_acc = des_acc - dis_acc_;
    // std::cout<<"acc: "<<des_acc.transpose()<<"   yaw:"<<des_yaw<<std::endl;
    last_des_acc_ = att_acc;
    pub_SO3_command(att_acc, des_yaw, cur_yaw_);
    if (record_log_)
        recordLog(cur_vel_, cur_acc_, des_acc, dis_acc_, cur_yaw_, des_yaw);
    position_cmd_init_ = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    cur_yaw_ = tf::getYaw(odom->pose.pose.orientation);
    cur_vel_ = Eigen::Vector3d(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    if(!position_cmd_init_){
        Eigen::Vector3d position = Eigen::Vector3d(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
        so3_controller_.setPosition(position);
        so3_controller_.setVelocity(cur_vel_);
        Eigen::Vector3d des_pos = Eigen::Vector3d(init_x_, init_y_, init_z_);
        Eigen::Vector3d des_vel = Eigen::Vector3d(0, 0, 0);
        Eigen::Vector3d des_acc = Eigen::Vector3d(0, 0, 0);
        double des_yaw = 0.0;
        double des_yaw_dot = 0.0;
        Eigen::Vector3d att_acc = publishHoverSO3Command(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot);

        if(takeoff_cmd_init_ && use_disturbance_observer_){
            disturbance_observer.HGDO_ext_force_ob(last_des_acc_, cur_vel_, dis_acc_);
            std::cout<<"dis_acc: "<<dis_acc_.transpose()<<std::endl;
        }
        last_des_acc_ = att_acc;
        if (record_log_)
            recordLog(cur_vel_, cur_acc_, des_acc, dis_acc_, cur_yaw_, des_yaw);
        takeoff_cmd_init_ = true;
    }
}

void imu_callback(const sensor_msgs::Imu &imu)
{
    Eigen::Vector3d acc(imu.linear_acceleration.x,
                        imu.linear_acceleration.y,
                        imu.linear_acceleration.z);
    cur_acc_ = acc;
    so3_controller_.setAcc(acc);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "network_controller_node");
    ros::NodeHandle nh("~");

    // 创建订阅者和发布者
    so3_controller_.setMass(mass_);
    so3_command_pub_ = nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
    position_cmd_sub_ = nh.subscribe("position_cmd", 1, network_cmd_callback, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe("odom", 1, odom_callback, ros::TransportHints().tcpNoDelay());
    imu_sub_ = nh.subscribe("imu", 1, imu_callback, ros::TransportHints().tcpNoDelay());
    nh.param("use_disturbance_observer", use_disturbance_observer_, false);
    nh.param("init_state_x", init_x_, 0.0);
    nh.param("init_state_y", init_y_, 0.0);
    nh.param("init_state_z", init_z_, 2.0);
    nh.param("record_log", record_log_, false);
    nh.param("logger_file_name", logger_file_name, std::string("/home/lu/"));
    if (record_log_)
        initLogRecorder();
    // 进入循环
    ros::spin();

    return 0;
}