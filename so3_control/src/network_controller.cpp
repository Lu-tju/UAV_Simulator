/*
    神经网络直接预测期望加速度，这个程序把他转成期望姿态+推力，直接发给仿真器，目前看来并没大区别，但可能加上阻力就变不好了
*/
#include <Eigen/Eigen>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include "so3_control/HGDO.h"

#define ONE_G 9.81
// 他这个控制器收的是绝对的力，px4收的是油门百分比，例如只有重力g时就是0.4（假设40%油门悬停）
double mass_ = 0.98;
double cur_yaw_ = 0;
bool position_cmd_init_ = false;
ros::Publisher so3_command_pub_;
ros::Subscriber position_cmd_sub_, odom_sub_;
double init_x_, init_y_, init_z_;

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
    position_cmd_init_ = true;
    Eigen::Vector3d des_acc = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                                              cmd->acceleration.z);
    double des_yaw = cmd->yaw;
    // std::cout<<"acc: "<<des_acc.transpose()<<"   yaw:"<<des_yaw<<std::endl;
    pub_SO3_command(des_acc, des_yaw, cur_yaw_);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    cur_yaw_ = tf::getYaw(odom->pose.pose.orientation);
    if(!position_cmd_init_){
        double acc = 0.5 * (init_z_ - odom->pose.pose.position.z);
        Eigen::Vector3d des_acc = Eigen::Vector3d(0, 0, acc);
        pub_SO3_command(des_acc, cur_yaw_, cur_yaw_);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "network_controller_node");
    ros::NodeHandle nh;

    // 创建订阅者和发布者
    so3_command_pub_ = nh.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);
    position_cmd_sub_ = nh.subscribe("position_cmd", 1, network_cmd_callback, ros::TransportHints().tcpNoDelay());
    odom_sub_ = nh.subscribe("odom", 1, odom_callback, ros::TransportHints().tcpNoDelay());
    nh.param("init_state_x", init_x_, 0.0);
    nh.param("init_state_y", init_y_, 0.0);
    nh.param("init_state_z", init_z_, 2.0);
    // 进入循环
    ros::spin();

    return 0;
}