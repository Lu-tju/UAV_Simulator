#include "so3_control/NetworkControl.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "network_ctrl_node");
    ros::NodeHandle node("~");
    NetworkControl controller(node);
    // 传递的参数是线程数量。例如，使用4个线程来处理回调。
    ros::MultiThreadedSpinner spinner(4);
    // 启动spinner，它会自动处理所有的回调函数
    spinner.spin();
    return 0;
}