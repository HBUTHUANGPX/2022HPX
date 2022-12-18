#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
using namespace std;
void doMsg(const geometry_msgs::Twist::ConstPtr &cmd_vel)
{
    ros::param::set("vx", (uint16_t)cmd_vel->linear.x);
    ros::param::set("vy", (uint16_t)cmd_vel->linear.y);
    ros::param::set("wz", (uint16_t)cmd_vel->angular.z);
    cout << "vx: " << cmd_vel->linear.x << " vy: " << cmd_vel->linear.y << " wz " << cmd_vel->angular.z << endl;
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化 ROS 节点:命名(唯一)
    ros::init(argc, argv, "cmd_vel_sub");
    // 3.实例化 ROS 句柄
    ros::NodeHandle nh;
    nh.setParam("vx", 0); // 整型
    nh.setParam("vy", 0); // 浮点型
    nh.setParam("wz", 0); // bool
    // 4.实例化 订阅者 对象
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, doMsg);
    // 5.处理订阅的消息(回调函数)

    //     6.设置循环调用回调函数
    ros::spin(); // 循环读取接收的数据，并调用回调函数处理

    return 0;
}