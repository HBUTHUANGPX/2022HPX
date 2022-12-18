
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <can_read/Can.h>
#include <can_read/chassis_control.h>
#include <can_read/pid.h>
using namespace std;

int serial_read(serial::Serial &Can_ser, std::string &result)
{
    result = Can_ser.read(Can_ser.available());
    return 0;
}

class can_recv_serial
{
private:
    /* data */
public:
    can_recv_serial(/* args */);
    ~can_recv_serial();
    // 初始化节点
    ros::NodeHandle Can_recv;
    // 初始化回调函数
    void callback_cmd_vel_param();
    // 初始化串口
    serial::Serial Can_ser;
    std::string can_serial_read_data;
    uint8_t can_serial_write_data[30];

    // 初始化电机
    int v[3] = {0, 0, 0};

    // 初始化电电机参数刷新函数
    void registerNodeHandle(ros::NodeHandle &_nh);
};

can_recv_serial::can_recv_serial(/* args */)
{
    uint8_t data[30] = {0x55, 0xaa, 0x1e, 0x01, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88};
    for (int l = 0; l < 30; l++)
    {
        can_serial_write_data[l] = data[l];
    }
    for (int k = 0; k < 30; k++)
    {
        printf("0x/%02X_", can_serial_write_data[k]);
    }
    cout << endl;

    // 初始化串口相关设置
    Can_ser.setPort("/dev/ttyACM0");                           // 设置打开的串口名称
    Can_ser.setBaudrate(921600);                               // 设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
    Can_ser.setTimeout(to);                                    // 设置串口的timeout
    // 打开串口
    try
    {
        Can_ser.open(); // 打开串口
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Unable to open port "); // 打开串口失败，打印信息
    }
    if (Can_ser.isOpen())
    {
        ROS_INFO_STREAM("CAN Serial Port initialized."); // 成功打开串口，打印信息
    }
    else
    {
    }
}
can_recv_serial::~can_recv_serial()
{
}
void can_recv_serial::callback_cmd_vel_param()
{
    Can_recv.getParam("vx", v[0]);
    Can_recv.getParam("vy", v[1]);
    Can_recv.getParam("wz", v[2]);
    for (int q = 0; q < 3; q++)
    {
        can_serial_write_data[21 + q * 2] = (((int16_t)v[q]) >> 8);
        can_serial_write_data[22 + q * 2] = ((((int16_t)v[q]) >> 8) << 8) ^ ((int16_t)v[q]);
    }

    Can_ser.write(can_serial_write_data, 30);
    for (int k = 21; k < 29; k++)
    {
        printf("0x/%02X_", can_serial_write_data[k]);
    }
    cout<<endl;
}
void can_recv_serial::registerNodeHandle(ros::NodeHandle &_nh)
{
    Can_recv = _nh;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_can_recv_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(180);
    can_recv_serial my_can;
    my_can.registerNodeHandle(nh);
    while (ros::ok())
    {
        my_can.callback_cmd_vel_param();
    }
    ros::spin();
    return 0;
}
