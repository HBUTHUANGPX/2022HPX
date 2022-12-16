#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
using namespace std;
typedef union
{
    float fdata[6];
    uint8_t ldata[24];
} Posture;
int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read(ser.available());
    return 0;
}
void byte_to_float(float px, float py, std::string byte)
{
    Posture pos;
    for (int i = 0; i < 24; i++)
    {
        pos.ldata[i] = byte[i + 2];
    }
    px = pos.fdata[3];
    py = pos.fdata[4];
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port_ops");
    ros::NodeHandle n;
    ros::Publisher pub_pos_x = n.advertise<std_msgs::Float32>("pos_x", 10);
    ros::Publisher pub_pos_y = n.advertise<std_msgs::Float32>("pos_y", 10);
    serial::Serial ser;
    ser.setPort("/dev/USB_ops");                               // 设置打开的串口名称
    ser.setBaudrate(115200);                                   // 设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(5000); // 创建timeout
    ser.setTimeout(to);                                        // 设置串口的timeout
    // 打开串口
    try
    {
        ser.open(); // 打开串口
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Unable to open port "); // 打开串口失败，打印信息
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized. \n"); // 成功打开串口，打印信息
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(100);
    std::string data;
    float pos_x = 0;
    float pos_y = 0;
    float fdata[6];
    int flag = 0;
    std_msgs::Float32 msg_pos_x, msg_pos_y;
    while (ros::ok())
    {
        serial_read(ser, data);
        if (flag == 0)
        {
            if (data[0] == 0xD & data[1] == 0xA)
            {
                byte_to_float(pos_x, pos_y, data);
                msg_pos_x.data = pos_x;
                msg_pos_y.data = pos_y;
                flag = 1;
            }
        }
        if (data.length() == 28)
        {
            if (data[0] == 0xD & data[1] == 0xA)
            {
                byte_to_float(pos_x, pos_y, data);
                msg_pos_x.data -= pos_x;
                msg_pos_y.data -= pos_y;
                pub_pos_x.publish(msg_pos_x);
                pub_pos_y.publish(msg_pos_y);
            }
        }
    }
    ser.close();
    return 0;
}
