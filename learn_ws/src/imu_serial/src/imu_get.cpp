#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/Imu.h>
#include <imu_serial/my_imu.h>
#include <iostream>
#include <string>
#include <sstream>
#include "nav_msgs/Odometry.h"
#include <tf/tf.h>
using namespace std;

int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read(ser.available());
    return 0;
}
typedef union
{
    unsigned char cdata[8];
    short sdata[4];
} Imu;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port_imu");
    ros::NodeHandle n;
    ros::Publisher pub_imu = n.advertise<imu_serial::my_imu>("Imu_pub", 10);
    serial::Serial ser;

    //初始化串口相关设置
    ser.setPort("/dev/ttyUSB0");                               //设置打开的串口名称
    ser.setBaudrate(115200);                                   //设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); //创建timeout
    ser.setTimeout(to);                                        //设置串口的timeout
    //打开串口
    try
    {
        ser.open(); //打开串口
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Unable to open port "); //打开串口失败，打印信息
        return -1;
    }
    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized. \n"); //成功打开串口，打印信息
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(200);
    std::string data;
    sensor_msgs::Imu imu_data;
    imu_serial::my_imu my_imu_data;
    my_imu_data.Imu.header.frame_id="base_link";
    my_imu_data.Imu.header.seq=0;
    nav_msgs::Odometry position;
    tf::Quaternion RQ2;
    Imu acc, wv, rpy,sy;
    while (ros::ok())
    {
        serial_read(ser, data);
        
        if (data[0] == 0x55)
        {
            if (data[0x0b] == 0x55 && data[0x0c] == 0x51) // l_acc
            {
                for (int i = 0; i < 8; i++)
                {
                    acc.cdata[i] = data[0x0d + i];
                }
                my_imu_data.Imu.linear_acceleration.x = acc.sdata[0] / 32768.0f * 16.0f;
                my_imu_data.Imu.linear_acceleration.y = acc.sdata[1] / 32768.0f * 16.0f;
                my_imu_data.Imu.linear_acceleration.z = acc.sdata[2] / 32768.0f * 16.0f;
            }
            if (data[0x16] == 0x55 && data[0x17] == 0x52) // w_v
            {
                for (int i = 0; i < 8; i++)
                {
                    wv.cdata[i] = data[0x18 + i];
                }
                my_imu_data.Imu.angular_velocity.x = wv.sdata[0] / 32768.0f * 2000.0f;
                my_imu_data.Imu.angular_velocity.y = wv.sdata[1] / 32768.0f * 2000.0f;
                my_imu_data.Imu.angular_velocity.z = wv.sdata[2] / 32768.0f * 2000.0f;
            }
            if (data[0x21] == 0x55 && data[0x22] == 0x53) // w
            {
                for (int i = 0; i < 8; i++)
                {
                    rpy.cdata[i] = data[0x23 + i];
                }
                my_imu_data.wx=rpy.sdata[0]/ 32768.0f * 180.0f;
                my_imu_data.wy=rpy.sdata[1]/ 32768.0f * 180.0f;
                my_imu_data.wz=rpy.sdata[2]/ 32768.0f * 180.0f;
            }
            if (data[0x2C] == 0x55 && data[0x2D] == 0x59) // orientation
            {
                for (int i = 0; i < 8; i++)
                {
                    sy.cdata[i] = data[0x2E + i];
                }
                my_imu_data.Imu.orientation.x = sy.sdata[0] / 32768.0f;
                my_imu_data.Imu.orientation.y = sy.sdata[1] / 32768.0f;
                my_imu_data.Imu.orientation.z = sy.sdata[2] / 32768.0f;
                my_imu_data.Imu.orientation.w = sy.sdata[3] / 32768.0f;
            }
            if (data[0x00] == 0x55 && data[0x01] == 0x50) // time
            {
                // imu_data.header.stamp.nsec = ((data[0x09] << 8) | data[0x08]) * 1000;
            }
            my_imu_data.Imu.header.seq+=1;
            my_imu_data.Imu.header.stamp = ros::Time::now();
            pub_imu.publish(my_imu_data);
        }
    }

    ser.close();
    return 0;
}
