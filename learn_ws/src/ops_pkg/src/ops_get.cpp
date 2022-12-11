#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read(ser.available());
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port");
    ros::NodeHandle n;
    ros::Publisher pub_pos_x = n.advertise<std_msgs::Float32>("pos_x", 10);
    ros::Publisher pub_pos_y = n.advertise<std_msgs::Float32>("pos_y", 10);
    ros::Publisher pub_w_z = n.advertise<std_msgs::Float32>("w_z", 10);
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
    ros::Rate loop_rate(100);
    std::string data;
    sensor_msgs::Imu imu_data;
    float pos_x = 0;
    float pos_y = 0;
    float w_z = 0;
    float zangle = 0;
    float xangle = 0;
    float yangle = 0;
    int flag=0;
    std_msgs::Float32 msg_pos_x, msg_pos_y, msg_w_z;
    while (ros::ok())
    {
        serial_read(ser, data);
        if (flag == 0)
        {
            pos_x = (float)((data[17] << 24) | (data[16] << 16) | (data[15] << 8) | (data[14]));
            pos_y = (float)((data[17 + 4] << 24) | (data[16 + 4] << 16) | (data[15 + 4] << 8) | (data[14 + 4]));
            w_z = (float)((data[17 + 8] << 24) | (data[16 + 8] << 16) | (data[15 + 8] << 8) | (data[14 + 8]));
            flag=1;
        }
        if (data.length() > 0)
        {
            for (int i = 0; i < data.length(); i++)
            {
                cout << hex << "/0x" << (int)data[i];
            }
            cout << endl;
            // if (data.length()==28)
            // {
            // if (data[0] == 0xd && data[1] == 0xa && data[0x26] == 0xa && data[27] == 0xd)
            // {
            msg_pos_x.data = (float)((data[17] << 24) | (data[16] << 16) | (data[15] << 8) | (data[14]))-pos_x;
            msg_pos_y.data = (float)((data[17 + 4] << 24) | (data[16 + 4] << 16) | (data[15 + 4] << 8) | (data[14 + 4]))-pos_y;
            msg_w_z.data = (float)((data[17 + 8] << 24) | (data[16 + 8] << 16) | (data[15 + 8] << 8) | (data[14 + 8]))-w_z;
            // msg_pos_x.data=(float)((data[14]<<24)|(data[15]<<16)|(data[16]<<8)|(data[17]));
            // msg_pos_y.data=(float)((data[14+4]<<24)|(data[15+4]<<16)|(data[16+4]<<8)|(data[17+4]));
            // msg_w_z.data=(float)((data[14+8]<<24)|(data[15+8]<<16)|(data[16+8]<<8)|(data[17+8]));
            // cout<<a<<endl;
            // cout<<b<<endl;
            // cout<<c<<endl;
            pub_pos_x.publish(msg_pos_x);
            pub_pos_y.publish(msg_pos_y);
            pub_w_z.publish(msg_w_z);
            // }
            // }
        }
    }

    ser.close();
    return 0;
}
