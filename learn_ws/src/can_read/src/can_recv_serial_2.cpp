#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include <sstream>

#include <can_read/Can.h>
using namespace std;

int serial_read(serial::Serial &ser, std::string &result)
{
    result = ser.read(ser.available());
    return 0;
}

union can_recv_serial
{
    unsigned char cdata[4];
    short sdata[2];
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port_imu");
    ros::NodeHandle Can_recv;
    // ros::Publisher pub_imu = n.advertise<imu_serial::my_imu>("Imu_pub", 1);
    ros::Publisher pub_moto1 = Can_recv.advertise<can_read::Can>("Can_moto1", 1);
    ros::Publisher pub_moto2 = Can_recv.advertise<can_read::Can>("Can_moto2", 1);
    ros::Publisher pub_moto3 = Can_recv.advertise<can_read::Can>("Can_moto3", 1);
    ros::Publisher pub_moto4 = Can_recv.advertise<can_read::Can>("Can_moto4", 1);
    serial::Serial ser;

    // 初始化串口相关设置
    ser.setPort("/dev/ttyACM0");                               // 设置打开的串口名称
    ser.setBaudrate(921600);                                   // 设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
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
        ROS_INFO_STREAM("IMU Serial Port initialized."); // 成功打开串口，打印信息
    }
    else
    {
        return -1;
    }
    ros::Rate loop_rate(180);
    std::string data;
    can_recv_serial motor_1, motor_2, motor_3, motor_4, motor;
    can_read::Can new_recv_1, new_recv_2, new_recv_3, new_recv_4;
    short encoder,velotic;
    while (ros::ok())
    {
        serial_read(ser, data);
        if (data.length() == 16)
        {
            switch (data[3])
            {
            case 1:
                new_recv_1.header.frame_id = "motor_1";
                new_recv_1.header.stamp = ros::Time::now();
                new_recv_1.encoder = ((uint8_t)data[7] << 8 )| (uint8_t)data[8];
                new_recv_1.velotic = ((uint8_t)data[9] << 8 )| (uint8_t)data[10];
                pub_moto1.publish(new_recv_1);
                break;
            case 2:
                new_recv_2.header.frame_id = "motor_2";
                new_recv_2.header.stamp = ros::Time::now();
                new_recv_2.encoder = ((uint8_t)data[7] << 8 )| (uint8_t)data[8];
                new_recv_2.velotic = ((uint8_t)data[9] << 8 )| (uint8_t)data[10];
                pub_moto2.publish(new_recv_2);
                break;
            case 3:
                new_recv_3.header.frame_id = "motor_3";
                new_recv_3.header.stamp = ros::Time::now();
                new_recv_3.encoder = ((uint8_t)data[7] << 8 )| (uint8_t)data[8];
                new_recv_3.velotic = ((uint8_t)data[9] << 8 )| (uint8_t)data[10];
                pub_moto3.publish(new_recv_3);
                break;
            case 4:
                new_recv_4.header.frame_id = "motor_4";
                new_recv_4.header.stamp = ros::Time::now();
                new_recv_4.encoder = ((uint8_t)data[7] << 8 )| (uint8_t)data[8];
                new_recv_4.velotic = ((uint8_t)data[9] << 8 )| (uint8_t)data[10];
                pub_moto4.publish(new_recv_4);
                break;
            }
        }
    }

    ser.close();
    return 0;
}
