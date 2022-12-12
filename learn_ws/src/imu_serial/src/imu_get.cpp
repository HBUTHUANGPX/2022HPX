#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Char.h>
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
    ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("Imu_pub", 10);
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
    while (ros::ok())
    {
        serial_read(ser, data);
        if (data[0] == 0x55)
        {
            // cout << "=====" << data.length();
            if (data.length() == 0x42)
            {

                if (data[0x0b] == 0x55 && data[0x0c] == 0x51) // l_acc
                {
                    imu_data.linear_acceleration.x = (float)((data[0x0e] << 8) | data[0x0d]) / 32768.0 * 16.0;
                    if (imu_data.linear_acceleration.x >= 16.0)
                    {
                        imu_data.linear_acceleration.x -= 32.0;
                    }
                    imu_data.linear_acceleration.y = (float)((data[0x10] << 8) | data[0x0f]) / 32768.0 * 16.0;
                    if (imu_data.linear_acceleration.y >= 16.0)
                    {
                        imu_data.linear_acceleration.y -= 32.0;
                    }
                    imu_data.linear_acceleration.z = (float)((data[0x12] << 8) | data[0x11]) / 32768.0 * 16.0;
                    if (imu_data.linear_acceleration.z >= 16.0)
                    {
                        imu_data.linear_acceleration.z -= 32.0;
                    }
                }
                if (data[0x16] == 0x55 && data[0x17] == 0x52) // w_v
                {
                    imu_data.angular_velocity.x = (float)((data[0x19] << 8) | data[0x18]) / 32768.0 * 2000.0;
                    if (imu_data.angular_velocity.x >= 2000.0)
                    {
                        imu_data.angular_velocity.x -= 4000.0;
                    }
                    imu_data.angular_velocity.y = (float)((data[0x1b] << 8) | data[0x1a]) / 32768.0 * 2000.0;
                    if (imu_data.angular_velocity.y >= 2000.0)
                    {
                        imu_data.angular_velocity.y -= 4000.0;
                    }
                    imu_data.angular_velocity.z = (float)((data[0x1d] << 8) | data[0x1c]) / 32768.0 * 2000.0;
                    if (imu_data.angular_velocity.z >= 2000.0)
                    {
                        imu_data.angular_velocity.z -= 4000.0;
                    }
                }
                if (data[0x21] == 0x55 && data[0x22] == 0x53) // w
                {
                }
                if (data[0x2c] == 0x55 && data[0x2d] == 0x54) // c
                {
                }
                if (data[0x37] == 0x55 && data[0x38] == 0x59) // orientation
                {
                    imu_data.orientation.x = (float)((data[0x3a] << 8) | data[0x39]) / 32768.0;
                    if (imu_data.orientation.x >= 1.0)
                    {
                        imu_data.orientation.x -= 2.0;
                    }
                    imu_data.orientation.y = (float)((data[0x3c] << 8) | data[0x3b]) / 32768.0;
                    if (imu_data.orientation.y >= 1.0)
                    {
                        imu_data.orientation.y -= 2.0;
                    }
                    imu_data.orientation.z = (float)((data[0x3e] << 8) | data[0x3d]) / 32768.0;
                    if (imu_data.orientation.z >= 1.0)
                    {
                        imu_data.orientation.z -= 2.0;
                    }
                    imu_data.orientation.w = (float)((data[0x40] << 8) | data[0x3f]) / 32768.0;
                    if (imu_data.orientation.w >= 1.0)
                    {
                        imu_data.orientation.w -= 2.0;
                    }
                }
                if (data[0x00] == 0x55 && data[0x01] == 0x50) // time
                {
                    // imu_data.header.stamp.nsec = ((data[0x09] << 8) | data[0x08]) * 1000;
                }
            }
            // for (int i = 0; i < data.length(); i++)
            // {
            //     if (data[i] == 0x55)
            //     {
            //         cout << endl;
            //     }
            //     cout << hex << (int)data[i] << "(" << i << ")"
            //          << " ";
            // }
            // cout << endl;
            imu_data.header.stamp = ros::Time::now();

            // cout << imu_data.orientation << endl;
            pub_imu.publish(imu_data);
        }
        // cout << " the data read from serial is : " << data.c_str()<<endl;
    }

    ser.close();
    return 0;
}
