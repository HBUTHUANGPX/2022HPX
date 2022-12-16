#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <imu_serial/my_imu.h>
#include <iostream>
#include <string>
#include <sstream>
#include <math.h>
#include <nav_msgs/Odometry.h>
using namespace std;

class OdomWorkFlow
{
public:
    ros::NodeHandle n;
    ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("my_Odom", 10);
    serial::Serial ser;
    std::string data;
    float new_pos_x, new_pos_y, new_pos_z,old_pos_x, old_pos_y, old_pos_z, msg_pos_x, msg_pos_y = 0;
    float fdata[6];
    int flag = 0;
    imu_serial::my_imu my_imu;
    nav_msgs::Odometry my_od;
    ros::Subscriber sub_imu = n.subscribe<imu_serial::my_imu>("Imu_pub", 10, &OdomWorkFlow::doMsg, this);
    ros::Time  _now,_old;
    double sample_time,ops_x_wheel_vel,ops_y_wheel_vel;
    int init()
    {
        ros::Rate loop_rate(100);
        // ser.setPort("/dev/USB_ops");                               // 设置打开的串口名称
        // ser.setBaudrate(115200);                                   // 设置串口的波特率
        // serial::Timeout to = serial::Timeout::simpleTimeout(5000); // 创建timeout
        // ser.setTimeout(to);                                        // 设置串口的timeout
        // // 打开串口
        // try
        // {
        //     ser.open(); // 打开串口
        // }
        // catch (const std::exception &e)
        // {
        //     ROS_ERROR_STREAM("Unable to open port "); // 打开串口失败，打印信息
        //     return -1;
        // }
        // if (ser.isOpen())
        // {
        //     ROS_INFO_STREAM("Serial Port initialized. \n"); // 成功打开串口，打印信息
        // }
        // else
        // {
        //     return -1;
        // }
    }
    void doMsg(const imu_serial::my_imu::ConstPtr &msg)
    {
        while (ros::ok())
        {
            // serial_read(ser, data);
            if (flag == 0)
            {
                // ops清零操作
                // flag = 1;
                
                
            }
            if (data.length() == 28 & flag==1)
            {
                if (data[0] == 0xD & data[1] == 0xA)
                {
                    byte_to_float(new_pos_x, new_pos_y, data);
                    
                    my_od.header.frame_id = "my_Odom";
                    my_od.child_frame_id = "basefoot_print";
                    my_od.pose.pose.orientation = msg->Imu.orientation;
                    my_od.twist.twist.angular = msg->Imu.angular_velocity;

                    my_od.header.stamp =_now= ros::Time::now();
                    sample_time=(_now-_old).toSec();
                    ops_x_wheel_vel=(new_pos_x-old_pos_x)/sample_time;
                    ops_y_wheel_vel=(new_pos_y-old_pos_y)/sample_time;
                    old_pos_x=new_pos_x;
                    old_pos_y=new_pos_y;

                    my_od.twist.twist.linear.x = ops_x_wheel_vel;
                    my_od.twist.twist.linear.y = ops_y_wheel_vel;
                    my_od.twist.twist.linear.z = 0;
                    my_od.pose.pose.position.x = new_pos_x;
                    my_od.pose.pose.position.y = new_pos_y;
                    my_od.pose.pose.position.z = 0;
                    _old=_now;
                    

                }
            }
            
            // Sampling_Time = (_Now - _Last_Time).toSec();
            my_od.header.frame_id = "my_Odom";
            my_od.child_frame_id = "basefoot_print";
            my_od.pose.pose.position.x = 2;
            my_od.pose.pose.position.y = 22;
            my_od.pose.pose.position.z = 222;
            my_od.pose.pose.orientation = msg->Imu.orientation;
            my_od.twist.twist.angular = msg->Imu.angular_velocity;
            my_od.twist.twist.linear.x = 1;
            my_od.twist.twist.linear.y = 11;
            my_od.twist.twist.linear.z = 111;
            my_od.header.stamp =_now= ros::Time::now();
            // cout<<my_od<<endl;
        }
        // ser.close();
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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_odom");
    OdomWorkFlow my_odom;
    my_odom.init();
    ros::spin();
    return 0;
}
