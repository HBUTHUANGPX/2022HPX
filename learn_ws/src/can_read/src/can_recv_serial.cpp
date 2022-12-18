
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
    // 初始化PID
    pid_type_def motor_speed_pid[4];
    pid_type_def chassis_angle_pid;
    pid_type_def chassis_speed_pid;
    // 初始化底盘参数
    float rx = 0.2042;
    float ry = 0.1838;
    float R = 0.2747;
    float sum_r = 0.388;
    float sum = 0.0f;
    // 初始化电机
    can_read::Can new_recv_1, new_recv_2, new_recv_3, new_recv_4;
    uint16_t out[4];
    float w[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float v[3] = {0.0f, 0.0f, 0.0f};
    float normal_matric[4][3] = {{-1, -1, -sum_r},
                                 {1, -1, -sum_r},
                                 {1, 1, -sum_r},
                                 {-1, 1, -sum_r}};
        // float normal_matric[4][3] = {{1, 1, sum_r},
        //                          {1, -1, -sum_r},
        //                          {1, 1, -sum_r},
        //                          {-1, 1, -sum_r}};
    int flag[5];
    int16_t velotic[4];
    // 初始化电电机参数刷新函数
    void can_fresh();
    void normal();
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
    // 初始化PID参数
    const static float motor_speed_pid1[3] = {M3505_v_P, M3505_v_I, M3505_v_D};
    const static float chassis_angle_pid1[3] = {M3505_angle_P, M3505_angle_I, M3505_angle_D};
    const static float chassis_speed_pid1[3] = {M3505_speed_P, M3505_speed_I, M3505_speed_D};
    for (int i = 0; i < 4; i++)
    {
        PID_init(&motor_speed_pid[i], PID_POSITION, motor_speed_pid1, M3508_out_max, M3508_iout_max);
    }
    PID_init(&chassis_angle_pid, PID_POSITION, chassis_angle_pid1, M3508_angle_out_max, M3508_angle_iout_max); // ���̸�����̨pid
    PID_init(&chassis_speed_pid, PID_POSITION, chassis_speed_pid1, M3508_speed_out_max, M3508_speed_iout_max);
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
void can_recv_serial::can_fresh()
{
    // ROS_INFO_STREAM("fresh ok 0");
    while (ros::ok())
    {
        serial_read(Can_ser, can_serial_read_data);
        if (can_serial_read_data.length() == 16)
        {
            velotic[can_serial_read_data[3]-1] = ((uint8_t)can_serial_read_data[9] << 8) | (uint8_t)can_serial_read_data[10];
            // ROS_INFO_STREAM("fresh ok 1");
            // new_recv.header.frame_id =  to_string(can_serial_read_data[3]);
            // new_recv.header.stamp = ros::Time::now();
            // new_recv.encoder = ((uint8_t)can_serial_read_data[7] << 8) | (uint8_t)can_serial_read_data[8];
            // new_recv.velotic = ((uint8_t)can_serial_read_data[9] << 8) | (uint8_t)can_serial_read_data[10];
            // switch (can_serial_read_data[3])
            // {
            // case 1:
            //     // new_recv_1.header.frame_id = "motor_1";
            //     // new_recv_1.header.stamp = ros::Time::now();
            //     // new_recv_1.encoder = ((uint8_t)can_serial_read_data[7] << 8) | (uint8_t)can_serial_read_data[8];
            //     velotic[0] = ((uint8_t)can_serial_read_data[9] << 8) | (uint8_t)can_serial_read_data[10];
            //     // pub_moto1.publish(new_recv_1);
            //     break;
            // case 2:
            //     // new_recv_2.header.frame_id = "motor_2";
            //     // new_recv_2.header.stamp = ros::Time::now();
            //     // new_recv_2.encoder = ((uint8_t)can_serial_read_data[7] << 8) | (uint8_t)can_serial_read_data[8];
            //     velotic[1] = ((uint8_t)can_serial_read_data[9] << 8) | (uint8_t)can_serial_read_data[10];
            //     // pub_moto2.publish(new_recv_2);
            //     break;
            // case 3:
            //     // new_recv_3.header.frame_id = "motor_3";
            //     // new_recv_3.header.stamp = ros::Time::now();
            //     // new_recv_3.encoder = ((uint8_t)can_serial_read_data[7] << 8) | (uint8_t)can_serial_read_data[8];
            //     velotic[2] = ((uint8_t)can_serial_read_data[9] << 8) | (uint8_t)can_serial_read_data[10];
            //     // pub_moto3.publish(new_recv_3);
            //     break;
            // case 4:
            //     // new_recv_4.header.frame_id = "motor_4";
            //     // new_recv_4.header.stamp = ros::Time::now();
            //     // new_recv_4.encoder = ((uint8_t)can_serial_read_data[7] << 8) | (uint8_t)can_serial_read_data[8];
            //     velotic[3] = ((uint8_t)can_serial_read_data[9] << 8) | (uint8_t)can_serial_read_data[10];
            //     // pub_moto4.publish(new_recv_4);
            //     break;
            // }
            // // Can_ser.write(can_serial_write_data, 30);
            // for (int k = 0; k < 30; k++)
            // {
            //     printf("0x/%02X ", can_serial_write_data[k]);
            // }
            // cout << endl;
            // ROS_INFO_STREAM("fresh ok 2");
        }
        callback_cmd_vel_param();
    }
}
void can_recv_serial::callback_cmd_vel_param()
{
    Can_recv.getParam("vx", v[0]);
    Can_recv.getParam("vy", v[1]);
    Can_recv.getParam("wz", v[2]);
    normal();
    for (int q = 0; q < 1; q++)
    {   

        cout<<abs(w[q]-velotic[q] / 4)<<endl;
        if (abs(w[q]-velotic[q] / 4) < 0.001)
        {
            out[q] = 0;
        }
        else
        {
            out[q] = (int)PID_calc(&motor_speed_pid[q], velotic[q] / 4, w[q]);
            
        }
        cout << q << " out: " << out[q] << " velotic: " << velotic[q] / 4 << " w: " << w[q];
        printf("  0x/%02x 0x/%02x\n", out[q] >> 8, ((out[q] >> 8) << 8) ^ out[q]);
        can_serial_write_data[21 + q * 2] = (out[q] >> 8);
        can_serial_write_data[22 + q * 2] = ((out[q] >> 8) << 8) ^ out[q];
    }
    Can_ser.write(can_serial_write_data, 30);
    for (int k = 21; k < 29; k++)
    {
        printf("0x/%02X_", can_serial_write_data[k]);
    }
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 0;
    // ROS_ERROR_STREAM("callback ok!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
}
void can_recv_serial::normal()
{
    for (int i = 0; i < 4; i++)
    {
        sum = 0.0f;
        for (int j = 0; j < 3; j++)
        {
            sum += normal_matric[i][j] * v[j];
        }
        w[i] = sum * 19.2 * 60 * 0.4767;
    }
    cout << endl;
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
    my_can.can_fresh();
    ros::spin();
    return 0;
}
