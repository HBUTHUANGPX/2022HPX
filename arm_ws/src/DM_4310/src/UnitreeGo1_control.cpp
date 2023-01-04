
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <serial/serial.h>

using namespace std;
#define DM4310_ID1 0x01
#define DM4310_ID2 0x02
#define DM4310_ID3 0x03
#define DM4310_ID4 0x04
#define Baudrate_A1 4800000;
#define Baudrate_GM8010 4000000;
#define Baudrate_DM_can 4000000;
#define PI 3.1415926;
int serial_read(serial::Serial &RS485_ser, std::string &result)
{
    result = RS485_ser.read(RS485_ser.available());
    return 0;
}

class Unitree_Go1
{
private:
    /* data */
public:
    Unitree_Go1(/* args */);
    ~Unitree_Go1();
};
Unitree_Go1::Unitree_Go1(/* args */)
{
}
Unitree_Go1::~Unitree_Go1()
{
}
class Unitree_Go1_recv_serial
{
private:
    /* data */
public:
    Unitree_Go1_recv_serial(/* args */);
    ~Unitree_Go1_recv_serial();
    ros::NodeHandle RS485_recv;
    void registerNodeHandle(ros::NodeHandle &_nh);
    void Go1_Serial_Write(int task_ID);
    int float_to_uint(float x, float times);
    void ctrl_motor_data_fresh(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD);
    float reducing_ratio=6.33;
    serial::Serial RS485_ser;
    std::string RS485_serial_read_data;
    uint8_t RS485_serial_write_data[17];
    int pos_send, vel_send, kp_send, kd_send, tor_send;
    uint16_t CRC16;
};
/**
 * @brief          更新将要发送的数据(减速器输入端)
 * @param[in]      target_tor:     期望电机转矩 |T|<= 127.99 N*m
 * @param[in]      target_vel: 期望电机转速 |w|<= 804.0 rad/s
 * @param[in]      target_pos: 期望电机输出位置 |sita|<=411774 rad
 * @param[in]      Pos_PID_kP: 电机刚度系数 0<=kp<=25.599
 * @param[in]      VEL_PID_kD: 电机阻尼系数 0<=kd<=25.599
 * @retval         经过比例缩放的数据
 */
void Unitree_Go1_recv_serial::ctrl_motor_data_fresh(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD)
{
    tor_send = float_to_uint(target_tor,256);             // N*m
    vel_send = float_to_uint(target_vel,128/3.1415926);   // rad/s
    pos_send = float_to_uint(target_pos,16384/3.1415926); // rad
    kp_send = float_to_uint(Pos_PID_kP, 1280);             // 无量纲
    kd_send = float_to_uint(VEL_PID_kD, 1280);              // 无量纲
}


/**
 * @brief          float转uint
 * @param[in]      x:     控制参数
 * @param[in]      times: 数据放大描述倍数,细分程度
 * @retval         经过比例缩放的数据
 */
int Unitree_Go1_recv_serial::float_to_uint(float x, float times)
{
    return int(x * times);
}
Unitree_Go1_recv_serial::Unitree_Go1_recv_serial(/* args */)
{
    // 初始化串口相关设置
    uint8_t data[17] = {
        0xFE,
        0xEE,
    };
    RS485_ser.setPort("/dev/ttyCH9344USB0");                   // 设置打开的串口名称
    RS485_ser.setBaudrate(4000000);                            // 设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 创建timeout
    RS485_ser.setTimeout(to);                                  // 设置串口的timeout
    // 打开串口
    try
    {
        RS485_ser.open(); // 打开串口
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("Unable to open port "); // 打开串口失败，打印信息
    }
    if (RS485_ser.isOpen())
    {
        ROS_DEBUG_STREAM("Unitree Go1 Serial Port initialized."); // 成功打开串口，打印信息
    }
    else
    {
    }
}

Unitree_Go1_recv_serial::~Unitree_Go1_recv_serial()
{
}
void Unitree_Go1_recv_serial::registerNodeHandle(ros::NodeHandle &_nh)
{
    RS485_recv = _nh;
}
void Unitree_Go1_recv_serial::Go1_Serial_Write(int task_ID)
{
    switch (task_ID)
    {
    case 1:
        /* code */
        break;

    default:
        break;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "DM_4310_control");
    ros::NodeHandle nh;
    Unitree_Go1_recv_serial my_can;
    my_can.registerNodeHandle(nh);
    ros::spin();
    return 0;
}
