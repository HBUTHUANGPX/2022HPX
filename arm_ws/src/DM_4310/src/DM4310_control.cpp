
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <serial/serial.h>
#include "moveit_msgs/RobotTrajectory.h"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <thread>

using namespace std;
#define DM4310_ID1 0x01
#define DM4310_ID2 0x02
#define DM4310_ID3 0x03
#define DM4310_ID4 0x04

int serial_read(serial::Serial &Can_ser, std::string &result)
{
    result = Can_ser.read(Can_ser.available());
    return 0;
}
void print_binary(unsigned int number)
{
    if (number >> 1)
    {
        print_binary(number >> 1);
    }
    putc((number & 1) ? '1' : '0', stdout);
}
class DM4310
{
private:
    /* data */
public:
    DM4310();
    ~DM4310();
    // 初始MIT控制参数
    float target_pos;                                        // 数值精度是千分之一   16位 正负数
    float target_vel;                                        // 数值精度是百分之一   12位 正负数
    float target_tor;                                        // 数值精度是百分之一   12位 正负数
    float Pos_PID_P;                                         // 数值精度是百分之一   12位 正数
    float VEL_PID_D;                                         // 数值精度是百分之一   12位 正数
    uint16_t pos_send, vel_send, kp_send, kd_send, tor_send; // 传入can的数据
    float now_pos;
    uint8_t can_MIT_control_data[8];
    uint8_t motor_ID; // 电机的ID

    const float Max_Pos = 25.0f;
    const float Min_Pos = -25.0f;
    const float Max_Vel = 30.0f;
    const float Min_Vel = -30.0f;
    const float Max_pKp = 500.0f;
    const float Min_pKp = 0.0f;
    const float Max_vKd = 5.0f;
    const float Min_vKd = 0.0f;
    const float Max_Tor = 10.0f;
    const float Min_Tor = -10.0f;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct sockaddr_can recv_addr, send_addr;
    struct can_frame recv_frame, send_frame;
    struct ifreq ifr;
    socklen_t len;
    int ret;
    int nbytes = 0;
    uint8_t ID;
    uint8_t DM4310_motor_ACTIVE_can_data[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFC};
    uint8_t DM4310_motor_DISABL_can_data[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFD};

    int float_to_uint(float x, float x_max, float x_min, int bits);
    float uint_to_float(int x_int, float x_max, float x_min, int bits);
    void ctrl_motor_data_fresh(float target_pos, float target_vel, float target_tor, float Pos_PID_P, float VEL_PID_D);
    void recv_motor_data_fresh();
    void NX_can_send();
    void active();
    void disabl();
    void zero();
    void control_twice();
    void init_ID(uint8_t id);
};

DM4310::DM4310()
{
    recv_addr.can_family = AF_CAN;
    recv_addr.can_ifindex = 0; // 关键点, 接口索引为0 ！！！
    bind(s, (struct sockaddr *)&recv_addr, sizeof(recv_addr));

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    send_addr.can_family = AF_CAN;
    send_addr.can_ifindex = ifr.ifr_ifindex;
    send_frame.can_dlc = 8;
}
DM4310::~DM4310()
{
}
void DM4310::control_twice()
{

}
/**
 * @brief          uint转float
 * @param[in]      x_int: 控制参数
 * @param[in]      x_max: 数据最大值
 * @param[in]      x_min: 数据最小值
 * @param[in]      bits:  数据字节大小
 * @retval         经过比例缩放的数据
 */
float DM4310::uint_to_float(int x_int, float x_max, float x_min, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief          float转uint
 * @param[in]      x:     控制参数
 * @param[in]      x_max: 数据最大值
 * @param[in]      x_min: 数据最小值
 * @param[in]      bits:  数据字节大小
 * @retval         经过比例缩放的数据
 */
int DM4310::float_to_uint(float x, float x_max, float x_min, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    // cout<<x<<"  "<<x_min<<"  "<<x_max<<endl;
    // cout<<(1<<bits)<<" "<<(float)(1<<bits)-1<<" "<<x-offset<<" "<<(x-offset)/span<<" "<<(x - offset) * ((float)((1 << bits) - 1)) / span<<endl;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
 * @brief          更新电机的CAN发送数据，并发送
 * @param[in]      target_pos: 目标角度
 * @param[in]      target_vel: 目标角速度
 * @param[in]      Pos_PID_P:  位置PID参数P
 * @param[in]      VEL_PID_D:  速度PID参数D
 * @param[in]      target_tor: 目标扭矩
 * @retval         NONE
 */
void DM4310::ctrl_motor_data_fresh(float target_pos, float target_vel, float Pos_PID_P, float VEL_PID_D, float target_tor)
{
    pos_send = float_to_uint(target_pos, Max_Pos, Min_Pos, 16);
    vel_send = float_to_uint(target_vel, Max_Vel, Min_Vel, 12);
    kp_send = float_to_uint(Pos_PID_P, Max_pKp, Min_pKp, 12);
    kd_send = float_to_uint(VEL_PID_D, Max_vKd, Min_vKd, 12);
    tor_send = float_to_uint(target_tor, Max_Tor, Min_Tor, 12);
    send_frame.data[0] = (pos_send >> 8);
    send_frame.data[1] = pos_send;
    send_frame.data[2] = (vel_send >> 4);
    send_frame.data[3] = ((vel_send & 0xF) << 4) | (kp_send >> 8);
    send_frame.data[4] = kp_send;
    send_frame.data[5] = (kd_send >> 4);
    send_frame.data[6] = ((kd_send & 0xF) << 4) | (tor_send >> 8);
    send_frame.data[7] = tor_send;
    NX_can_send();
}
/**
 * @brief  内敛函数 用来更新当前的状态变量
 */
inline void DM4310::recv_motor_data_fresh()
{
    now_pos = uint_to_float(recv_frame.data[1] << 8 | recv_frame.data[2], Max_Pos, Min_Pos, 16);
    // cout<<now_pos<<endl;
}
/**
 * @brief 内敛函数，发送数据
 */
inline void DM4310::NX_can_send()
{
    sendto(s, &send_frame, sizeof(struct can_frame), 0, (struct sockaddr *)&send_addr, sizeof(send_addr));
}
/**
 * @brief DM4310的使能函数
 */
void DM4310::active()
{
    ros::Rate r(10);
    for (int t = 0; t < 8; t++)
    {
        send_frame.data[t] = DM4310_motor_ACTIVE_can_data[t];
    }
    NX_can_send();
    r.sleep();
}
/**
 * @brief DM4310的电机失能函数
 */
void DM4310::disabl()
{
    for (int t = 0; t < 8; t++)
    {
        send_frame.data[t] = DM4310_motor_DISABL_can_data[t];
    }
    NX_can_send();
}
/**
 * @brief 回零位函数，先以小阻尼，小刚度的状态运行至零位，随后再以大刚度保持
 */
void DM4310::zero()
{
    ros::Rate r(1);
    ctrl_motor_data_fresh(0.0, 0.0, 5.0, 0.05, 0.0);
    ROS_INFO("DM4310 %d ZERO soft ok", ID);
    r.sleep();
    ctrl_motor_data_fresh(0.0, 0.0, 50.0, 0.5, 0.0);
    ROS_INFO("DM4310 %d ZERO hard ok", ID);
}
/**
 * @brief 初始化成员电机id
 * 
 * @param id 
 */
void DM4310::init_ID(uint8_t id)
{
    ID = id;
    send_frame.can_id = ID;
}
/**
 * @brief DM4310的主要控制类
 * 
 */
class DM_recv_control
{
private:
    /* data */
public:
    DM_recv_control();
    ~DM_recv_control();
    // 初始化节点
    ros::NodeHandle Can_recv;
    ros::Subscriber sub;
    // 初始化回调函数
    uint8_t DM4310_motor_ACTIVE_can_data[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFC};
    uint8_t DM4310_motor_DISABL_can_data[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFD};
    // 初始化电电机参数刷新函数
    DM4310 m_1_DM4310_1; // 第一个yaw轴电机
    DM4310 m_4_DM4310_2; // 第二个yaw轴电机
    DM4310 m_5_DM4310_3; // 第三个pitch轴电机
    DM4310 m_6_DM4310_4; // 第三个yaw轴电机
    void callback_moveit_param(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p);
    void registerNodeHandle(ros::NodeHandle &_nh);
    void DM_motor_active();
    void DM_test();
    void DM_fast_zero();
    void NX_can_recv();
    void NX_can_send();
    void control_twice();
    float test_angle = 0.0f;
    float test_v = 0.0f;
    float test_tor = 0.0f;
    int trun_flag = 0;

    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct sockaddr_can recv_addr, send_addr;
    struct can_frame recv_frame, send_frame;
    struct ifreq ifr;
    socklen_t len;
    int ret;
    int nbytes = 0;
};

DM_recv_control::DM_recv_control()
{
    sub = Can_recv.subscribe<moveit_msgs::RobotTrajectory>("chatter", 1, &DM_recv_control::callback_moveit_param, this);
    recv_addr.can_family = AF_CAN;
    recv_addr.can_ifindex = 0; // 关键点, 接口索引为0 ！！！
    bind(s, (struct sockaddr *)&recv_addr, sizeof(recv_addr));

    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    send_addr.can_family = AF_CAN;
    send_addr.can_ifindex = ifr.ifr_ifindex;
    send_frame.can_dlc = 8;

    m_1_DM4310_1.init_ID(DM4310_ID1);
    m_4_DM4310_2.init_ID(DM4310_ID2);
    m_5_DM4310_3.init_ID(DM4310_ID3);
    m_6_DM4310_4.init_ID(DM4310_ID4);
}
DM_recv_control::~DM_recv_control()
{
}
/**
 * @brief Can 的feedback函数，在main中以子线程的形式独立运行，将接收的数据赋给四个电机对象 
 */
void DM_recv_control::NX_can_recv()
{
    while (ros::ok())
    {
        ret = recvfrom(s, &recv_frame, sizeof(struct can_frame), 0, (struct sockaddr *)&recv_addr, &len);
        if (recv_frame.can_dlc > 0)
        {
            // for (int i = 0; i < recv_frame.can_dlc; ++i)
            // {
            //     printf("%02X ", recv_frame.data[i]);
            // }
            // cout << endl;
            // cout<<m_1_DM4310_1.now_pos<<" "<<m_4_DM4310_2.now_pos<<" "<<m_5_DM4310_3.now_pos<<" "<<m_6_DM4310_4.now_pos<<endl;
            /*
            这里是不能使用for循环降低代码量的，因为数组不能实例化类
            */
            switch (recv_frame.data[0])
            {
            case DM4310_ID1:
                m_1_DM4310_1.recv_frame = recv_frame;
                m_1_DM4310_1.recv_motor_data_fresh();
                Can_recv.setParam("joint_1", m_1_DM4310_1.now_pos);
                break;
            case DM4310_ID2:
                m_4_DM4310_2.recv_frame = recv_frame;
                m_4_DM4310_2.recv_motor_data_fresh();
                Can_recv.setParam("joint_4", m_4_DM4310_2.now_pos);
                break;
            case DM4310_ID3:
                m_5_DM4310_3.recv_frame = recv_frame;
                m_5_DM4310_3.recv_motor_data_fresh();
                Can_recv.setParam("joint_5", m_5_DM4310_3.now_pos);
                break;
            case DM4310_ID4:
                m_6_DM4310_4.recv_frame = recv_frame;
                m_6_DM4310_4.recv_motor_data_fresh();
                Can_recv.setParam("joint_6", m_6_DM4310_4.now_pos);
                break;
            default:
                break;
            }
        }
        
    }
}
/**
 * @brief 获得来自于经过二次计算的moviet控制信息
 */
void DM_recv_control::callback_moveit_param(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p)
{
    // int i = msg_p->joint_trajectory.points.size();
    ros::Rate r(1000);
    // for (int i = 0; i < msg_p->joint_trajectory.points.size(); i++)
    // {
    //     cout << msg_p->joint_trajectory.points[i].positions[5] << endl;
    //     m_6_DM4310_4.ctrl_motor_data_fresh(msg_p->joint_trajectory.points[i].positions[5], 0.0, 20.0, 0.5, 0.0);
        
    //     m_4_DM4310_2.ctrl_motor_data_fresh(msg_p->joint_trajectory.points[i].positions[3], 0.0, 20.0, 0.5, 0.0);
    //     r.sleep();
    // }
    // cout<<msg_p->joint_trajectory<<endl;
    int i =msg_p->joint_trajectory.points.size()*100;

    // float start_1=msg_p->joint_trajectory.points[0].positions[0];
    // float end_1=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[0];

    float start_2=msg_p->joint_trajectory.points[0].positions[3];
    float end_2=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[3];

    float start_3=msg_p->joint_trajectory.points[0].positions[4];
    float end_3=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[4];

    float start_4=msg_p->joint_trajectory.points[0].positions[5];
    float end_4=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[5];

    // float step_1=(end_1-start_1)/i;
    float step_2=(end_2-start_2)/i;
    float step_3=(end_3-start_3)/i;
    float step_4=(end_4-start_4)/i;

    // float cup_1,cuv_1,cukp_1;
    float cup_2,cuv_2,cukp_2;
    float cup_3,cuv_3,cukp_3;
    float cup_4,cuv_4,cukp_4;
    while (i--)
    {
        // cup_1=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[0]-i*step_1;
        // cuv_1=sin(cup_1) * sin(cup_1);
        // cukp_1=20 * (0.8 * sin(cup_1) + 0.2);
        // m_1_DM4310_1.ctrl_motor_data_fresh(cup_1,cuv_1, cukp_1, 0.5, 0.0);

        cup_2=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[3]-i*step_2;
        cuv_2=sin(cup_2) * sin(cup_2);
        cukp_2=50 * (0.3 * sin(cup_2) + 0.7);
        m_4_DM4310_2.ctrl_motor_data_fresh(cup_2,cuv_2, cukp_2, 0.5, 0.0);

        cup_3=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[4]-i*step_3;
        cuv_3=sin(cup_3) * sin(cup_3);
        cukp_3=50 * (0.3 * sin(cup_3) + 0.7);
        m_5_DM4310_3.ctrl_motor_data_fresh(cup_3,cuv_3, cukp_3, 0.5, 0.0);

        cup_4=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[5]-i*step_4;
        cuv_4=sin(cup_4) * sin(cup_4);
        cukp_4=50 * (0.3 * sin(cup_4) + 0.7);
        m_6_DM4310_4.ctrl_motor_data_fresh(cup_4,cuv_4, cukp_4, 0.5, 0.0);

        r.sleep();
    }
    DM_motor_active();
}
void DM_recv_control::control_twice()
{

}
/**
 * @brief 用于激活电机，让电机进入可控制模式
 */
void DM_recv_control::DM_motor_active()
{
    m_1_DM4310_1.active();
    m_4_DM4310_2.active();
    m_5_DM4310_3.active();
    m_6_DM4310_4.active();
    ROS_INFO("DM 4310 init OK");
}
/**
 * @brief 初始化ROS句柄
 * 
 * @param _nh 在main中声明的句柄
 */
void DM_recv_control::registerNodeHandle(ros::NodeHandle &_nh)
{
    Can_recv = _nh;
}
/**
 * @brief 使能电机快速回零位 
 */
void DM_recv_control::DM_fast_zero()
{
    m_1_DM4310_1.zero();
    m_4_DM4310_2.zero();
    m_5_DM4310_3.zero();
    m_6_DM4310_4.zero();
    ROS_INFO("DM 4310 ZERO OK");
}
/**
 * @brief 测试电机，使能电机做纯位置闭环运动，限位-Pi~Pi
 * 
 */
void DM_recv_control::DM_test()
{
    ros::Rate r(1000);
    while (ros::ok()) // 电机逆时针转是正的
    {
        // ROS_INFO("%f__%f__%d", test_angle, test_v/30.0,1);
        // m_4_DM4310_2电机限位-pi~pi
        //
        m_1_DM4310_1.ctrl_motor_data_fresh(0.0, 0.0, 0.0, 0.0, 0.0);
        m_4_DM4310_2.ctrl_motor_data_fresh(0.0, 0.0, 0.0, 0.0, 0.0);
        m_5_DM4310_3.ctrl_motor_data_fresh(0.0, 0.0, 0.0, 0.0, 0.0);
        m_6_DM4310_4.ctrl_motor_data_fresh(0.0, 0.0, 0.0, 0.0, 0.0);
        // if (test_angle >= 3.1415f)
        // {
        //     trun_flag = 1;
        // }
        // if (test_angle <= -3.1415f)
        // {
        //     trun_flag = 0;
        // }
        // if (trun_flag == 1)
        // {
        //     test_angle -= 0.001;
        // }
        // else
        // {
        //     test_angle += 0.001;
        // }
        // cout<<test_angle<<"  "<<trun_flag<<endl;
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DM_4310_control");
    ros::NodeHandle nh;
    DM_recv_control my_can;
    my_can.registerNodeHandle(nh);
    std::thread recv(&DM_recv_control::NX_can_recv, &my_can);
    my_can.DM_motor_active();
    my_can.DM_fast_zero();
    // my_can.DM_test();
    ros::spin();
    return 0;
}
