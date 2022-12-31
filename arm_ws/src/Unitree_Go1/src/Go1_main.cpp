#include "Go1/unitreeMotor/include/motor_msg.h" // 电机通信协议
#include <unistd.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include "stdio.h"
#include "Go1/crc/crc_ccitt.h"
#include <thread>
#include <fstream>
#include <iostream>
#include <sstream>
#include "moveit_msgs/RobotTrajectory.h"
using namespace std;
template <class Type>
Type stringToNum(const string &str)
{
  istringstream iss(str);
  Type num;
  iss >> num;
  return num;
}

void print_binary(unsigned int number)
{
  if (number >> 1)
  {
    print_binary(number >> 1);
  }
  putc((number & 1) ? '1' : '0', stdout);
}
void txt_write(float zero)
{
  ofstream out_txt_file;
  out_txt_file.open("/home/hpx/2022HPX/src/arm_ws/src/Unitree_Go1/param/zero.txt");
  out_txt_file << fixed;
  out_txt_file << setprecision(5) << zero << endl;
  out_txt_file.close();
}
float txt_read()
{
  ifstream infile;
  infile.open("/home/hpx/2022HPX/src/arm_ws/src/Unitree_Go1/param/zero.txt");
  string s;
  float zero;
  while (getline(infile, s))
  {
    zero = stringToNum<float>(s);
    cout << zero << endl;
  }
  infile.close();
  return zero;
}
union Go1_send_data
{
  ControlData_t motor_send_data;
  uint8_t c[17];
};
union Go1_recv_data
{
  MotorData_t motor_recv_data;
  uint8_t c[16];
};
class Unitree_Go1_recv_serial
{
private:
  /* data */
public:
  Unitree_Go1_recv_serial(/* args */);
  ~Unitree_Go1_recv_serial();
  ros::NodeHandle RS485_recv;
  ros::Subscriber sub;
  void registerNodeHandle(ros::NodeHandle &_nh);
  void Go1_Serial_Write(int task_ID);
  void Go1_Serial_Read(int task_ID);
  int float_to_uint(float x, float times);
  float uint_to_float(int x, float times);
  void motor_data_fresh_send(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD);
  void motor_data_fresh_recv();
  void test();
  void zero();
  void set_manual_zero_point();
  void zero_tor();
  void callback_moveit_param(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p);
  float manual_zero;
  float reducing_ratio = 6.33;
  serial::Serial RS485_ser;
  std::string RS485_serial_read_data;
  uint8_t RS485_serial_write_data[17];
  int pos_send, vel_send, kp_send, kd_send, tor_send;
  float now_pos;
  uint16_t CRC16;
  Go1_send_data send_data;
  Go1_recv_data recv_data;
  // Go1_send_data send_data;
  float test_angle = 0.0f;
  float test_v = 0.0f;
  float test_tor = 0.0f;
  int trun_flag = 0;
};
Unitree_Go1_recv_serial::Unitree_Go1_recv_serial(/* args */)
{
  sub = RS485_recv.subscribe<moveit_msgs::RobotTrajectory>("chatter", 1, &Unitree_Go1_recv_serial::callback_moveit_param, this);
  // 初始化串口相关设置
  send_data.motor_send_data.head[0] = 0xFE;
  send_data.motor_send_data.head[1] = 0xEE;
  send_data.motor_send_data.mode.id = 0;
  send_data.motor_send_data.mode.status = 1;
  send_data.motor_send_data.mode.none = 0;
  send_data.motor_send_data.comd.tor_des = 0x00;
  send_data.motor_send_data.comd.spd_des = 0x00;
  send_data.motor_send_data.comd.pos_des = 0x00;
  send_data.motor_send_data.comd.k_pos = 0x00;
  send_data.motor_send_data.comd.k_spd = 0x00;
  send_data.motor_send_data.CRC16 = 0x00;
  RS485_ser.setPort("/dev/ttyCH9344USB3");                   // 设置打开的串口名称
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
/**
 * @brief          更新将要发送的数据(减速器输入端)
 * @param[in]      target_tor:     期望电机转矩 |T|<= 127.99 N*m
 * @param[in]      target_vel: 期望电机转速 |w|<= 804.0 rad/s
 * @param[in]      target_pos: 期望电机输出位置 |sita|<=411774 rad
 * @param[in]      Pos_PID_kP: 电机刚度系数 0<=kp<=25.599
 * @param[in]      VEL_PID_kD: 电机阻尼系数 0<=kd<=25.599
 * @retval         经过比例缩放的数据
 */
void Unitree_Go1_recv_serial::motor_data_fresh_send(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD)
{
  send_data.motor_send_data.comd.tor_des = (int16_t)float_to_uint(target_tor, 256);               // N*m
  send_data.motor_send_data.comd.spd_des = (int16_t)float_to_uint(target_vel, 128 / 3.1415926);   // rad/s
  send_data.motor_send_data.comd.pos_des = (int32_t)float_to_uint(target_pos, 16384 / 3.1415926); // rad
  send_data.motor_send_data.comd.k_pos = (uint16_t)float_to_uint(Pos_PID_kP, 1280);               // 无量纲
  send_data.motor_send_data.comd.k_spd = (uint16_t)float_to_uint(VEL_PID_kD, 1280);
  send_data.motor_send_data.CRC16 = 0x00; // 无量纲
  send_data.motor_send_data.CRC16 = crc_ccitt(send_data.motor_send_data.CRC16, send_data.c, 15);
  RS485_ser.write(send_data.c, 17);
  // for (int k = 0; k < 17; k++)
  // {
  //   printf("0x/%02X_", send_data.c[k]);
  // }
  // cout << endl;
}
/**
 * @brief 获得来自于经过二次计算的moviet控制信息
 */
void Unitree_Go1_recv_serial::callback_moveit_param(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p)
{
  // int i = msg_p->joint_trajectory.points.size();
  ros::Rate r(10);
  for (int i = 0; i < msg_p->joint_trajectory.points.size(); i++)
  {
    cout << msg_p->joint_trajectory.points[i].positions[2] << endl;
    motor_data_fresh_send(0.0, 0.0, -msg_p->joint_trajectory.points[i].positions[2] * 6.33f + manual_zero * 6.33f, 1.0, 0.01);
    r.sleep();
  }
  // cout<<msg_p->joint_trajectory<<endl;
}
/**
 * @brief 串口 的feedback函数，在main中以子线程的形式独立运行，更新数据 
 */
void Unitree_Go1_recv_serial::motor_data_fresh_recv()
{
  while (ros::ok())
  {
    RS485_ser.read(recv_data.c, 16);
    now_pos = uint_to_float(recv_data.motor_recv_data.fbk.pos, 16384 / 3.1415926) / 6.33f;
    cout << "now pos: " << now_pos << endl;
    RS485_recv.setParam("joint_3", -now_pos+manual_zero); //浮点型
  }
}
/**
 * @brief int转float
 * 
 * @param          x:     控制参数
 * @param          times: 数据放大描述倍数,细分程度
 * @retval         经过比例缩放的数据
 */
float Unitree_Go1_recv_serial::uint_to_float(int x, float times)
{
  return x / times;
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
/**
 * @brief 初始化ROS句柄
 * 
 * @param _nh 
 */
void Unitree_Go1_recv_serial::registerNodeHandle(ros::NodeHandle &_nh)
{
  RS485_recv = _nh;
}
/**
 * @brief 使能电机快速回零位
 */
void Unitree_Go1_recv_serial::zero()
{
  // ros::Rate r(10);
  // while (ros::ok())
  // {
  motor_data_fresh_send(0.0, 0.0, 0.0 + manual_zero * 6.33f, 1.0, 0.01);
  // r.sleep();
  // }
}
/**
 * @brief 因为电机的特殊性，人为设置零点
 */
void Unitree_Go1_recv_serial::set_manual_zero_point()
{
  zero_tor();
  manual_zero = now_pos;
  cout << "manual zero: " << manual_zero << endl;
  txt_write(manual_zero);
  zero();
}
/**
 * @brief 使能电机进入零力矩模式
 */
void Unitree_Go1_recv_serial::zero_tor()
{
  ros::Rate r(10);
  int t = 10;
  while (t--)
  {
    motor_data_fresh_send(0.0, 0.0, 0.0, 0.0, 0.0);
    r.sleep();
  }
}
/**
 * @brief 测试 
 */
void Unitree_Go1_recv_serial::test()
{
  ros::Rate r(1000);
  while (ros::ok())
  {
    if (test_angle >= 3.1415f * 6.33f)
    {
      trun_flag = 1;
    }
    if (test_angle <= -3.1415f * 6.33f)
    {
      trun_flag = 0;
    }
    if (trun_flag == 1)
    {
      test_angle -= 0.001f * 6.33f;
    }
    else
    {
      test_angle += 0.001f * 6.33f;
    }
    motor_data_fresh_send(0.0, 0.0, 0.0, 0.0, 0.04);
    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Go1_control");
  ros::NodeHandle nh;
  Unitree_Go1_recv_serial my_485;
  my_485.registerNodeHandle(nh);
  std::thread recv(&Unitree_Go1_recv_serial::motor_data_fresh_recv, &my_485);
  // my_485.test();
  txt_read();
  my_485.set_manual_zero_point();
  ros::spin();
  return 0;
}