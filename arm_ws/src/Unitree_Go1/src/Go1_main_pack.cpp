#include "Go1/unitreeMotor/include/motor_msg.h" // 电机通信协议
#include <unistd.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include "stdio.h"
#include "crc/crc_ccitt.h"
using namespace std;
typedef union Go1_send_data
{
  ControlData_t motor_send_data;
  uint8_t c[17];
};
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
  void ctrl_motor_data_fresh_send(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD);
  void test();
  float reducing_ratio = 6.33;
  serial::Serial RS485_ser;
  std::string RS485_serial_read_data;
  uint8_t RS485_serial_write_data[17];
  int pos_send, vel_send, kp_send, kd_send, tor_send;
  uint16_t CRC16;
  Go1_send_data send_data;
  float test_angle = 0.0f;
  float test_v = 0.0f;
  float test_tor = 0.0f;
  int trun_flag = 0;
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
void Unitree_Go1_recv_serial::ctrl_motor_data_fresh_send(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD)
{
  send_data.motor_send_data.comd.tor_des = (int16_t)float_to_uint(target_tor, 256);               // N*m
  send_data.motor_send_data.comd.spd_des = (int16_t)float_to_uint(target_vel, 128 / 3.1415926);   // rad/s
  send_data.motor_send_data.comd.pos_des = (int32_t)float_to_uint(target_pos, 16384 / 3.1415926); // rad
  send_data.motor_send_data.comd.k_pos = (uint16_t)float_to_uint(Pos_PID_kP, 1280);               // 无量纲
  send_data.motor_send_data.comd.k_spd = (uint16_t)float_to_uint(VEL_PID_kD, 1280);
  send_data.motor_send_data.CRC16 = 0x00; // 无量纲
  send_data.motor_send_data.CRC16 = crc_ccitt(send_data.motor_send_data.CRC16, send_data.c, 15);
  RS485_ser.write(send_data.c, 17);
  for (int k = 0; k < 17; k++)
  {
    printf("0x/%02X_", send_data.c[k]);
  }
  cout << endl;
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
void Unitree_Go1_recv_serial::test()
{
  while (ros::ok())
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
      ctrl_motor_data_fresh_send(0.0, 0.0, 0.0, 0.8, 0.0);
      r.sleep();
    }
  }
}
void print_binary(unsigned int number)
{
  if (number >> 1)
  {
    print_binary(number >> 1);
  }
  putc((number & 1) ? '1' : '0', stdout);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Go1_control");
  ros::NodeHandle nh;
  Unitree_Go1_recv_serial my_485;
  my_485.registerNodeHandle(nh);
  my_485.test();
}