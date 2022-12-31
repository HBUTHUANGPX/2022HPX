#include <unistd.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include "stdio.h"
#include "Unitree_A1/A1_recv.h"
#include "Unitree_A1/A1_send.h"

#include "Unitree_A1/crc.h"
#include "Unitree_A1/motor_msg.h" //电机通信协议
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
void txt_write(float zero)
{
  ofstream out_txt_file;
  out_txt_file.open("/home/hpx/2022HPX/src/arm_ws/src/Unitree_A1/param/zero.txt");
  out_txt_file << fixed;
  out_txt_file << setprecision(5) << zero << endl;
  out_txt_file.close();
}
float txt_read()
{
  ifstream infile;
  infile.open("/home/hpx/2022HPX/src/arm_ws/src/Unitree_A1/param/zero.txt");
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
union A1_send_data
{
  MasterComdDataV3 motor_send_data;
  // vector<uint8_t> sv[34];
  uint8_t c[34];
  // std::string ss[34];
  uint32_t sdata[7];
};
struct A1_recv_data
{
  ServoComdDataV3 motor_recv_data;
  uint8_t sdata[78];
};
class Unitree_A1_recv_serial
{
private:
  /* data */
public:
  Unitree_A1_recv_serial(/* args */);
  ~Unitree_A1_recv_serial();
  ros::NodeHandle RS485_recv;
  ros::Publisher pub = RS485_recv.advertise<Unitree_A1::A1_recv>("A1_recv_data", 1);
  ros::Subscriber sub = RS485_recv.subscribe<Unitree_A1::A1_send>("A1_send_data", 1, &Unitree_A1_recv_serial::doMsg, this);
  ros::Subscriber sub2 =RS485_recv.subscribe<moveit_msgs::RobotTrajectory>("chatter", 1, &Unitree_A1_recv_serial::callback_moveit_param, this);
  Unitree_A1::A1_recv pub_data;
  Unitree_A1::A1_send sub_data;
  void registerNodeHandle(ros::NodeHandle &_nh);
  void A1_Serial_Write(int task_ID);
  int float_to_uint(float x, float times);
  float uint_to_float(int x, float times);
  void motor_data_fresh_send(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD);
  void test();
  void zero();
  void set_manual_zero_point();
  void zero_tor();
  void doMsg(const Unitree_A1::A1_send::ConstPtr &msg_p);
  void callback_moveit_param(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p);
  float manual_zero;
  float reducing_ratio = 9.1;
  uint8_t RS485_serial_write_data[34];
  int pos_send, vel_send, kp_send, kd_send, tor_send;
  A1_send_data send_data;
  A1_recv_data recv_data;
  float test_angle = 0.0f;
  float test_v = 0.0f;
  float test_tor = 0.0f;
  int trun_flag = 1;
  float now_pos;
};
Unitree_A1_recv_serial::Unitree_A1_recv_serial(/* args */)
{
  // 初始化串口相关设置
  send_data.motor_send_data.head.start[0] = 0xFE;
  send_data.motor_send_data.head.start[1] = 0xEE;
  send_data.motor_send_data.head.motorID = 1;
  send_data.motor_send_data.Mdata.ModifyBit = 0xFF;
  send_data.motor_send_data.Mdata.mode = 10;
  send_data.motor_send_data.Mdata.T = 0x0000;
  send_data.motor_send_data.Mdata.W = 0x0000;
  send_data.motor_send_data.Mdata.Pos = 0x00000000;
  send_data.motor_send_data.Mdata.K_P = 0x0000;
  send_data.motor_send_data.Mdata.K_W = 0x0000;
  send_data.motor_send_data.CRCdata = 0x00000000;
}
Unitree_A1_recv_serial::~Unitree_A1_recv_serial()
{
}
/**
 * @brief 获得来自于经过二次计算的moviet控制信息
 */
void Unitree_A1_recv_serial::callback_moveit_param(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p)
{
  // int i = msg_p->joint_trajectory.points.size();
  ros::Rate r(100);
  // for (int i = 0; i < msg_p->joint_trajectory.points.size(); i++)
  // {
  //   cout << msg_p->joint_trajectory.points[i].positions[1] << endl;
  //   motor_data_fresh_send(0.0, 0.0, msg_p->joint_trajectory.points[i].positions[1]*9.10f+ manual_zero*9.10f, 0.2, 3.0);
  //   r.sleep();
  // }
  float start=msg_p->joint_trajectory.points[0].positions[1];
  float end=msg_p->joint_trajectory.points[msg_p->joint_trajectory.points.size()-1].positions[1];
  int i=msg_p->joint_trajectory.points.size()*10;
  float step=(end-start)/i;
  cout<<"i: "<<i<<" step: "<<step<<endl;
  while (i--)
  {
    motor_data_fresh_send(0.0, 0.0, (end-step*i)*9.10f+ manual_zero*9.10f, 0.2, 3.0);
    r.sleep();
  }
  
  // cout<<msg_p->joint_trajectory<<endl;
}
/**
 * @brief          更新将要发送的数据(减速器输入端)
 * @param[in]      target_tor:     期望电机转矩 |T|<= 127.99 N*m
 * @param[in]      target_vel: 期望电机转速 |w|<= 255.99 rad/s
 * @param[in]      target_pos: 期望电机输出位置 |sita|<=823549 rad
 * @param[in]      Pos_PID_kP: 电机刚度系数 0<=kp<=15.99
 * @param[in]      VEL_PID_kD: 电机阻尼系数 0<=kd<=31.99
 * @retval         经过比例缩放的数据
 */
void Unitree_A1_recv_serial::motor_data_fresh_send(float target_tor, float target_vel, float target_pos, float Pos_PID_kP, float VEL_PID_kD)
{
  send_data.motor_send_data.Mdata.T = (int16_t)float_to_uint(target_tor, 256);
  send_data.motor_send_data.Mdata.W = (int16_t)float_to_uint(target_vel, 128);
  send_data.motor_send_data.Mdata.Pos = (int32_t)float_to_uint(target_pos, 8192 / 3.1415926535897);
  send_data.motor_send_data.Mdata.K_P = (int16_t)float_to_uint(Pos_PID_kP, 2048);
  send_data.motor_send_data.Mdata.K_W = (int16_t)float_to_uint(VEL_PID_kD, 1280);
  send_data.motor_send_data.CRCdata = crc32_core(send_data.sdata, 7);

  for (int k = 0; k < 34; k++)
  {
    // printf("0x/%02X_", send_data.c[k]);
    pub_data.data[k] = send_data.c[k];
  }
  pub.publish(pub_data);
  // cout << endl;
}
/**
 * @brief 回调函数
 * 
 * @param msg_p 
 */
void Unitree_A1_recv_serial::doMsg(const Unitree_A1::A1_send::ConstPtr &msg_p)
{
  ROS_INFO("i heard");
  for (int j = 0; j < 78; j++)
  {
    recv_data.sdata[j] = msg_p->data[j];
    // printf("0x/%02X_", recv_data.sdata[j]);
  }
  // cout << endl;
  // for (int j = 0; j < 4; j++)
  // {
  // printf("0x/%02X_", msg_p->data[30+j]);
  // }
  // cout << endl;
  int c = (msg_p->data[33] << 24) | (msg_p->data[32] << 16) | (msg_p->data[31] << 8) | (msg_p->data[30]);
  now_pos=uint_to_float(c, 8192 / 3.1415926535897) / 9.1f;
  cout<<"joint_2: "<<now_pos-manual_zero<<endl;
  RS485_recv.setParam("joint_2",now_pos-manual_zero); //浮点型
}

/**
 * @brief          float转int
 * @param[in]      x:     控制参数
 * @param[in]      times: 数据放大描述倍数,细分程度
 * @retval         经过比例缩放的数据
 */
int Unitree_A1_recv_serial::float_to_uint(float x, float times)
{
  return int(x * times);
}
/**
 * @brief          int转float
 * @param[in]      x:     控制参数
 * @param[in]      times: 数据放大描述倍数,细分程度
 * @retval         经过比例缩放的数据
 */
float Unitree_A1_recv_serial::uint_to_float(int x, float times)
{
  return x / times;
}
/**
 * @brief 初始化ROS句柄
 * 
 * @param _nh 
 */
void Unitree_A1_recv_serial::registerNodeHandle(ros::NodeHandle &_nh)
{
  RS485_recv = _nh;
}
/**
 * @brief 使能电机快速回零位
 */
void Unitree_A1_recv_serial::zero()
{
  // ros::Rate r(10);
  // while (ros::ok())
  // {
    motor_data_fresh_send(0.0, 0.0, 0.0 + manual_zero*9.1f, 0.2, 3.0);
  //   r.sleep();
  // }
}
/**
 * @brief 因为电机的特殊性，人为设置零点
 */
void Unitree_A1_recv_serial::set_manual_zero_point()
{
  zero_tor();
  manual_zero=now_pos;
  cout<<"manual zero: "<<manual_zero<<endl;
  txt_write(manual_zero);
  zero();
  RS485_recv.setParam("joint_2",now_pos-manual_zero); //浮点型
//   test();
}
/**
 * @brief 使能电机进入零力矩模式
 */
void Unitree_A1_recv_serial::zero_tor()
{
  ros::Rate r(10);
  int t = 10;
  while (t--)
  // while (ros::ok())
  {
    motor_data_fresh_send(0.0, 0.0, 0.0, 0.0, 0.0);
    // ROS_INFO("new_line");
    r.sleep();
  }
}
/**
 * @brief 测试 
 */
void Unitree_A1_recv_serial::test()
{
  float old_target_pos;
  while (ros::ok())
  {
    ros::Rate r(10);
    while (ros::ok())
    {
      // if (test_angle >= 3.1415926535897f * 9.10f)
      // {
      //   trun_flag = -1;
      // }
      // if (test_angle <= 0.0)
      // {
      //   trun_flag = 1;
      // }
      // if (trun_flag == -1)
      // {
      //   test_angle -= 0.001f * 9.10f;
      //   old_target_pos = 3.1415926535897f * 9.10f;
      // }
      // else
      // {
      //   test_angle += 0.001f * 9.10f;
      //   old_target_pos = 0.0;
      // }
      // test_tor = trun_flag * 10.0 * (0.8 * sin(test_angle / 9.10f) * sin(test_angle / 9.10f) + 0.2 * sin(test_angle / 9.10f));
      // test_v = 3.1415926 * trun_flag * sin(test_angle / 9.10f) * sin(test_angle / 9.10f);
      // float kp = 0.5 * (0.8 * sin(test_angle / 9.10f) + 0.2);
      // motor_data_fresh_send(test_tor, test_v, test_angle, kp, 3);
      // cout << test_angle / 9.1f << endl;
      motor_data_fresh_send(0.0, 0.0, 0.0, 0.0, 0.0);
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
  ros::init(argc, argv, "A1_control");
  ros::NodeHandle nh;
  Unitree_A1_recv_serial my_485;
  my_485.registerNodeHandle(nh);
  std::thread recv(&Unitree_A1_recv_serial::set_manual_zero_point, &my_485);
  // my_485.test();
  ros::spin();
}