#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/ros.h>

#include <iostream>
#include <string>
#include <sstream>
#include <math.h>

#include <can_read/Can.h>
#include <imu_serial/my_imu.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>

#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace message_filters;

class OdomWorkFlow
{
public:
  ros::NodeHandle n;
  ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("odom", 1);
  serial::Serial ops_ser;
  std::string ops_serial_data;
  float abs_now_ops_pos_x = 0;
  float abs_now_ops_pos_y = 0;
  float real_now_ops_pos_x = 0;
  float real_now_ops_pos_y = 0;
  float old_real_ops_pos_x = 0;
  float old_real_ops_pos_y = 0;
  float ops_ori_pos_x = 0;
  float ops_ori_pos_y = 0;

  int flag = 0;
  imu_serial::my_imu my_imu;
  nav_msgs::Odometry my_od;
  ros::Subscriber sub_imu = n.subscribe<imu_serial::my_imu>("Imu_pub", 1, &OdomWorkFlow::callBack_imu, this);

  void callBack_imu(const imu_serial::my_imu::ConstPtr &msg);
  void byte_to_float(float *px, float *py, std::string byte);
  void serial_read(serial::Serial &ops_ser, std::string &result);
  // float inverse(float rotation[4]);
  OdomWorkFlow();
  ros::Time _now, _old;
  double sample_time, ops_x_wheel_vel, ops_y_wheel_vel;
  double cmd_vel_vx, cmd_vel_vy, cmd_vel_wz;
  // 初始化PID
  typedef union
  {
    float fops_serial_data[6];
    uint8_t lops_serial_data[24];
  } Posture;
};
OdomWorkFlow::OdomWorkFlow(void)
{
  ops_ser.setPort("/dev/USB_ops");                           // 设置打开的串口名称
  ops_ser.setBaudrate(115200);                               // 设置串口的波特率
  serial::Timeout to = serial::Timeout::simpleTimeout(5000); // 创建timeout
  ops_ser.setTimeout(to);                                    // 设置串口的timeout
  // 打开串口
  try
  {
    ops_ser.open(); // 打开串口
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Unable to open OPS port "); // 打开串口失败，打印信息
  }
  if (ops_ser.isOpen())
  {
    ROS_INFO_STREAM("OPS Serial Port initialized."); // 成功打开串口，打印信息
  }
}
void OdomWorkFlow::serial_read(serial::Serial &ops_ser, std::string &result)
{
  result = ops_ser.read(ops_ser.available());
}
void OdomWorkFlow::callBack_imu(const imu_serial::my_imu::ConstPtr &msg)
{
  serial_read(ops_ser, ops_serial_data);
  // if (ops_serial_data.length() >0)
  // {
  //   cout<<ops_serial_data.length()<<endl;
  // }
  if (ops_serial_data.length() == 28 & flag == 0)
  {
    if (ops_serial_data[0] == 0xD & ops_serial_data[1] == 0xA)
    {
      byte_to_float(&ops_ori_pos_x, &ops_ori_pos_y, ops_serial_data);
      _old = _now = ros::Time::now();
      old_real_ops_pos_x = ops_ori_pos_x - ops_ori_pos_x;
      old_real_ops_pos_x = ops_ori_pos_x - ops_ori_pos_y;
      flag = 1;
    }
  }
  if (ops_serial_data.length() == 28 & flag == 1)
  {
    byte_to_float(&abs_now_ops_pos_x, &abs_now_ops_pos_y, ops_serial_data);
    real_now_ops_pos_x = abs_now_ops_pos_x - ops_ori_pos_x;
    real_now_ops_pos_y = abs_now_ops_pos_y - ops_ori_pos_y;
    my_od.header.frame_id = "my_Odom";
    my_od.child_frame_id = "basefoot_print";
    my_od.pose.pose.orientation = msg->Imu.orientation;
    my_od.twist.twist.angular = msg->Imu.angular_velocity;
    my_od.header.stamp = _now = ros::Time::now();
    sample_time = (_now - _old).toSec();
    ops_x_wheel_vel = (real_now_ops_pos_x - old_real_ops_pos_x) / sample_time;
    ops_y_wheel_vel = (real_now_ops_pos_y - old_real_ops_pos_y) / sample_time;
    old_real_ops_pos_x = real_now_ops_pos_x;
    old_real_ops_pos_y = real_now_ops_pos_y;
    my_od.twist.twist.linear.x = ops_x_wheel_vel;
    my_od.twist.twist.linear.y = ops_y_wheel_vel;
    my_od.twist.twist.linear.z = 0;
    my_od.pose.pose.position.x = real_now_ops_pos_x;
    my_od.pose.pose.position.y = real_now_ops_pos_y;
    my_od.pose.pose.position.z = 0;
    pub_odom.publish(my_od);
    _old = _now;
    // cout<<"odom ok"<<endl;
  }
}
void OdomWorkFlow::byte_to_float(float *px, float *py, std::string byte)
{
  Posture pos;
  for (int i = 0; i < 24; i++)
  {
    pos.lops_serial_data[i] = byte[i + 2];
  }
  *px = pos.fops_serial_data[3] / 1000.0f;
  *py = pos.fops_serial_data[4] / 1000.0f;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "chassis_node");
  OdomWorkFlow my_odom;
  ros::spin();
  return 0;
}
