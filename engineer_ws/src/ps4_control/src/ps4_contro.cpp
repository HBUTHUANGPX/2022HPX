#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <thread>
using namespace std;

class TeleopTurtle
{
public:
    TeleopTurtle();
    void my_spin();

private:
    // 处理手柄发送过来的信息
    void callback(const sensor_msgs::Joy::ConstPtr &joy);
    // 实例化ROS句柄
    ros::NodeHandle nh;
    // 定义订阅者对象，用来订阅手柄发送的数据
    ros::Subscriber sub;
    // 定义发布者对象，用来将手柄数据发布到乌龟控制话题上
    ros::Publisher pub;
    // 用来接收launch文件中设置的参数，绑定手柄摇杆、轴的映射
    int axis_linear_x, axis_linear_y, axis_angular_z, axis_4, axis_2, axis_3, axis_5, axis_6, axis_7, shift;
    float x, y, z, pitch, yaw, roll;
    float axis_x, axis_y, axis_z, axis_z_w, axis_shift;
};

TeleopTurtle::TeleopTurtle()
{
    // 从参数服务器读取的参数
    nh.param<int>("axis_x", axis_linear_x, 1);
    nh.param<int>("axis_y", axis_linear_y, 0);
    nh.param<int>("axis_z", axis_4, 4);
    nh.param<int>("axis_z_w", axis_3, 3);
    nh.param<int>("shift", shift, 5);

    pub = nh.advertise<geometry_msgs::Pose>("/cmd_vel", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::callback, this);
    axis_x= axis_y= axis_z= axis_z_w= axis_shift=x = y = z = pitch = yaw =  0.0f;
    roll=0.0f;
}

void TeleopTurtle::callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Pose vel;
    // ROS_INFO("%5.3lf,%5.3lf,%5.3lf,%5.3lf,%5.3lf,%5.3lf,%5.3lf", joy->axes[axis_2], joy->axes[axis_3], joy->axes[axis_4], joy->axes[axis_5], joy->axes[axis_6], joy->axes[axis_7]);
    axis_shift = joy->axes[shift];
    axis_x = joy->axes[axis_linear_x];
    axis_y = joy->axes[axis_linear_y];
    axis_z = joy->axes[axis_4];
    axis_z_w = joy->axes[axis_3];
}
void TeleopTurtle::my_spin()
{
    ros::Rate r(100);
    while (ros::ok())
    {
        if (axis_shift >= 0)
        {
            x += axis_x/100;
            y += axis_y/100;
            z += axis_z/100;
        }
        else
        {
            roll += axis_x/100;
            pitch += axis_y/100;
            yaw += axis_z/100;
        }
        cout<<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<endl;
        r.sleep();
    }
}
int main(int argc, char **argv)
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "teleop_ps4");
    TeleopTurtle teleopTurtle;
    std::thread recv(&TeleopTurtle::my_spin, &teleopTurtle);

    ros::spin();
    return 0;
}
