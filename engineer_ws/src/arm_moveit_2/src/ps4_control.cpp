#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <thread>
#include <sensor_msgs/Joy.h>
#include <tf/tf.h>
using namespace std;
class my_clock
{
private:
    ros::Time start_time;

public:
    my_clock(/* args */);
    ~my_clock();
    void reset_start_time();
    double get_duration();
};

my_clock::my_clock(/* args */)
{
    start_time = ros::Time::now();
}

my_clock::~my_clock()
{
}
void my_clock::reset_start_time()
{
    start_time = ros::Time::now();
}
double my_clock::get_duration()
{
    return (ros::Time::now() - start_time).toSec();
}
class ps4_control
{
private:
    /* data */
public:
    ps4_control(std::string name) : arm(name)
    {
        std::string end_effector_link = arm.getEndEffectorLink();
        std::string reference_frame = "base_link";
        arm.setPoseReferenceFrame(reference_frame);
        arm.allowReplanning(true);
        arm.setGoalPositionTolerance(0.001);
        arm.setGoalOrientationTolerance(0.01);
        sub = Can_recv.subscribe<sensor_msgs::Joy>("joy", 10, &ps4_control::callback, this);
        Can_recv.param<int>("axis_x", axis_linear_x, 1);
        Can_recv.param<int>("axis_y", axis_linear_y, 0);
        Can_recv.param<int>("axis_z", axis_4, 4);
        Can_recv.param<int>("axis_z_w", axis_3, 3);
        Can_recv.param<int>("shift", shift, 5);

        axis_x = axis_y = axis_z = axis_z_w = axis_shift = 0.0f;
    }
    ~ps4_control();
    void control();
    void make_plan();
    void execute_plan();
    // void callback(const sensor_msgs::Joy::ConstPtr &joy);
    void registerNodeHandle(ros::NodeHandle &_nh);
    void callback(const sensor_msgs::Joy::ConstPtr &joy);
    void my_spin();
    void fresh_pose();
    geometry_msgs::Pose target_pose, now_pose;
    geometry_msgs::Quaternion q;
    tf::Quaternion RQ2;
    moveit::planning_interface::MoveGroupInterface arm;
    ros::Subscriber sub;
    ros::NodeHandle Can_recv;
    moveit::planning_interface::MoveItErrorCode success;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    int axis_linear_x, axis_linear_y, axis_angular_z, axis_4, axis_2, axis_3, axis_5, axis_6, axis_7, shift;
    double target_x, target_y, target_z, target_pitch, target_yaw, target_roll;
    double now_x, now_y, now_z, now_pitch, now_yaw, now_roll;
    double axis_x, axis_y, axis_z, axis_z_w, axis_shift;
};

ps4_control::~ps4_control()
{
}

void ps4_control::make_plan()
{
    while (ros::ok())
    {
        arm.setStartStateToCurrentState();
        arm.setPoseTarget(target_pose);
        success = arm.plan(plan);
        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");

        now_pose = arm.getCurrentPose().pose;
        target_x = now_x = now_pose.position.x;
        target_y = now_y = now_pose.position.y;
        target_z = now_z = now_pose.position.z;
        tf::quaternionMsgToTF(now_pose.orientation, RQ2);
        tf::Matrix3x3(RQ2).getRPY(now_roll, now_pitch, now_yaw);
        target_roll=now_roll;
        target_pitch=now_pitch;
        target_yaw=now_yaw;
    }
}
void ps4_control::execute_plan()
{
    while (ros::ok())
    {
        if (success)
        {
            arm.execute(plan);
        }
    }
}
// void ps4_control::callback()
// {
//     while (ros::ok())
//     {
//         Can_recv.getParam("orientation_x", target_pose.orientation.x);
//         Can_recv.getParam("orientation_y", target_pose.orientation.y);
//         Can_recv.getParam("orientation_z", target_pose.orientation.z);
//         Can_recv.getParam("orientation_w", target_pose.orientation.w);
//         Can_recv.getParam("position_x", target_pose.position.x);
//         Can_recv.getParam("position_y", target_pose.position.y);
//         Can_recv.getParam("position_z", target_pose.position.z);
//         cout << target_pose << endl;
//     }
// }
void ps4_control::callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // ROS_INFO("%5.3lf,%5.3lf,%5.3lf,%5.3lf,%5.3lf,%5.3lf,%5.3lf", joy->axes[axis_2], joy->axes[axis_3], joy->axes[axis_4], joy->axes[axis_5], joy->axes[axis_6], joy->axes[axis_7]);
    axis_shift = joy->axes[shift];
    axis_x = joy->axes[axis_linear_x];
    axis_y = joy->axes[axis_linear_y];
    axis_z = joy->axes[axis_4];
    axis_z_w = joy->axes[axis_3];
}
void ps4_control::my_spin()
{
    ros::Rate r(100);
    while (ros::ok())
    {
        if (axis_shift >= 0)
        {
            target_x += axis_x / 100;
            target_y += axis_y / 100;
            target_z += axis_z / 100;
        }
        else
        {
            target_roll += axis_x / 100;
            target_pitch += axis_y / 100;
            target_yaw += axis_z / 100;
        }
        // cout << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << endl;
        target_pose.position.x=target_x;
        target_pose.position.y=target_y;
        target_pose.position.z=target_z;
        target_pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(target_roll,target_pitch,target_yaw);
        cout<<"target_pose"<<target_pose<<endl;
        cout<<"now_pose"<<now_pose<<endl;

        r.sleep();
    }
}
void ps4_control::fresh_pose()
{
    ros::Rate r(30);
    while (ros::ok())
    {
        now_pose = arm.getCurrentPose().pose;
        target_x = now_x = now_pose.position.x;
        target_y = now_y = now_pose.position.y;
        target_z = now_z = now_pose.position.z;
        tf::quaternionMsgToTF(now_pose.orientation, RQ2);
        tf::Matrix3x3(RQ2).getRPY(now_roll, now_pitch, now_yaw);
        target_roll=now_roll;
        target_pitch=now_pitch;
        target_yaw=now_yaw;
        r.sleep();
    }
}
/**
 * @brief 初始化ROS句柄
 * 
 * @param _nh 在main中声明的句柄
 */
void ps4_control::registerNodeHandle(ros::NodeHandle &_nh)
{
    Can_recv = _nh;
}
int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "moveit_fk_demo");
    ps4_control my_ctl("s_2_arm");
    ros::NodeHandle nh;
    my_ctl.registerNodeHandle(nh);
    std::thread recv1(&ps4_control::my_spin, &my_ctl);
    // std::thread recv2(&ps4_control::fresh_pose, &my_ctl);
    std::thread recv3(&ps4_control::make_plan, &my_ctl);
    std::thread recv4(&ps4_control::execute_plan, &my_ctl);
    ros::spin();

    return 0;
}