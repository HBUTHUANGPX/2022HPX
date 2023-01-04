#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include "moveit_msgs/RobotTrajectory.h"
#include <sensor_msgs/JointState.h>
#include <thread>
using namespace std;
// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class action_ser
{
public:
    void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goalPtr, Server *moveit_server);
    void param();
    ros::NodeHandle nh;
    moveit_msgs::RobotTrajectory moveit_tra;
    ros::Publisher pub,pub2;
    Server moveit_server;
    sensor_msgs::JointState joint_state;
    float slide_joint,joint_1, joint_2, joint_3, joint_4, joint_5, joint_6;
    action_ser(std::string name) : moveit_server(nh, name, boost::bind(&action_ser::execute_callback, this, _1, &moveit_server), false)
    {
        pub = nh.advertise<moveit_msgs::RobotTrajectory>("chatter", 10);
        pub2 = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
        moveit_server.start();
    }
    ~action_ser(void)
    {
    }
};
void action_ser::execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goalPtr, Server *moveit_server)
{
    // 1、解析提交的目标值
    int n_joints = goalPtr->trajectory.joint_names.size();
    int n_tra_Points = goalPtr->trajectory.points.size();

    moveit_tra.joint_trajectory.header.frame_id = goalPtr->trajectory.header.frame_id;
    moveit_tra.joint_trajectory.joint_names = goalPtr->trajectory.joint_names;
    moveit_tra.joint_trajectory.points.resize(n_tra_Points);

    for (int i = 0; i < n_tra_Points; i++) // 遍历每组路点
    {
        moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);

        moveit_tra.joint_trajectory.points[i].time_from_start = goalPtr->trajectory.points[i].time_from_start;
        for (int j = 0; j < n_joints; j++) // 遍历每组路点中的每个关节数据
        {
            moveit_tra.joint_trajectory.points[i].positions[j] = goalPtr->trajectory.points[i].positions[j];
            moveit_tra.joint_trajectory.points[i].velocities[j] = goalPtr->trajectory.points[i].velocities[j];
            moveit_tra.joint_trajectory.points[i].accelerations[j] = goalPtr->trajectory.points[i].accelerations[j];
        }
    }
    pub.publish(moveit_tra);
    moveit_server->setSucceeded();
}
void action_ser::param()
{
    ros::Rate r(10);
    while (ros::ok())
    {
        nh.getParam("slide_joint", slide_joint);
        nh.getParam("joint_1", joint_1);
        nh.getParam("joint_2", joint_2);
        nh.getParam("joint_3", joint_3);
        nh.getParam("joint_4", joint_4);
        nh.getParam("joint_5", joint_5);
        nh.getParam("joint_6", joint_6);
        joint_state.header.stamp=ros::Time::now();
        joint_state.name.resize(7);
        joint_state.position.resize(7);
        joint_state.name[6]="slide_joint";
        joint_state.name[0]="joint_1";
        joint_state.name[1]="joint_2";
        joint_state.name[2]="joint_3";
        joint_state.name[3]="joint_4";
        joint_state.name[4]="joint_5";
        joint_state.name[5]="joint_6";
        joint_state.position[6]=slide_joint;
        joint_state.position[0]=joint_1;
        joint_state.position[1]=joint_2;
        joint_state.position[2]=joint_3;
        joint_state.position[3]=joint_4;
        joint_state.position[4]=joint_5;
        joint_state.position[5]=joint_6;
        pub2.publish(joint_state);
        r.sleep();
        cout<<joint_state<<endl;
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moveit_action_server");
    // 手动启动服务器
    action_ser my_ser("my_controller/follow_joint_trajectory");
    std::thread recv(&action_ser::param, &my_ser);
    ros::spin();
    return 0;
}
