#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
// #include <serial/serial.h>
#include "moveit_msgs/RobotTrajectory.h"

#include <thread>
using namespace std;
class fake_joint
{
private:
    /* data */
    ros::NodeHandle fake_handle;
    ros::Subscriber sub;
    float joint_pose[7];
    int points_size, for_i, for_j, for_k;

public:
    fake_joint(/* args */);
    ~fake_joint();
    void callback(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p);
    void fresh_param();
};

fake_joint::fake_joint(/* args */)
{
    sub = fake_handle.subscribe<moveit_msgs::RobotTrajectory>("chatter", 1, &fake_joint::callback, this);
    for (int i = 0; i < 7; i++)
    {
        joint_pose[i] = 0.0f;
    }
    points_size = 0;
    fresh_param();
}

fake_joint::~fake_joint()
{
}
void fake_joint::fresh_param()
{
    // cout<<"start fresh"<<endl;
    fake_handle.setParam("slide_joint", joint_pose[6]);
    fake_handle.setParam("joint_1", joint_pose[0]);
    fake_handle.setParam("joint_2", joint_pose[1]);
    fake_handle.setParam("joint_3", joint_pose[2]);
    fake_handle.setParam("joint_4", joint_pose[3]);
    fake_handle.setParam("joint_5", joint_pose[4]);
    fake_handle.setParam("joint_6", joint_pose[5]);
    // cout<<"end fresh"<<endl;
}
void fake_joint::callback(const moveit_msgs::RobotTrajectory::ConstPtr &msg_p)
{
    ros::Rate r(1000);
    points_size = msg_p->joint_trajectory.points.size();
    cout << msg_p->joint_trajectory << endl;
    for (for_j = 0; for_j < points_size * 100 ; for_j++)
    {
        // cout<<for_j<<endl;
        cout << "=======" << endl;
        for (for_i = 0; for_i < 7; for_i++)
        {
            joint_pose[for_i] = (-msg_p->joint_trajectory.points[0].positions[for_i] + msg_p->joint_trajectory.points[points_size - 1].positions[for_i]) / (100*points_size) * for_j + msg_p->joint_trajectory.points[0].positions[for_i];
            // if (for_i == 6)
            // {
            cout << for_i << " " << joint_pose[for_i] << " " << msg_p->joint_trajectory.points[0].positions[for_i] << " " << msg_p->joint_trajectory.points[points_size - 1].positions[for_i] << endl;
            // }
        }
        fresh_param();
        r.sleep();
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "listener");
    fake_joint my_node;
    ros::spin(); //循环读取接收的数据，并调用回调函数处理
    return 0;
}
