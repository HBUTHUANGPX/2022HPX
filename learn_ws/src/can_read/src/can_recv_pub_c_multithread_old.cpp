
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>
#include "ros/ros.h"
#include <can_read/Can.h>
#include <iostream>
#include <typeinfo>
#include <std_msgs/Header.h>
class CanWorkFlow
{
public:
    ros::NodeHandle Can_recv;
    can_read::Can new_recv;
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct sockaddr_can addr;
    int sum = 0;
    int id;
    unsigned int a, b;
    ros::Publisher pub_moto;
    void receiveThread()
    {
        struct can_frame frame;
        struct sockaddr_can addr;
        socklen_t len = 0;
        int ret;
        ros::Rate r(1000);
        int n=0;
        while (ros::ok())
        {
            ret = recvfrom(s, &frame, sizeof(struct can_frame), 0, (struct sockaddr *)&addr, &len);
            if (ret > 0)
            {
                sum = 0;
                if (frame.can_id == id)
                {
                    std::cout<<"id:"<<frame.can_id<<" data:"<<&frame.data<<std::endl;
                    new_recv.hex_=&frame.data;
                    // for (uint8_t i = 0; i < frame.can_dlc; ++i)
                    // {
                    //     new_recv.hex_[i] = frame.data[i];
                    //     sum += frame.data[i];
                    // }
                    // if (sum == frame.data[0])
                    // {
                    //     continue;
                    // }
                    new_recv.header.frame_id = frame.can_id;
                    new_recv.header.stamp = ros::Time::now();
                    a = new_recv.hex_[0] << 8 | new_recv.hex_[1];
                    new_recv.encoder = a;
                    b = new_recv.hex_[2] << 8 | new_recv.hex_[3];
                    new_recv.velotic = b;
                    pub_moto.publish(new_recv);
                    n++;
                    std::cout<<id<<":"<<n<<",time:"<<new_recv.header.stamp<<std::endl;
                }
            }
            ros::spinOnce();
        }
    }
    void init(int ID, const char *TOPIC)
    {
        id = ID;
        addr.can_family = AF_CAN;
        addr.can_ifindex = 0; // 关键点, 接口索引为0 ！！！
        bind(s, (struct sockaddr *)&addr, sizeof(addr));
        new_recv.header.seq = 1;
        pub_moto = Can_recv.advertise<can_read::Can>(TOPIC, 1);
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Can_recv_publisher");
    CanWorkFlow my_can1, my_can2, my_can3, my_can4;
    my_can1.init(0x201, "Can_moto1");
    my_can2.init(0x202, "Can_moto2");
    my_can3.init(0x203, "Can_moto3");
    my_can4.init(0x204, "Can_moto4");
    // 开启接收线程
    std::thread t1(&CanWorkFlow::receiveThread, &my_can1);
    std::thread t2(&CanWorkFlow::receiveThread, &my_can2);
    std::thread t3(&CanWorkFlow::receiveThread, &my_can3);
    std::thread t4(&CanWorkFlow::receiveThread, &my_can4);
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    return 0;
}
