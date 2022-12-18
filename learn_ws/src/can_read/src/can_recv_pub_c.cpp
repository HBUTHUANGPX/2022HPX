
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
    ros::Publisher pub_moto1 = Can_recv.advertise<can_read::Can>("Can_moto1", 10);
    ros::Publisher pub_moto2 = Can_recv.advertise<can_read::Can>("Can_moto2", 10);
    ros::Publisher pub_moto3 = Can_recv.advertise<can_read::Can>("Can_moto3", 10);
    ros::Publisher pub_moto4 = Can_recv.advertise<can_read::Can>("Can_moto4", 10);
    can_read::Can new_recv_1, new_recv_2, new_recv_3, new_recv_4;
    // new_recv=[new_recv_1, new_recv_2, new_recv_3, new_recv_4];
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    struct sockaddr_can addr;
    int sum = 0;
    unsigned short a,b;
    void receiveThread()
    {
        while (ros::ok())
        {
            struct can_frame frame;
            struct sockaddr_can addr;
            struct ifreq ifr;
            socklen_t len = 0;
            int ret = recvfrom(s, &frame, sizeof(struct can_frame), 0, (struct sockaddr *)&addr, &len);
            if (ret > 0)
            {
                sum = 0;

                if (frame.can_id == 0x201)
                {
                    // for (uint8_t i = 0; i < frame.can_dlc; ++i)
                    // {
                    //     new_recv_1.hex_[i] = frame.data[i];
                    //     sum += frame.data[i];
                    // }
                    if (sum == frame.data[0])
                    {
                        continue;
                    }
                    new_recv_1.header.frame_id = frame.can_id;
                    new_recv_1.header.stamp = ros::Time::now();
                    a = frame.data[0] << 8 | frame.data[1];
                    new_recv_1.encoder = a;
                    b = frame.data[2] << 8 | frame.data[3];
                    new_recv_1.velotic = b;
                    pub_moto1.publish(new_recv_1);
                }
                else if(frame.can_id == 0x202)
                {
                    // for (uint8_t i = 0; i < frame.can_dlc; ++i)
                    // {
                    //     new_recv_2.hex_[i] = frame.data[i];
                    //     sum += frame.data[i];
                    // }
                    if (sum == frame.data[0])
                    {
                        continue;
                    }
                    new_recv_2.header.frame_id = frame.can_id;
                    new_recv_2.header.stamp = ros::Time::now();
                    a = frame.data[0] << 8 | frame.data[1];
                    new_recv_2.encoder = a;
                    b = frame.data[2] << 8 | frame.data[3];
                    new_recv_2.velotic = b;
                    pub_moto2.publish(new_recv_2);
                }
                else if(frame.can_id == 0x203)
                {
                    // for (uint8_t i = 0; i < frame.can_dlc; ++i)
                    // {
                    //     new_recv_3.hex_[i] = frame.data[i];
                    //     sum += frame.data[i];
                    // }
                    if (sum == frame.data[0])
                    {
                        continue;
                    }
                    new_recv_3.header.frame_id = frame.can_id;
                    new_recv_3.header.stamp = ros::Time::now();
                    a = frame.data[0] << 8 | frame.data[1];
                    new_recv_3.encoder = a;
                    b = frame.data[2] << 8 | frame.data[3];
                    new_recv_3.velotic = b;
                    pub_moto3.publish(new_recv_3);
                }
                else if(frame.can_id == 0x204)
                {
                    // for (uint8_t i = 0; i < frame.can_dlc; ++i)
                    // {
                    //     new_recv_4.hex_[i] = frame.data[i];
                    //     sum += frame.data[i];
                    // }
                    if (sum == frame.data[0])
                    {
                        continue;
                    }
                    new_recv_4.header.frame_id = frame.can_id;
                    new_recv_4.header.stamp = ros::Time::now();
                    a = frame.data[0] << 8 | frame.data[1];
                    new_recv_4.encoder = a;
                    b = frame.data[2] << 8 | frame.data[3];
                    new_recv_4.velotic = b;
                    pub_moto4.publish(new_recv_4);
                }
            }
        }
        close(s);
    }
    void init()
    {
        addr.can_family = AF_CAN;
        addr.can_ifindex = 0; // 关键点, 接口索引为0 ！！！
        bind(s, (struct sockaddr *)&addr, sizeof(addr));
        new_recv_1.header.seq = 1;
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Can_recv_publisher");
    CanWorkFlow my_can0;
    my_can0.init();
    // 开启接收线程
    std::thread t(&CanWorkFlow::receiveThread, &my_can0);
    t.join();
    return 0;
}
