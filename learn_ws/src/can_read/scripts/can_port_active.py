#!/usr/bin/env python3

import can
import rospy
import os
from can_read.msg import Can
if __name__=="__main__":
    try:
        password = '0000'
        cmdline = ['ip link set can0 down',
                'busybox devmem 0x0c303000 32 0x0000C400',
                'busybox devmem 0x0c303008 32 0x0000C458',
                'busybox devmem 0x0c303010 32 0x0000C400',
                'busybox devmem 0x0c303018 32 0x0000C458',
                'modprobe can',
                'modprobe can_raw',
                'modprobe can_dev',
                'modprobe mttcan',
                'ip link set can0 type can bitrate 1000000',
                'ip link set up can0',]
        for i in cmdline:
            os.system("echo %s | sudo -S %s"%(password,i))
    except:
        pass