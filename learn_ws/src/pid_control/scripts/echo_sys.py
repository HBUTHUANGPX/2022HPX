import os
import rospy

if __name__=='__main__':
    rospy.init_node("echo_sys",anonymous=True)

    try:
        password = '0000'
        cmdline = ['chmod -R 777 /dev/ttyU*',
                   'ip link set can0 down',
                'busybox devmem 0x0c303000 32 0x0000C400',
                'busybox devmem 0x0c303008 32 0x0000C458',
                'busybox devmem 0x0c303010 32 0x0000C400',
                'busybox devmem 0x0c303018 32 0x0000C458',
                'modprobe can',
                'modprobe can_raw',
                'modprobe can_dev',
                'modprobe mttcan',
                'ip link set can0 type can bitrate 1000000',
                'ip link set up can0']
        for i in cmdline:
            os.system("echo %s | sudo -S %s"%(password,i))
    except:
        pass