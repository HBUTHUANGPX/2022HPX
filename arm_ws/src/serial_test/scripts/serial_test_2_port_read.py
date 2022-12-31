import serial
import os
import rospy
import time

if __name__ == '__main__':
    # rospy.init_node("echo_sys",anonymous=True)

    try:
        password = '0000'
        cmdline = ['chmod -R 777 /dev/tty*']
        for i in cmdline:
            os.system("echo %s | sudo -S %s" % (password, i))
        os.system("ls /dev/ttyCH9344*")
    except:
        pass
    ser2 = serial.Serial("/dev/ttyCH9344USB0", 4800000, timeout=0.5)  # 使用USB连接串行口
    while True:
        read = ser2.read(34)
        print(len(read))
        if len(read):
            print(time.time(),read,len(read))
