import serial
import os
import rospy
import time
class my_clock:
    def __init__(self) -> None:
        self.start_time=time.time()
        self.site_time=time.time()
        pass
    def site(self):
        self.site_time=time.time()
        pass
    def diff(self):

        return time.time()-self.site_time
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
    ser2 = serial.Serial("/dev/ttyCH9344USB3", 4800000, timeout=0.5)  # 使用USB连接串行口
    clc=my_clock()
    num=0
    while 1:
        if clc.diff()>0.1:
            ser2.write(("hello\n").encode())
            clc.site()
            num+=1
            # print(num)
        if num==40:
            break
    