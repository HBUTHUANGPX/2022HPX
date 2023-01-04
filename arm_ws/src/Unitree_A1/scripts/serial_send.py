import serial
import os
import rospy
import time
from Unitree_A1.msg import A1_recv
from Unitree_A1.msg import A1_send
import os
import multiprocessing
import threading
class A1_serial_sub:
    def __init__(self):
        rospy.init_node("A1_serial")
        self.sub = rospy.Subscriber("A1_recv_data",A1_recv,self.callback,queue_size=10)
        self.pub = rospy.Publisher("A1_send_data",A1_send,queue_size=1) 
        self.ser = serial.Serial("/dev/ttyCH9344USB0", 4800000, timeout=0.5)  # 使用USB连接串行口
        self.pub_msg =A1_send()

    def callback(self,A1):
        self.ser.write(A1.data)
        # print("new ",A1.data)
    def serial_recv(self):
        while not rospy.is_shutdown():
            # print("ser close")

            try:
                new_recv=self.ser.read(78)
                if len(new_recv):
                    print(time.time(),len(new_recv))
                    self.pub_msg.data=new_recv
                    self.pub.publish(self.pub_msg)
            except KeyboardInterrupt:
                print("ser close")
                self.ser.close()
                break






if __name__ == '__main__':
    # rospy.init_node("echo_sys",anonymous=True)
    #2.创建订阅者对象
    try:
        password = '0000'
        cmdline = ['chmod -R 777 /dev/tty*']
        for i in cmdline:
            os.system("echo %s | sudo -S %s" % (password, i))
        os.system("ls /dev/ttyCH9344*")
    except:
        pass
    

    my_sub = A1_serial_sub()
    recv_process=threading.Thread(target=my_sub.serial_recv)
    recv_process.start()
    rospy.spin() #4.循环
    
    