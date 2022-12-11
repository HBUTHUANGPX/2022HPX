from math import degrees
import queue
from statistics import multimode
from turtle import right
from xmlrpc.client import MultiCall
import rospy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import tf
from tf.transformations import euler_from_quaternion
import message_filters
from pid_control.msg import Can
import os
import can
import time
import numpy as np
from pid_control.msg import PS4

class PID_init():
    def __init__(self, mode, PID_param):
        self.mode = mode
        self.Kp = PID_param[0]
        self.Ki = PID_param[1]
        self.Kd = PID_param[2]
        self.max_out = PID_param[3]
        self.min_iout = PID_param[4]
        self.Dbuf = np.array([0.0, 0.0, 0.0])
        self.error = np.array([0.0, 0.0, 0.0])
        self.Pout = 0.0
        self.Iout = 0.0
        self.Dout = 0.0
        self.out = 0.0
        self.set = 0.0
        self.fdb = 0.0

    def PID_calculate(self, ref, set):
        self.error[2] = self.error[1]
        self.error[1] = self.error[0]
        self.set = set
        self.fdb = ref
        self.error[0] = set-ref
        if (self.mode == 0):
            self.Pout = self.Kp*self.error[0]
            self.Iout += self.Ki*self.error[0]
            self.Dbuf[2] = self.Dbuf[1]
            self.Dbuf[1] = self.Dbuf[0]
            self.Dbuf[0] = self.error[0]-self.error[1]
            self.Dout = self.Kd*self.Dbuf[0]
            self.LimitMax(self.Iout, self.max_out)
            if (self.Iout > self.max_out):self.Iout = self.max_out
            elif (self.Iout < -self.max_out):self.Iout = -self.max_out
            self.out = self.Pout+self.Iout+self.Dout
            if (self.out > self.max_out):self.out = self.max_out
            elif (self.out < -self.max_out):self.out = -self.max_out
        elif (self.mode == 1):
            self.Pout = self.Kp*(self.error[0]-self.error[1])
            self.Iout = self.Ki*self.error[0]
            self.Dbuf[2] = self.Dbuf[1]
            self.Dbuf[1] = self.Dbuf[0]
            self.Dbuf[0] = self.error[0]-2*self.error[1]+self.error[2]
            self.Dout = self.Kd*self.Dbuf[0]
            self.out += self.Pout+self.Iout+self.Dout
            if (self.out > self.max_out):self.out = self.max_out
            elif (self.out < -self.max_out):self.out = -self.max_out
        return self.out

    def LimitMax(self, input, max):
        if (input > max):input = max
        elif (input < -max):input = -max


class my_pid():
    def __init__(self):

        # PID init
        self.init_PID()

        # Imu date
        self.my_imu = None
        # encoder
        self.encoder_left_front = None
        self.encoder_left_back = None
        self.encoder_right_front = None
        self.encoder_right_back = None
        # velotic
        self.velotic_left_front = None
        self.velotic_right_front = None
        self.velotic_left_back = None
        self.velotic_right_back = None
        # ml
        self.rx = 0.2042
        self.ry = 0.1838
        self.R = 0.2747
        self.sum_r = self.rx+self.ry
        self.z_arr = np.array([[-1, -1, -self.sum_r],
                               [1, -1, -self.sum_r],
                               [1, 1, -self.sum_r],
                               [-1, 1, -self.sum_r]])
        self.n_arr = np.array([[-1,            1,           1,          -1],
                               [-1,           -1,           1,           1],
                               [-1/self.sum_r, -1/self.sum_r, -1/self.sum_r, -1/self.sum_r]])
        # fake speed
        self.v=np.array([[1],[0.0],[0.0]])
        self.can_date = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.PS4_param=[0,0,0,0,0,0,0,0]
        self.PS4_mode=0
    def quaternion2euler(self):
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.my_imu.orientation.x,
                                                                   self.my_imu.orientation.y,
                                                                   self.my_imu.orientation.z,
                                                                   self.my_imu.orientation.w])

    def init_motor_speed_PID(self):
        self.motor_param = np.array([10000.0, 10.0, 0.0, 6000.0, 2000.0])
        self.motor_speed_PID = [PID_init(0, self.motor_param),PID_init(0, self.motor_param),PID_init(0, self.motor_param),PID_init(0, self.motor_param)]
        print("motor speed PID has init")
    def init_chassis_speed_PID(self):
        self.chassis_speed_param = np.array([8.0, 0.0, 0.2, 3.0, 0.2])
        self.chassis_speed_PID = PID_init(0, self.chassis_speed_param)
        print("chassis speed PID has init")

    def init_chassis_angle_PID(self):
        self.chassis_angle_param = np.array([8.0, 0.0, 0.2, 50.50, 100.2])
        self.chassis_angle_PID = PID_init(0, self.chassis_angle_param)
        print("chassis angle PID has init")
    
    def init_PID(self):
        print("Start to init PID")
        
        self.init_motor_speed_PID()
        self.init_chassis_angle_PID()
        self.init_chassis_speed_PID()
        print("PID has init")

    def zhengjie(self, arr_v):
        '''
        Input target speed_x speed_y w_z,   
            np.array([[speed_x],[speed_y],[w_z]])
        obtain 4 rotation
        '''
        return np.dot(self.z_arr, arr_v)/self.R

    def nijie(self, arr_w):
        '''
        Input newly w ,   
            np.array([[w1],[w2],[w3],[w4]])
        obtain speed_x speed_y w_z
        '''
        return np.dot(self.n_arr, arr_w)*self.R/4

    def output(self):
        '''
        the output block
        '''
        
        if self.PS4_mode==1:
            self.v[0]=self.PS4_param[0]
            self.v[1]=self.PS4_param[1]
            self.w = self.zhengjie(self.v)
            for i in range(4):
                out=int(self.motor_speed_PID[i].PID_calculate(self.now_w[i],self.w[i])+6000)
                self.can_date[i*2]=(out>>8)
                self.can_date[i*2+1]=((out>>8)<<8)^out
            print(self.can_date)
        if self.PS4_mode==0:
            self.can_date=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
            pass
        print(self.can_date)
        self.msg = can.Message(arbitration_id=0x200,data=self.can_date, is_extended_id=False)
        self.can0.send(self.msg)
    
    def imu_fresh(self, new_imu):
        '''
        the block of fresh imu
        '''
        self.my_imu = new_imu
        self.quaternion2euler()

    def motor_fresh(self, sub_Can_1, sub_Can_2, sub_Can_3, sub_Can_4):
        self.encoder_left_front = sub_Can_4.encoder
        self.encoder_right_front = sub_Can_3.encoder
        self.encoder_left_back = sub_Can_1.encoder
        self.encoder_right_back = sub_Can_2.encoder

        self.velotic_left_front = sub_Can_4.velotic
        self.velotic_right_front = sub_Can_3.velotic
        self.velotic_left_back = sub_Can_1.velotic
        self.velotic_right_back = sub_Can_2.velotic
        self.now_w = np.array([[sub_Can_1.velotic], [sub_Can_1.velotic], [sub_Can_3.velotic], [sub_Can_4.velotic]])
        self.now_v = self.nijie(self.now_w)

    def PS4_fresh(self):
        self.PS4_param[0] = rospy.get_param("L3_up_down")
        self.PS4_param[1] = rospy.get_param("L3_left_right")
        self.PS4_param[2] = rospy.get_param("L3_press")
        self.PS4_param[3] = rospy.get_param("R3_up_down")
        self.PS4_param[4] = rospy.get_param("R3_left_right")
        self.PS4_param[5] = rospy.get_param("R3_press")
        self.PS4_param[6] = rospy.get_param("circle_press")
        self.PS4_param[7] = rospy.get_param("square_press")
        print("============")
        if int(self.PS4_param[6])==1:
            if self.PS4_mode==0:
                self.PS4_mode=1
        if int(self.PS4_param[7])==1:
            print(self.PS4_param[6],self.PS4_param[7],self.PS4_mode)
            if self.PS4_mode==1:
                self.PS4_mode=0
        print(self.PS4_param[6],self.PS4_param[7],self.PS4_mode)
        
    def callBack(self, 
                #  sub_imu, 
                 sub_Can_1, 
                 sub_Can_2, 
                 sub_Can_3, 
                 sub_Can_4,
                 ):
        # fresh imu msg
        # self.imu_fresh(sub_imu)
        # fresh encoder msg
        self.motor_fresh(sub_Can_1, sub_Can_2, sub_Can_3, sub_Can_4)
        self.PS4_fresh()
        # print(self.PS4_param)
        self.output()
        pass

    def init_can(self):
        print("Start to init CAN port")
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan')
        print("CAN port has init")
        # self.can_date=[0xA1,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        # self.msg =can.Message(arbitration_id=0x142,data=self.can_date,is_extended_id=False)
        # self.can0.send(self.msg)
        # self.msg =can.Message(arbitration_id=0x141,data=self.can_date,is_extended_id=False)
        # self.can0.send(self.msg)



if __name__ == '__main__':
    rospy.init_node("PID_control", anonymous=True)
    a = my_pid()
    a.init_can()
    # sub_imu = message_filters.Subscriber('Imu_pub', Imu)
    sub_Can_1 = message_filters.Subscriber('Can_moto1', Can)
    sub_Can_2 = message_filters.Subscriber('Can_moto2', Can)
    sub_Can_3 = message_filters.Subscriber('Can_moto3', Can)
    sub_Can_4 = message_filters.Subscriber('Can_moto4', Can)
    sync = message_filters.ApproximateTimeSynchronizer([
        # sub_imu, 
         sub_Can_1, 
         sub_Can_2, 
         sub_Can_3, 
         sub_Can_4,
         ], 10, 10, allow_headerless=False)
    sync.registerCallback(a.callBack)
    rospy.spin()
