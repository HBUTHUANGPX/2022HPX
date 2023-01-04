#!/usr/bin/env python
# coding: utf-8
import struct
import os
import threading
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Quaternion
import moveit_commander
import time
import copy
import sys
import math
import transforms3d as tfs
import numpy as np 
# 固定轴欧拉角转四元数
# tfs.euler.euler2quat(0,0,0,"sxyz")
# 四元数转固定轴欧拉角
# tfs.euler.quat2euler([1,0,0,0],"sxyz")

class clock:
    def __init__(self):
        self.now_time = time.time()

    def set_now_time(self):
        self.now_time = time.time()

    def cal_dist(self):
        return time.time()-self.now_time


class my_keyboard:
    def __init__(self):
        self.key = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.key_dict = {"16": 0, "17": 0, "18": 0,
                         "30": 0, "31": 0, "32": 0,
                         "22": 0, "23": 0, "24": 0,
                         "36": 0, "37": 0, "38": 0,
                         "54": 0, "97": 0, "100": 0}
        rospy.init_node("A1_serial")
        # self.sub = rospy.Subscriber("A1_recv_data",A1_recv,self.callback,queue_size=10)
        # self.pub = rospy.Publisher("A1_send_data",A1_send,queue_size=1)
        self.target_pose = Pose()
        self.now_pose = Pose()
        self.euler = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 0.0]
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.0001)
        self.arm.set_goal_orientation_tolerance(0.0001)
        self.my_clock = clock()
        self.NUM = 0.001
        self.fresh_now_pose()
        self.target_pose = copy.deepcopy(self.now_pose)
        [self.roll, self.pitch, self.yaw] = tfs.euler.quat2euler([self.now_pose.orientation.w,
                                                                  self.now_pose.orientation.x,
                                                                  self.now_pose.orientation.y,
                                                                  self.now_pose.orientation.z],"rxyz")
        self.go=0

    def main_loop(self):
        rate = rospy.Rate(100)
        with open("/dev/input/event5", "rb") as f:
            while True:
                data = f.read(24)

                # 末端位置
                if self.key_dict["16"] == 1:
                    self.target_pose.position.x += self.NUM
                    # print("x+", self.target_pose.position.x)
                if self.key_dict["30"] == 1:
                    self.target_pose.position.x -= self.NUM
                    # print("x-", self.target_pose.position.x)
                if self.key_dict["17"] == 1:
                    self.target_pose.position.y += self.NUM
                if self.key_dict["31"] == 1:
                    self.target_pose.position.y -= self.NUM
                if self.key_dict["18"] == 1:
                    self.target_pose.position.z += self.NUM
                if self.key_dict["32"] == 1:
                    self.target_pose.position.z -= self.NUM

                if self.key_dict["22"] == 1:
                    self.roll += self.NUM*10
                    # print("x+", self.target_pose.position.x)
                if self.key_dict["36"] == 1:
                    self.roll -= self.NUM*10
                    # print("x-", self.target_pose.position.x)
                if self.key_dict["23"] == 1:
                    self.pitch += self.NUM*10
                if self.key_dict["37"] == 1:
                    self.pitch -= self.NUM*10
                if self.key_dict["24"] == 1:
                    self.yaw += self.NUM*10
                if self.key_dict["38"] == 1:
                    self.yaw -= self.NUM*10
                self.orien = tfs.euler.euler2quat(self.roll,self.pitch,self.yaw,"rxyz")
                # print("target_pose orien",self.orien)
                # print("now_pose    orien\n",self.now_pose.orientation)
                # self.target_pose.orientation=self.now_pose.orientation
                self.target_pose.orientation.w = self.orien[0]
                self.target_pose.orientation.x = self.orien[1]
                self.target_pose.orientation.y = self.orien[2]
                self.target_pose.orientation.z = self.orien[3]
                self.arm.set_start_state_to_current_state()
                self.arm.set_pose_target(
                    self.target_pose, self.end_effector_link)
                # print("==========")
                # print("%.5f" % (self.target_pose.position.x -
                #       self.now_pose.position.x))
                # print("%.5f" % (self.target_pose.position.y -
                #       self.now_pose.position.y))
                # print("%.5f" % (self.target_pose.position.z -
                #       self.now_pose.position.z))
                # print("%.4f\t%.4f\t%.4f" % (self.roll, self.pitch, self.yaw))
                time_sec, time_usec, type_, code, value = struct.unpack(
                    "QQHHi", data)
                if type_ == 0:
                    continue
                if type_ == 1:
                    if value == 2:
                        continue
                
                    self.key_dict[str(code)] = value
                # print("{}.{}\ttype={}\tcode={}\tvalue={}".format(
                #     time_sec, time_usec, type_, code, value))
                if type_==1:
                    self.go=1
                    # print(self.go)
                rate.sleep()

    def fresh_now_pose(self):
        self.now_pose = self.arm.get_current_pose(self.end_effector_link).pose

    def fresh_now_eular(self):
        [self.roll, self.pitch, self.yaw] = tfs.euler.quat2euler([self.now_pose.orientation.w,
                                                                  self.now_pose.orientation.x,
                                                                  self.now_pose.orientation.y,
                                                                  self.now_pose.orientation.z],"rxyz")

    def make_plan(self):  # shift
        while True:
            self.traj = self.arm.plan()

    def make_excute(self):  # ctrl
        # rate = rospy.Rate(10)
        while True:
            # print("now_pose eular",tfs.euler.quat2euler([self.now_pose.orientation.w,self.now_pose.orientation.x,self.now_pose.orientation.y,self.now_pose.orientation.z],"rxyz"))
            # self.fresh_now_pose()
            if self.go==1:
                self.arm.set_pose_target(self.target_pose)
                self.arm.go()
                self.fresh_now_pose()
                self.go=0
                # self.fresh_now_eular()
                # rate.sleep

    def orien_2_eular(self, x, y, z, w):
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(2*(w*y-z*z))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))

        # angleR = roll*180/math.pi
        # angleP = pitch*180/math.pi
        # angleY = yaw*180/math.pi
        return [roll*180/math.pi, pitch*180/math.pi, yaw*180/math.pi]

    def eular_2_orien(self, roll, pitch, yaw):
        cy = math.cos(yaw*0.5)
        sy = math.sin(yaw*0.5)
        cp = math.cos(pitch*0.5)
        sp = math.sin(pitch*0.5)
        cr = math.cos(roll*0.5)
        sr = math.sin(roll*0.5)
        return [cy*cr*cp+sy*sr*sp,
                cy*sr*cp-sy*cr*sp,
                cy*cr*sp+sy*sr*cp,
                sy*cr*cp-cy*sr*sp]


if __name__ == "__main__":
    a = my_keyboard()
    make_excute_process = threading.Thread(target=a.make_excute)
    mainloop_process = threading.Thread(target=a.main_loop)
    mainloop_process.start()
    make_excute_process.start()
    while True:
        b = 1
