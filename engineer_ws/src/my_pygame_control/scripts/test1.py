from pickle import TRUE
import pygame
import sys
import threading
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Quaternion
import math
import moveit_commander
import time
import copy

class clock:
    def __init__(self):
        self.now_time = time.time()

    def set_now_time(self):
        self.now_time = time.time()

    def cal_dist(self):
        return time.time()-self.now_time


class A1_serial_sub:
    def __init__(self):
        self.lin_press = {"q": 0, "w": 0, "e": 0, "a": 0, "s": 0, "d": 0,
                          "u": 0, "i": 0, "o": 0, "j": 0, "k": 0, "l": 0,
                          "ctrl": 0, "shift": 0}
        self.lin_name = ["q", "w", "e", "a", "s", "d",
                         "u", "i", "o", "j", "k", "l",
                         "ctrl", "shift"]
        size = width, height = 600, 400
        screen = pygame.display.set_mode(size)
        pygame.init()
        rospy.init_node("A1_serial")
        # self.sub = rospy.Subscriber("A1_recv_data",A1_recv,self.callback,queue_size=10)
        # self.pub = rospy.Publisher("A1_send_data",A1_send,queue_size=1)
        self.target_pose = PoseStamped()
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
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.my_clock = clock()
        self.NUM = 0.001
        self.fresh_now_pose()
        self.target_pose.pose = copy.deepcopy(self.now_pose)

    def mainloop(self):
        rate = rospy.Rate(100)
        while True:
            self.my_clock.set_now_time()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    sys.exit()
                # print(event)
                if event.type == pygame.KEYDOWN:
                    if event.key == 113:
                        self.lin_press[self.lin_name[0]] = 1
                    if event.key == 119:
                        self.lin_press[self.lin_name[1]] = 1
                    if event.key == 101:
                        self.lin_press[self.lin_name[2]] = 1
                    if event.key == 97:
                        self.lin_press[self.lin_name[3]] = 1
                    if event.key == 115:
                        self.lin_press[self.lin_name[4]] = 1
                    if event.key == 100:
                        self.lin_press[self.lin_name[5]] = 1
                    if event.key == 117:
                        self.lin_press[self.lin_name[6]] = 1
                    if event.key == 105:
                        self.lin_press[self.lin_name[7]] = 1
                    if event.key == 111:
                        self.lin_press[self.lin_name[8]] = 1
                    if event.key == 106:
                        self.lin_press[self.lin_name[9]] = 1
                    if event.key == 107:
                        self.lin_press[self.lin_name[10]] = 1
                    if event.key == 108:
                        self.lin_press[self.lin_name[11]] = 1
                if event.type == pygame.KEYUP:
                    if event.key == 113:
                        self.lin_press[self.lin_name[0]] = 0
                    if event.key == 119:
                        self.lin_press[self.lin_name[1]] = 0
                    if event.key == 101:
                        self.lin_press[self.lin_name[2]] = 0
                    if event.key == 97:
                        self.lin_press[self.lin_name[3]] = 0
                    if event.key == 115:
                        self.lin_press[self.lin_name[4]] = 0
                    if event.key == 100:
                        self.lin_press[self.lin_name[5]] = 0
                    if event.key == 117:
                        self.lin_press[self.lin_name[6]] = 0
                    if event.key == 105:
                        self.lin_press[self.lin_name[7]] = 0
                    if event.key == 111:
                        self.lin_press[self.lin_name[8]] = 0
                    if event.key == 106:
                        self.lin_press[self.lin_name[9]] = 0
                    if event.key == 107:
                        self.lin_press[self.lin_name[10]] = 0
                    if event.key == 108:
                        self.lin_press[self.lin_name[11]] = 0
                    if event.key == 1073742052:
                        self.lin_press[self.lin_name[12]] = 1
                    if event.key == 1073742053:
                        self.lin_press[self.lin_name[13]] = 1

            # print(self.my_clock.cal_dist())
            self.target_pose.header.frame_id = self.reference_frame
            self.target_pose.header.stamp = rospy.Time.now()  # 时间戳
            # 末端位置
            if self.lin_press[self.lin_name[0]] == 1:
                self.target_pose.pose.position.x += self.NUM
                # print("x+",self.target_pose.pose.position.x)
            if self.lin_press[self.lin_name[3]] == 1:
                self.target_pose.pose.position.x -= self.NUM
                # print("x-",self.target_pose.pose.position.x)
            if self.lin_press[self.lin_name[1]] == 1:
                self.target_pose.pose.position.y += self.NUM
            if self.lin_press[self.lin_name[4]] == 1:
                self.target_pose.pose.position.y -= self.NUM
            if self.lin_press[self.lin_name[2]] == 1:
                self.target_pose.pose.position.z += self.NUM
            if self.lin_press[self.lin_name[5]] == 1:
                self.target_pose.pose.position.z -= self.NUM
            # 末端姿态，四元数
            if self.lin_press[self.lin_name[12]] == 1:
                self.fresh_now_pose()
            self.target_pose.pose.orientation = self.now_pose.orientation
            self.arm.set_start_state_to_current_state()
            self.arm.set_pose_target(self.target_pose, self.end_effector_link)
            print("==========")
            # print("%.5f"%(self.target_pose.pose.position.x- self.now_pose.position.x))
            # print("%.5f"%(self.target_pose.pose.position.y- self.now_pose.position.y))
            # print("%.5f"%(self.target_pose.pose.position.z- self.now_pose.position.z))
            self.traj=self.arm.plan()
            rate.sleep()

    def fresh_now_pose(self):
        print("now_pose fresh")
        self.now_pose = self.arm.get_current_pose(self.end_effector_link).pose
        print(self.now_pose)


    def make_plan(self):  # shift
        while True:
            if self.lin_press[self.lin_name[13]] == 1:
                self.traj=self.arm.plan()
                print("shift press")
                self.lin_press[self.lin_name[13]] = 0

    def make_excute(self):  # ctrl
        while True:
            if self.lin_press[self.lin_name[12]] == 1:
                if self.lin_press[self.lin_name[13]] == 0:
                    # self.arm.execute(self.traj)
                    print("ctrl press")
                    self.lin_press[self.lin_name[12]] = 0

    def EulerAndQuaternionTransform(self, intput_data):
        data_len = len(intput_data)
        angle_is_not_rad = False

        if data_len == 3:
            r = 0
            p = 0
            y = 0
            if angle_is_not_rad:  # 180 ->pi
                r = math.radians(intput_data[0])
                p = math.radians(intput_data[1])
                y = math.radians(intput_data[2])
            else:
                r = intput_data[0]
                p = intput_data[1]
                y = intput_data[2]

            sinp = math.sin(p/2)
            siny = math.sin(y/2)
            sinr = math.sin(r/2)

            cosp = math.cos(p/2)
            cosy = math.cos(y/2)
            cosr = math.cos(r/2)

            w = cosr*cosp*cosy + sinr*sinp*siny
            x = sinr*cosp*cosy - cosr*sinp*siny
            y = cosr*sinp*cosy + sinr*cosp*siny
            z = cosr*cosp*siny - sinr*sinp*cosy
            return [w, x, y, z]

        elif data_len == 4:

            w = intput_data[0]
            x = intput_data[1]
            y = intput_data[2]
            z = intput_data[3]

            r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
            p = math.asin(2 * (w * y - z * x))
            y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

            if angle_is_not_rad:  # pi -> 180
                r = math.degrees(r)
                p = math.degrees(p)
                y = math.degrees(y)
            return [r, p, y]


if __name__ == "__main__":
    a = A1_serial_sub()
    make_plan_process = threading.Thread(target=a.make_plan)
    make_excute_process = threading.Thread(target=a.make_excute)
    mainloop_process = threading.Thread(target=a.mainloop)
    make_plan_process.start()
    mainloop_process.start()
    make_excute_process.start()
    while True:
        a = 1
