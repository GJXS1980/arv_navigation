#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import json
import threading
import requests
import os
import sys

import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String, Float32MultiArray, Int32, Int64
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt


class NavTest():
    def __init__(self):
        rospy.init_node('arv_test', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        self.rest_time = rospy.get_param("~rest_time", 10)

        # 导航目标状态
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                            'SUCCEEDED', 'ABORTED', 'REJECTED',
                            'PREEMPTING', 'RECALLING', 'RECALLED',
                            'LOST']
        # 目标点文件路径
        self.Target_PATH = "./target_data.txt"
        # 全部位置文件路径
        self.All_PATH = "./center_data.txt"

        self.target_point = []
        self.all_point = []
        self.target_id = []

        try:
            if os.path.exists(self.Target_PATH):
                sz = os.path.getsize(self.Target_PATH)
                if not sz:
                    rospy.logerr("target_data.txt is empty!")
                    os._exit(0)
            else:
                os.mknod(self.Target_PATH)
                rospy.logerr("target_data.txt is not exists!")
                os._exit(0)

            target_file = open(self.Target_PATH)
            for line in target_file.readlines():
                curLine = line.strip().split(" ")
                self.target_point.append(list(map(float, curLine)))
            target_file.close()
            rospy.loginfo("get target point success")
        except IOError:
            rospy.logerr("Target_PATH File is not accessible")
            os._exit(0)
        # print("target point: ", self.target_point)

        try:
            if os.path.exists(self.All_PATH):
                sz = os.path.getsize(self.All_PATH)
                if not sz:
                    rospy.logerr("target_data.txt is empty!")
                    os._exit(0)
            else:
                os.mknod(self.All_PATH)
                rospy.logerr("target_data.txt is not exists!")
                os._exit(0)

            center_file = open(self.All_PATH)
            for line in center_file.readlines():
                curLine = line.strip().split(" ")
                self.all_point.append(list(map(float, curLine)))
            center_file.close()
            rospy.loginfo("get all point success")
        except IOError:
            rospy.logerr("All_PATH File is not accessible")
            os._exit(0)
        # print("all point: ", self.all_point)

        for i in range(len(self.all_point)):
            for j in range(len(self.target_point)):
                if self.all_point[i] == self.target_point[j]:
                    self.target_id.append(i)

        # print(self.target_id)

        # AGV 控制状态 0: 语音控制　1: 全自动
        self.control_type = rospy.get_param("~control_type", 1)

        # move_base服务
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        # 等待服务
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")

        # cmd_vel发布
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.check_dqg_switch = rospy.Publisher('/light_device_check', Int32, queue_size=5)
        self.check_safe_switch = rospy.Publisher('/safe_device_check', Int32, queue_size=5)
        rospy.Subscriber("DQG_State", Int32, self.get_dqg_state)
 #       rospy.Subscriber("safe_check_state", Int32, self.get_safe_check)
   
 #   def get_safe_check(self, data):
 #       if data.data == 1:
 #           pass

    def get_dqg_state(self, data):
        if data.data == 1:
            pass


    def get_center_point(self, target_id):
        target_res = self.target_id[target_id-1]
        center_point = []
        if (target_id -1) == 0 and target_res > 0:
            rospy.loginfo("1号位存在过渡点")
            center_point = self.all_point[0:target_res]
        elif target_res:
            if (target_res - self.target_id[target_id-2] > 1):
                rospy.loginfo("存在过渡点")
                center_point = self.all_point[self.target_id[target_id-2]+1:target_res]
            else:
                rospy.loginfo("不存在过渡点")
        else:
            rospy.loginfo("不存在过渡点")
        rospy.loginfo("过渡点: " + str(center_point))
        return center_point

    # 语音控制
    def voice_control_move(self, data):
        if data.data == 1:
            self.arv_move_target(1)

        if data.data == 2:
            self.arv_move_target(2)

        if data.data == 3:
            self.arv_move_target(3)

    # 监听初始化
    def start_nav_mission(self):
        rospy.Subscriber("ready_to_go", Int32, self.arv_start)
        print("等待开始指令")
        rospy.wait_for_message("ready_to_go", Int32)
        if self.control_type:
            self.arv_move_target(1)
            print("Start Mission")


    def arv_move_target(self, target_id):
        target_center_point = self.get_center_point(target_id)
        if len(target_center_point):
            rospy.loginfo("存在过渡点,优先移动到过渡点")
            for i in range(len(target_center_point)):
                self.center_goal = MoveBaseGoal()
                self.center_goal.target_pose.pose.position.x = target_center_point[i][0]
                self.center_goal.target_pose.pose.position.y = target_center_point[i][1]
                self.center_goal.target_pose.pose.position.z = target_center_point[i][2]
                self.center_goal.target_pose.pose.orientation.x = target_center_point[i][3]
                self.center_goal.target_pose.pose.orientation.y = target_center_point[i][4]
                self.center_goal.target_pose.pose.orientation.z = target_center_point[i][5]
                self.center_goal.target_pose.pose.orientation.w = target_center_point[i][6]
                self.center_goal.target_pose.header.frame_id = 'map'
                self.center_goal.target_pose.header.stamp = rospy.Time.now()

                rospy.loginfo("Going To Center Goal: " + str(target_center_point[i]))
                # 启动导航
                self.move_base.send_goal(self.center_goal)
                # 导航等待5分钟
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(30))
                # 获取导航状态
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving center goal")
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Center Goal Move Succeese")

            rospy.loginfo("Center Goal Move Finish, Start Move To Target Goal")
        else:
            rospy.loginfo("不存在过渡点,移动到目标点")

        # 设置导航点
        self.goal = MoveBaseGoal()
        self.goal.target_pose.pose.position.x = self.target_point[target_id-1][0]
        self.goal.target_pose.pose.position.y = self.target_point[target_id-1][1]
        self.goal.target_pose.pose.position.z = self.target_point[target_id-1][2]
        self.goal.target_pose.pose.orientation.x = self.target_point[target_id-1][3]
        self.goal.target_pose.pose.orientation.y = self.target_point[target_id-1][4]
        self.goal.target_pose.pose.orientation.z = self.target_point[target_id-1][5]
        self.goal.target_pose.pose.orientation.w = self.target_point[target_id-1][6]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()

        # 打印相关信息
        rospy.loginfo("Going To Target Goal: " + str(self.target_point[target_id-1]))

        # 启动导航
        if self.control_type and target_id == 2:
            self.check_safe_switch.publish(1)
            safe_data = 1
            while safe_data == 1 :
                try:
                    value=rospy.wait_for_message("/safe_check_state",Int32,timeout=60)  #40S
                    safe_data=value.data
                except:
                    safe_data=0
                    pass
            self.check_safe_switch.publish(0)  #关闭安全设备检测
        self.move_base.send_goal(self.goal)

        # 导航等待5分钟
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

        # 获取导航状态
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
            else:
                rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
           
        if self.control_type and target_id == 3:
            self.check_dqg_switch.publish(1)    #电气柜检测打开
            #self.check_safe_switch.publish(1)   #安全设备检测打开 

            rospy.wait_for_message("DQG_State",Int32)


        if self.control_type and target_id+1 == len(self.target_point)+1:
            target_id = 0
            self.arv_move_target(target_id+1)
            rospy.loginfo("巡检任务完成" + str(target_id+1))

        elif target_id+1 < len(self.target_point)+1 :
            self.arv_move_target(target_id+1)
                #self.check_dqg_switch.publish(0)
       

    def arv_start(self, data):
        self.start_mission_tag = data.data

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        test = NavTest()
        test.start_nav_mission()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
