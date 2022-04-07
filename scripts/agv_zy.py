#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib
import time

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.nav_id_result, self.odom_x, self.odom_y =  None, None, None
        self.i, self.cm = 0, 0.0
        rospy.init_node('send_goals_node', anonymous=False)
        #rospy.on_shutdown(self.shutdown)

        # 机器人进行起始点位置的校准
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.sub = rospy.Subscriber('/agv_nav_point', Int64 , self.nav_id) 
        rospy.Subscriber('/Odom', Odometry, self.agv_odom)

        # while self.cm <= 7000:
        #     move_cmd = Twist()
        #     if self.cm < 7000:
        #         move_cmd.angular.z = 0.5
        #         self.cmd_vel.publish(move_cmd) 
        #         self.cm += 0.01
        #         #time.sleep(50)
        #     else:
        #         move_cmd.angular.z = 0.0
        #         self.cmd_vel.publish(move_cmd) 
        #         self.cm += 0.01


        #   订阅陀螺仪数据作为一直在跑的一个线程
        rospy.Subscriber('/IMU', Imu, self.flag_data)

        #   订阅语音输入的命令词
        # rospy.Subscriber('nav_position', Int64, self.getGoalPoint)

        rospy.spin()

    # #   获取语音输入的命令词
    # def getGoalPoint(self,data):
    #     self.point = data.data

    def agv_odom(self, pose):
        self.odom_x = pose.pose.pose.position.x
        self.odom_y = pose.pose.pose.position.y
        #print(self.nav_id_result)
        #self.flag_data()



    def nav_id(self, data):
        self.nav_id_result = data.data
        #print(self.nav_id_result)
        #self.flag_data()

    # 导航到目标点
    def flag_data(self, data):
        #print(self.odom_x, self.odom_y)
        if self.nav_id_result == 0:
                self.and_goal(self.i)
                if self.i == 3:
                    self.nav_id_result = -1
                    self.i = 0
                    self.odom_and = self.odom_y

        elif self.nav_id_result == 1:
                self.and_back_goal(self.i)
                if self.i == 1:
                    self.nav_id_result = 2
                    self.i = 0

        elif self.nav_id_result == 2:
                self.md_goal(self.i)
                if self.i == 2:
                    self.nav_id_result = -1
                    self.i = 0
                    self.odom_md = self.odom_y

        elif self.nav_id_result == 3:
                self.md_back_goal(self.i)
                if self.i == 1:
                    self.nav_id_result = 4
                    self.i = 0

        elif self.nav_id_result == 4:
                self.goHome_goal(self.i)
                if self.i == 2:
                    self.nav_id_result = -1
                    self.i = 0

        else:
                self.i = 0

    #   去岸吊导航函数
    def and_goal(self, i): 
        waypointsx = list([-0.613398373127, -2.47751951218, -2.34309148788])
        waypointsy = list([-0.947887957096, -0.793527126312, -2.3934264183])

        waypointsaw = list([0.999999261674, -0.683739089016, -0.718617713365])
        waypointsw = list([0.00121517548723, 0.729726563962, 0.695405336504])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    #   后退导航函数
    def and_back_goal(self, i): 
        move_cmd = Twist()
        if ((self.odom_and - self.odom_y) < 1.4 and (self.odom_and - self.odom_y) > 0) or ((self.odom_and - self.odom_y) > -1.4 and (self.odom_and - self.odom_y) < 0):
            move_cmd.linear.x = -0.2
            #self.cmd_vel.publish(move_cmd) 
        else:
            move_cmd.linear.x = 0.0
            self.i = 1
        self.cmd_vel.publish(move_cmd) 
        

    #   门吊导航函数
    def md_goal(self, i): 
        waypointsx = list([-2.17645215988, -2.13766241074])
        waypointsy = list([-0.939172685146, 0.502264797688])

        waypointsaw = list([0.695437277779, 0.706948011643])
        waypointsw = list([0.718586802463, 0.707265515089])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    #   后退导航函数
    def md_back_goal(self, i): 
        move_cmd = Twist()
        if ((((self.odom_y - self.odom_md) < 1.3) and ((self.odom_y - self.odom_md) > 0))  or (((self.odom_y - self.odom_md) > -1.3) and ((self.odom_y - self.odom_md) < 0))):
            move_cmd.linear.x = -0.2
            #self.cmd_vel.publish(move_cmd) 
        else:
            move_cmd.linear.x = 0.0
            self.i = 1
        self.cmd_vel.publish(move_cmd) 
        

    #   起始点导航函数
    def goHome_goal(self, i): 
        waypointsx = list([-1.63051044941, -0.583370566368])
        waypointsy = list([-0.872207641602, 0.751771330833])

        waypointsaw = list([0.00288962799317, 0.00189589304428])
        waypointsw = list([0.999995825016, 0.999998202793])

        # 订阅move_base服务器的消息
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        self.ac.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server");

        # 初始化goal为MoveBaseGoal类型
        goal = MoveBaseGoal()

        # 使用map的frame定义goal的frame id
        goal.target_pose.header.frame_id = 'map'

        # 设置时间戳
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标位置
        goal.target_pose.pose.position.x = waypointsx[i]
        goal.target_pose.pose.position.y = waypointsy[i]
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = waypointsaw[i]
        goal.target_pose.pose.orientation.w = waypointsw[i]
        rospy.loginfo("Sending goal")
        # 机器人移动
        self.move(goal)

    def move(self, goal):
        self.ac.send_goal(goal)

        # 设定5分钟的时间限制
        finished_within_time = self.ac.wait_for_result(rospy.Duration(300))

        # 如果5分钟之内没有到达，放弃目标
        if not finished_within_time:
            self.ac.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            state = self.ac.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.i += 1
                #   发布导航成功的flag
                rospy.loginfo("You have reached the goal!")


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Cancel any active goals
        self.ac.cancel_goal()
        #rospy.sleep(16)

        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)

if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
