#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
#roslib.load_manifest('simple_navigation_goals_tutorial')
import rospy
import actionlib

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray, Int64
from nav_msgs.msg import Odometry

'''
##########################  主函数  ############################

'''
class MoveBaseDoor():
    def __init__(self):
        self.i, self.cm = 0, 0.0
        rospy.init_node('send_goals_node', anonymous=False)
        #rospy.on_shutdown(self.shutdown)

        # 机器人进行起始点位置的校准
        # self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

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
        rospy.Subscriber('/odom', Odometry, self.flag_data)

        #   订阅语音输入的命令词
        # rospy.Subscriber('nav_position', Int64, self.getGoalPoint)

        rospy.spin()

    # #   获取语音输入的命令词
    # def getGoalPoint(self,data):
    #     self.point = data.data


    # 导航到目标点
    def flag_data(self, data):
        if self.i == 4:
        	self.i = 0
        	self.goal(self.i)
        else:
        	self.goal(self.i)

    #   导航函数
    def goal(self, i):
        # 目标点的x,y,w坐标(0:客厅,1:卧室,2:厨房,3:阳台)
 
        waypointsx = list([0.4689412117,  -0.858030796051, -0.612303733826, 0.798955440521])
        waypointsy = list([-0.0319719314575, -0.333612918854, 0.648940086365, 1.21361255646])

        waypointsaw = list([0.971717351556, 0.85112198936,  0.252455919928,-0.581534135655])
        waypointsw = list([-0.236146964168, 0.524967960192,0.96760839625,  0.813522002818])

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
        rospy.sleep(2)

        # Stop the robot
        #self.cmd_vel_pub.publish(Twist())
        #rospy.sleep(1)



if __name__ == '__main__':
    try:
        MoveBaseDoor()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
