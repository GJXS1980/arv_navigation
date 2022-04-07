#!/usr/bin/env python
# -*- coding: UTF-8 -*-
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
        rospy.init_node('nav_test', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        self.fake_test = rospy.get_param("~fake_test", False)
        
        # 导航目标状态
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        
        # 导航点
        locations = dict()
        
        locations['LTCK'] = Pose(Point(-0.856396913528, -0.556189537048, 0.000), Quaternion(0.000, 0.000, 0.844838190767, 0.535021898076))
        locations['ZPT'] = Pose(Point(-0.672147035599, 0.505731105804, 0.000), Quaternion(0.000, 0.000, 0.234804135434, 0.972042703786))
        locations['WLT'] = Pose(Point(0.594016551971, -0.360795497894, 0.000), Quaternion(0.000, 0.000, 0.967273716562, -0.253735210898))
        
        # cmd_vel发布０
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.agv_state_pub = rospy.Publisher('agv_state', Int32, queue_size=1)
        # move_base服务
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # 等待服务
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        
        # 监听rviz初始化位置
        initial_pose = PoseWithCovarianceStamped()
        
        # 导航任务
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""
        
        # 获取初始化位置是否成功
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
        # 确保初始化成功
        while initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")
        
        # 开始循环导航点
        while not rospy.is_shutdown():
            # 随机序列
            if i == n_locations:
                i = 0
                sequence = list(locations.keys())
                # 如果第一个点和最后一个点重复,跳过第一个点
                if sequence[0] == last_location:
                    i = 1
		    print("------重复了")
            
	    # 获取当前序列的下一个点
            location = sequence[i]
                        
            # 跟踪行驶距离
            # 更新初始化位姿
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x - 
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y - 
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x - 
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y - 
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""
            
            # 更新上一个导航目标点
            last_location = location
            
            # 计数增加
            i += 1
            n_goals += 1
        
            # 设置下一个导航点
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # 打印相关信息
            rospy.loginfo("Going to: " + str(location))
            
            # 启动导航
            self.move_base.send_goal(self.goal)
            
	    # 等待时间五分钟
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # 获取导航状态
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    self.agv_state_pub.publish(Twist())
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            
            # 计算导航时间
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0
            
            # 打印相关信息，导航状态，运行时间，行驶距离
            rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                          str(n_goals) + " = " + 
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            rospy.sleep(self.rest_time)
            
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
      
def trunc(f, n):
    # 小数截断，不进位
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")

