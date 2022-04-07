# coding=utf-8
from ctypes import *
import math
import random
import socket
import time
import sys
import threading
import cv2
import io
import numpy as np
import rospy
import actionlib
from actionlib_msgs.msg import *
from std_msgs.msg import String, Float32MultiArray, Int32, Int64
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import pyrealsense2 as rs
# import darknet as dk

class ZPT_Grasp_OBJ():

    def __init__(self):
	rospy.init_node('arv_test', anonymous=True)
        # 通讯设置
        self.host_ip = '192.168.2.100'
        self.host_port = 10010

        self.Socket_Client = socket.socket() #(socket.AF_INET,socket.SOCK_STREAM)

        # 非阻塞
        # self.Socket_Client.setblocking(False)

        # Socket接受信息
        self.msg = None

        # 可装配数量
        self.ZP_num = 0

        # 螺丝数据
        self.LS_all_data = []  # 全部数据
        self.LS_data = []  # 正确螺丝的数据
        self.LS_angle = 0.0  # 旋转角度

        # 螺母数据
        self.LM_data = []
        self.LM_angle = 0.0

        # 钣金数据
        self.BJJ_data = []
        self.BJJ_angle = 0.0

        self.start_mission = False

        # try:
        #     socket_data = []
        #     with open("connect.txt", "r+") as f:
        #         for line in f.readlines():
        #             line = line.strip('\n')
        #             socket_data.append(line)
        #     self.host_ip = socket_data[0]
        #     self.host_port = int(socket_data[1])
        # except:
        #     print("not file path, use default data!!!")

        try:
            connect_type = self.Socket_Client.connect_ex((self.host_ip, self.host_port))
            if connect_type == 0:
                print("PLC服务器连接成功, ip: %s, port: %s" %(self.host_ip,self.host_port))
        except Exception as e:
            print("PLC服务器连接失败: ", e)
            sys.exit(1)

        rospy.Subscriber('agv_state', Int32,self.get_agv_state)
        self.pub_arm_state = rospy.Publisher("arm_state", Int32, queue_size=5)

    def get_agv_state(self, data):
        if data.data == 1:
            self.start_mission = True
	    self.send_data("WLSLSQ")
            self.start_mission = False
	    print("OK")
    # 回复OK
    def reply_ok(self):
        self.Socket_Client.sendall(bytes(('OK'+'\r').encode("utf-8")))
        return True

    # 发送数据
    def send_data(self, data):
        print("开始发送数据: %s" % data)
        if data != None:
            while True:
                self.Socket_Client.sendall(bytes(data.encode("utf-8")))
                time.sleep(1.0)
                msg = self.Socket_Client.recv(8)
                msg = msg.decode("utf-8")
                # print(msg)
                if msg[0:2] == 'OK':
                    print("发送数据成功!!")
                    self.msg = None
                    break
                else:
                    continue
        else:
            print("无效数据,拒绝发送!!")

    # 转换,补零
    def set_new_data(self, send_data):
        request_str = ""
        for i in range(len(send_data)):
            current_data = send_data[i]
            lenth = 4
            # 切割小数
            new_data = math.modf(current_data)
            # if str(new_data[1])[0] == '-':
            #     lenth += 1
            # print(len(str(new_data[0])), new_data[0])
            data_list = list(new_data)
            if len(str(new_data[0])) == 4:
                new_data = (data_list[0]+0.01, new_data[1])

            request_str += str(new_data[1])[0:len(str(new_data[1]))-2].zfill(lenth) + str(abs(new_data[0]))[1:4]
            if i < len(send_data) - 1:
                request_str +=","
        return request_str + '\r'

    # 接受请求
    def accept_request(self):
        print("开始接收请求")
        self.request_data = None
        all_request_data = [ "TXCS","WLXLWC", "WLSLWC"]

        # net = dk.load_net("./cfg/yolov3_bj.cfg", "./backup/BJBK/yolov3_final.weights", 0)
        # meta = dk.load_meta("cfg/voc_bj.data")

        # 设置为非阻塞
        # self.Socket_Client.setblocking(0)
        while True:
            ready_state = False
            try:
                self.msg = self.Socket_Client.recv(8)
                if self.msg:
                    #self.msg = self.msg.decode("utf-8")
                    #self.msg.strip()
                    if self.msg in all_request_data:
                        print("接收到请求： %s" % self.msg)
                        ready_state = self.reply_ok()
                    else:
                        print("获取到无效请求: %s" % self.msg)
            except io.BlockingIOError as e:
                self.msg = None

            if ready_state:
                # 测试通讯数据
                if self.msg == "TXCS":
                    self.Socket_Client.sendall(bytes(('OK').encode("utf-8")))
                    print("发送测试通讯OK") 
                elif self.msg == "WLXLWC":
                    self.Socket_Client.sendall(bytes(('OK').encode("utf-8")))
                    time.sleep(1.0)
                    self.pub_arm_state.publish(1)
                    print("发送上料完成")
                elif self.msg == "WLSLWC":
                    self.Socket_Client.sendall(bytes(('OK').encode("utf-8")))
                    time.sleep(1.0)
                    self.send_data("WLXLSQ")
                    # send_str = "XL_SYWC"
                    # self.send_data(send_str)
                    print("发送下料完成")
                else:
                    print("无效请求,拒绝返回数据!!")
                    pass

if __name__ == "__main__":
    test = ZPT_Grasp_OBJ()
    test.accept_request()
    #test.send_data("WLSLSQ")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    test.pipeline.stop()
