#!/usr/bin/python
# -*- coding:utf8 -*-


import paho.mqtt.client as mqtt
from collections import OrderedDict
import json
import random
import time
import threading

MQTTHOST = "192.168.3.6"
MQTTPORT = 50000
mqttClient = mqtt.Client()


# 连接MQTT服务器
def on_mqtt_connect():
    mqttClient.connect(MQTTHOST, MQTTPORT, 60)
    mqttClient.loop_start()


# publish 消息
def on_publish(topic, payload, qos):
    print("pub msg: %s to %s" % (payload, topic))
    mqttClient.publish(topic, payload, qos)


# 消息处理函数
def on_message_come(lient, userdata, msg):
    sub_msg = str(msg.payload, 'utf-8')
	# python3 str转字典
    #print(sub_msg)
    print("from: "+ msg.topic + " " + ":" + sub_msg)
    #sub_msg = json.loads(msg.payload.encode('utf-8'))
    #print(type(sub_msg), sub_msg["dir"])

# subscribe 消息
def on_subscribe():
    mqttClient.subscribe("/HG_CAR/CAR_MSG", 2)
    mqttClient.on_message = on_message_come  # 消息到来处理函数


def get_data():
    x = 200.00
    y = 200.0
    z = 0.00
    json_dict = OrderedDict()
    '''
    json_dict["name"] = "TD"
    json_dict["dir"] = "ZK"
    json_dict["cmd_type"] = 0
    json_dict["pos"] = [x, y, z]
    json_dict["Grasp_Status"] = 0
    json_dict["EmStop_Status"] = 0
    json_dict["Stop_Status"] = 0
    json_dict["Reset_Status"] = 0
    json_dict["Reset_x"] = 0
    json_dict["Reset_y"] = 0
    json_dict["error"] = "null"
    '''
    json_dict["name"] = "AGV"
    json_dict["dir"] = "AD"
    json_dict["ation"] = "loading"
    json_dict["error"] = "null"
    json_dict["feedback"] = 0

    param = json.dumps(json_dict)
    return param


def main():
    # createClient()
    on_mqtt_connect()
    on_subscribe()
    
    while True:
        #on_publish("/HG_CAR/CAR_MSG", get_data(), 2)
        on_subscribe()
        time.sleep(0.5)

if __name__ == '__main__':
    main()

