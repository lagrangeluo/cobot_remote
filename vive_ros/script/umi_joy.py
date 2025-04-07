#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from std_msgs.msg import String
from survive_publisher.msg import umijoy
# from arm_control.msg import PosCmd

class joystick_node:
    def __init__(self):
        # uri and topic name
        self.topic_name_l = rospy.get_param('/vive/esp32_topic_name_l', '/esp32_json_string_left')
        self.topic_name_r = rospy.get_param('/vive/esp32_topic_name_r', '/esp32_json_string_right')
        self.pub_name_l = rospy.get_param('/vive/joystick_topic_l', '/joystick_l')
        self.pub_name_r = rospy.get_param('/vive/joystick_topic_r', '/joystick_r')
        
        # param
        self.button_press_time = rospy.get_param('/vive/button_press_time',20)
        self.button_single_press = rospy.get_param('/vive/button_single_press',4)

        # self.topic_pub = rospy.Publisher(self.topic_name, String, queue_size=10)
        self.esp32_sub_l = rospy.Subscriber(self.topic_name_l, String, self.esp32_json_callback_l)
        self.esp32_sub_r = rospy.Subscriber(self.topic_name_r, String, self.esp32_json_callback_r)
        # self.esp32_pub_l = rospy.Publisher("esp32_master_fb_left", String,queue_size=5)
        # self.esp32_pub_r = rospy.Publisher("esp32_master_fb_right", String,queue_size=5)
        self.js_pub_l = rospy.Publisher(self.pub_name_l,umijoy,queue_size=5)
        self.js_pub_r = rospy.Publisher(self.pub_name_r,umijoy,queue_size=5)

        # button dic and flag
        self.button_dic = {'left':0,'right':0}
        self.press_dic = {'left':False,'right':False}
        self.press_cnt = {'left':0,'right':0}
        # # 连续长按两个按键
        # self.left_right_cnt = 0
        # self.up_down_press = False

        # # 设置定时器，每0.5秒（即2Hz）执行一次回调函数
        # rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    # def timer_callback(self,event):
    #     # 在定时器回调中发布消息
    #     msg = String()
    #     msg.data = "This is a message from ros master!"

    #     self.esp32_pub_l.publish(msg)
    #     self.esp32_pub_r.publish(msg)

    def apply_deadband(self, increase):
        if abs(increase) > self.deadband:
            return increase - self.deadband if increase > 0 else increase + self.deadband
        return 0
    
    def resolve_json(self,message,hand_name):
        # 解析JSON数据
            try:
                data = json.loads(message)

                # 按键数据
                self.button_dic['left'] = data.get("left_button", 0)
                self.button_dic['right'] = data.get("right_button", 0)

                # 基础按键赋值
                joystick_msg = umijoy()
                joystick_msg.left_button = self.button_dic['left']
                joystick_msg.right_button = self.button_dic['right']

                # 长按按键部分
                for key in self.press_dic:
                    button = self.button_dic[key]
                    press = self.press_dic[key]

                    if button == 0:
                        self.press_cnt[key] = 0
                    if button == 1 and self.press_cnt[key] < self.button_single_press:
                        self.press_cnt[key] += 1
                    if self.press_cnt[key] == self.button_single_press:
                        self.press_dic[key] = not self.press_dic[key]
                        self.press_cnt[key] += 1                
                
                joystick_msg.press_left = self.press_dic['left']
                joystick_msg.press_right = self.press_dic['right']
 
                if hand_name=="right":
                    self.js_pub_r.publish(joystick_msg)
                elif hand_name=="left":
                    self.js_pub_l.publish(joystick_msg)

            except json.JSONDecodeError as e:
                rospy.logerr(f"JSON decode error: {e}")
                
    def esp32_json_callback_l(self,msg):
        self.resolve_json(msg.data,"left")
    def esp32_json_callback_r(self,msg):
        self.resolve_json(msg.data,"right")

def main():
    rospy.init_node('websocket_to_ros_node')
    node = joystick_node()
    rospy.loginfo("---------joystick init success------")
    rospy.spin()

if __name__ == '__main__':
    main()