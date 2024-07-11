#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import websockets
import json
import rospy
from std_msgs.msg import String
from survive_publisher.msg import joystick
# from arm_control.msg import PosCmd

class joystick_node:
    def __init__(self):
        # uri and topic name
        self.topic_name = rospy.get_param('topic_name', '/esp32_json_string')

        # param
        self.middle_value = rospy.get_param('middle_value', 2420)
        self.deadband = rospy.get_param('deadband',100)

        # self.topic_pub = rospy.Publisher(self.topic_name, String, queue_size=10)
        self.esp32_sub = rospy.Subscriber(self.topic_name, String, self.esp32_json_callback)
        self.js_pub = rospy.Publisher("/joystick",joystick,queue_size=5)


    def apply_deadband(self, increase):
        if abs(increase) > self.deadband:
            return increase - self.deadband if increase > 0 else increase + self.deadband
        return 0
    
    def resolve_json(self,message):
        # 解析JSON数据
            try:
                data = json.loads(message)
                # 摇杆数据
                x_axis = data.get("x", 2420)
                y_axis = data.get("y", 2420)

                x_increase = x_axis - self.middle_value
                y_increase = y_axis - self.middle_value

                x_increase = self.apply_deadband(x_increase)
                y_increase = self.apply_deadband(y_increase)

                # 按键数据
                js_button = data.get("js_button", 1)
                up_button = data.get("up_button", 0)
                down_button = data.get("down_button", 0)

                # self.topic_pub.publish(String(message))
                rospy.loginfo(f"message: {message} x_increase:{x_increase},y_increase:{y_increase}")
                rospy.loginfo("js_button: %d, up_button: %d, down_button: %d" %(js_button,up_button,down_button))

                joystick_msg = joystick()
                joystick_msg.x_increase = x_increase
                joystick_msg.y_increase = y_increase
                joystick_msg.js_button = not js_button
                joystick_msg.up_button = up_button
                joystick_msg.down_button = down_button
                self.js_pub.publish(joystick_msg)

            except json.JSONDecodeError as e:
                rospy.logerr(f"JSON decode error: {e}")
                
    def esp32_json_callback(self,msg):
        self.resolve_json(msg.data)

def main():
    rospy.init_node('websocket_to_ros_node')
    node = joystick_node()
    rospy.loginfo("---------joystick init success------")
    rospy.spin()

if __name__ == '__main__':
    main()
