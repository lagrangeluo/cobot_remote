#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import websockets
import json
import rospy
from std_msgs.msg import String

class joystick:
    def __init__(self) -> None:
        # uri and topic name
        # self.websocket_uri = rospy.get_param('websocket_uri', 'ws://10.10.96.110:81/aloha')
        self.websocket_uri = rospy.get_param('websocket_uri', 'ws://10.10.96.110:81/aloha')
        self.topic_name = rospy.get_param('topic_name', 'websocket_messages')

        # param
        self.middle_value = rospy.get_param('middle_value', 2420)
        self.deadband = rospy.get_param('deadband',300)

        self.topic_pub = rospy.Publisher(self.topic_name, String, queue_size=10)

    def apply_deadband(self, increase):
        if abs(increase) > self.deadband:
            return increase - self.deadband if increase > 0 else increase + self.deadband
        return 0
    
    def resolve_json(self,message):
        # 解析JSON数据
            try:
                data = json.loads(message)
                # 处理解析后的数据，例如：
                x_axis = data.get("x", 2420)
                y_axis = data.get("y", 2420)

                x_increase = x_axis - self.middle_value
                y_increase = y_axis - self.middle_value

                x_increase = self.apply_deadband(x_increase)
                y_increase = self.apply_deadband(y_increase)

                self.topic_pub.publish(String(message))
                rospy.loginfo(f"message: {message} x_increase:{x_increase},y_increase:{y_increase}")

            except json.JSONDecodeError as e:
                rospy.logerr(f"JSON decode error: {e}")
                
    async def websocket_client(self,uri):
        while not rospy.is_shutdown():
            async with websockets.connect(uri) as websocket:
                rospy.loginfo(f"Connected to WebSocket server at {uri}")
                
                while not rospy.is_shutdown():
                    try:
                        message = await asyncio.wait_for(websocket.recv(), timeout=0.5)
                        self.resolve_json(message)
                        
                    except asyncio.TimeoutError:
                        rospy.logwarn("No message received within timeout period")
                        await websocket.close()
                        break
                    except websockets.exceptions.ConnectionClosed as e:
                        rospy.logerr(f"WebSocket connection closed: {e}")
                        break

    def loop(self):
        # Start the WebSocket client in an asyncio event loop
        loop = asyncio.get_event_loop()
        try:
            loop.run_until_complete(self.websocket_client(self.websocket_uri))
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS Interrupt")
        except Exception as e:
            rospy.logerr(f"Exception: {e}")
        finally:
            loop.close()

def main():
    rospy.init_node('websocket_to_ros_node')
    node = joystick()
    node.loop()

if __name__ == '__main__':
    main()
