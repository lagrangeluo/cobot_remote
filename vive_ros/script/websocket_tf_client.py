#!/usr/bin/env python3
import asyncio
import json
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import websockets

WEBSOCKET_URI = "ws://192.168.1.51:8765"  # 请替换成你服务端的实际 IP

class TFWebSocketClient(Node):
    def __init__(self):
        super().__init__('tf_websocket_client')
        self.broadcaster = TransformBroadcaster(self)

    def handle_message(self, message):
        try:
            data = json.loads(message)
            robot1 = data.get("robot1", {})

            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "libsurvive_world"
            tf_msg.child_frame_id = "right_hand"

            tf_msg.transform.translation.x = robot1.get("x", 0.0)
            tf_msg.transform.translation.y = robot1.get("y", 0.0)
            tf_msg.transform.translation.z = robot1.get("z", 0.0)

            q = tf_transformations.quaternion_from_euler(
                robot1.get("roll", 0.0),
                robot1.get("pitch", 0.0),
                robot1.get("yaw", 0.0)
            )
            tf_msg.transform.rotation.x = q[0]
            tf_msg.transform.rotation.y = q[1]
            tf_msg.transform.rotation.z = q[2]
            tf_msg.transform.rotation.w = q[3]

            self.broadcaster.sendTransform(tf_msg)

        except Exception as e:
            self.get_logger().error(f"[Parse Error] {e}")

async def websocket_loop(node: TFWebSocketClient):
    while rclpy.ok():
        try:
            node.get_logger().info(f"Connecting to {WEBSOCKET_URI}")
            async with websockets.connect(WEBSOCKET_URI) as websocket:
                node.get_logger().info("WebSocket connected.")
                async for message in websocket:
                    node.handle_message(message)
        except Exception as e:
            node.get_logger().warn(f"WebSocket error: {e}")
            await asyncio.sleep(2.0)  # 等待重连

def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TFWebSocketClient()

    # 启动 ROS2 spin 线程
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()

    try:
        asyncio.run(websocket_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()