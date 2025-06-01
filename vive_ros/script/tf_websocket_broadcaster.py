#!/usr/bin/env python3
import rospy
import tf
import json
import asyncio
import signal
import websockets
from piper_msgs.msg import PosCmd
from sensor_msgs.msg import JointState

#TF_FRAME_1 = ("libsurvive_world", "right_hand")
TF_FRAME_1 = ("tracker_base_right", "right_hand")

#TF_FRAME_2 = ("world", "robot2/base_link")
WEBSOCKET_PORT = 8765

clients = set()
arm_publisher = rospy.Publisher('/pin_pos_cmd', PosCmd, queue_size=10)



def transform_to_dict(trans, rot):
    rpy = tf.transformations.euler_from_quaternion(rot)
    return {
        "x": round(trans[0],2),
        "y": round(trans[1],2),
        "z": round(trans[2],2),
        "roll": round(rpy[0],2),
        "pitch": round(rpy[1],2),
        "yaw": round(rpy[2],2)
    }

async def broadcast_loop():
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            # 获取tf坐标
            listener.waitForTransform(*TF_FRAME_1, rospy.Time(0), rospy.Duration(1.0))
            trans1, rot1 = listener.lookupTransform(*TF_FRAME_1, rospy.Time(0))
            # listener.waitForTransform(*TF_FRAME_2, rospy.Time(0), rospy.Duration(1.0))
            # trans2, rot2 = listener.lookupTransform(*TF_FRAME_2, rospy.Time(0))
            robot_dict = transform_to_dict(trans1, rot1)
            print(f"robot dict: {robot_dict}")

            # json打包并通过ws发送
            data = json.dumps({
                "robot1": robot_dict,
                # "robot2": transform_to_dict(trans2, rot2)
            })

            if clients:
                await asyncio.gather(*(client.send(data) for client in clients))
                
            # test:发送话题测试匹诺曹逆解
            target_msg = PosCmd()
            target_msg.x = robot_dict["x"]
            target_msg.y = robot_dict["y"]
            target_msg.z = robot_dict["z"]
            target_msg.roll = robot_dict["yaw"]
            target_msg.pitch = robot_dict["pitch"]
            target_msg.yaw = robot_dict["roll"]
            
            arm_publisher.publish(target_msg)

        except Exception as e:
            rospy.logwarn_throttle(5.0, f"Transform error: {e}")
        await asyncio.sleep(0.1)

async def handler(websocket, path):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.remove(websocket)

async def main():
    rospy.init_node("tf_websocket_broadcaster", anonymous=True)

    shutdown_event = asyncio.Event()

    # 注册 Ctrl+C 处理
    def handle_sigint(signum, frame):
        rospy.loginfo("Shutting down gracefully...")
        shutdown_event.set()

    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    server = await websockets.serve(handler, "0.0.0.0", WEBSOCKET_PORT)
    rospy.loginfo(f"WebSocket server started on port {WEBSOCKET_PORT}")

    broadcaster_task = asyncio.create_task(broadcast_loop())

    await shutdown_event.wait()

    # 清理资源
    broadcaster_task.cancel()
    server.close()
    await server.wait_closed()
    rospy.loginfo("WebSocket server closed.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
