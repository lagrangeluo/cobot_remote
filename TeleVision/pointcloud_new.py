#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import struct
import asyncio
from multiprocessing import Process, Manager
import numpy as np
from vuer import Vuer
from vuer.events import Set, ClientEvent
from vuer.schemas import DefaultScene, PointCloud

app = Vuer(host='0.0.0.0',port = 8012,cert="./cert.pem", key="./key.pem")

def pointcloud_callback(msg, shared_data):
    points_list = []
    colors_list = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        x, y, z, rgb = point
        points_list.append([x, y, z])
        # print("rgb",point)
        
        # 提取颜色
        rgb = struct.unpack('I', struct.pack('f', rgb))[0]
        r = (rgb >> 16) & 0x0000ff
        g = (rgb >> 8) & 0x0000ff
        b = rgb & 0x0000ff
        # print("r ",r,"g ",g,"b ",b)

        colors_list.append([r / 255.0, g / 255.0, b / 255.0])

    shared_data['points'] = np.array(points_list)
    shared_data['colors'] = np.array(colors_list)

def run_vuer_server():
    app.run()

# @app.spawn
# async def main(proxy):
#     app.set @ DefaultScene()

#     while True:
#         await asyncio.sleep(1.0)

async def update_pointcloud(proxy,shared_data):

    proxy @ Set (DefaultScene())
    
    while not rospy.is_shutdown():
        # print("enter while")
        if shared_data['points'] is not None and shared_data['colors'] is not None:
            proxy.upsert(
                PointCloud(
                    key="pointcloud_111",
                    vertices=np.array(shared_data['points']),
                    colors=np.array(shared_data['colors']),
                    position=[0, 1, 0],
                    rotation=[3.14, 0, 0],
                    size=0.008,
                ),
                # y-up
                # DefaultScene(up=[0, 1, 0]),
            )

        print("update pointcloud")
        await asyncio.sleep(0.2)

def main():

    manager = Manager()
    shared_data = manager.dict()
    shared_data['points'] = None
    shared_data['colors'] = None

    # 初始化 ROS 节点
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback, callback_args=shared_data)

    app.spawn(start=False)(lambda proxy: update_pointcloud(proxy,shared_data))
    process = Process(target=run_vuer_server)
    process.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")
        rospy.signal_shutdown('Shutting down...')
        process.join()

def signal_handler(signal, frame):
    print('You pressed Ctrl + C!')
    rospy.signal_shutdown('Shutting down...')

if __name__ == "__main__":
    import signal
    signal.signal(signal.SIGINT, signal_handler)
    main()
