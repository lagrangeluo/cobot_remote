#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from asyncio import sleep, get_event_loop, ensure_future
import numpy as np
from multiprocessing import Process, Manager
import time
import signal
from vuer import Vuer
from vuer.events import Set
from vuer.schemas import DefaultScene, PointCloud

app = Vuer(static_root="",port = 8010)

def pointcloud_callback(msg, shared_data):
    points_list = []
    colors_list = []
    
    for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
        x, y, z, rgb = point
        points_list.append([x, y, z])
        
        # 提取颜色
        r = (int(rgb) >> 16) & 0x0000ff
        g = (int(rgb) >> 8) & 0x0000ff
        b = int(rgb) & 0x0000ff
        colors_list.append([r / 255.0, g / 255.0, b / 255.0])
    
    shared_data['points'] = np.array(points_list)
    shared_data['colors'] = np.array(colors_list)

def run():
    app.run()

async def update_pointcloud(proxy, shared_data):
    while not rospy.is_shutdown():

        if shared_data['points'] is not None and shared_data['colors'] is not None:
            # 更新点云数据到 Vuer
            proxy @ Set(
                DefaultScene(
                    PointCloud(
                        key="pointcloud_111",
                        vertices=np.array(shared_data['points']),
                        # colors=np.array(shared_data['colors']),
                        position=[0, 0, 0],
                        size=0.008,
                    ),
                    # y-up
                    to="bgChildren",
                ),
            )
        await sleep(1)

def main():
    manager = Manager()
    shared_data = manager.dict()
    shared_data['points'] = None
    shared_data['colors'] = None

    # 初始化 ROS 节点
    rospy.init_node('pointcloud_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback, callback_args=shared_data)

    app.spawn(start=False)(lambda proxy: update_pointcloud(proxy, shared_data))
    process = Process(target=run)
    process.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")
        rospy.signal_shutdown('Shutting down...')

def signal_handler(signal, frame):
    print('You pressed Ctrl + C!')
    rospy.signal_shutdown('Shutting down...')

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
