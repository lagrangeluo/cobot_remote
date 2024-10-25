#!/usr/bin/env python3
import rospy
from frrobot_remote.msg import frrobot
import json
from std_msgs.msg import String

import signal
import sys
import math
import argparse
from scipy.spatial.transform import Rotation
import numpy as np

import airbot
import time

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.animation import FuncAnimation

squrt_2 = math.sqrt(2) / 2
x_base = -300
y_base = 0
z_base = 200

rx_base = 180
ry_base = 45
rz_base = 0

bot = airbot.create_agent(
    end_mode="encoder", bigarm_type="encoder", forearm_type="encoder"
)


###########################

def abs_compare(current_cmd,last_cmd):
    for i in range(6):
        if abs(current_cmd[0] - last_cmd[0]) >= 0.4:
            return True
    return False
            
# 定义绕Y轴旋转的函数
def rotation_matrix_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

# 定义绕X轴旋转的函数
def rotation_matrix_x(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

# 定义绕Z轴旋转的函数
def rotation_matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

def rotation_matrix_to_rpy(R):
    # 提取 RPY 角度
    yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw (z轴旋转)
    pitch = np.arcsin(-R[2, 0])         # Pitch (y轴旋转)
    roll = np.arctan2(R[2, 1], R[2, 2]) # Roll (x轴旋转)
    
    # 将角度从弧度转换为度
    yaw = np.degrees(yaw)
    pitch = np.degrees(pitch)
    roll = np.degrees(roll)
    
    return yaw, pitch, roll

def transfrom_rotation(yaw_before,pitch_before,roll_before):
    # 坐标1的位置
    C1 = np.array([0, 0, 0])  # 坐标1位置

    # 坐标2相对坐标1的位置
    # 假设坐标2在坐标1的右侧，旋转45度
    rotation_C2 = rotation_matrix_y(np.radians(45))

    # 基于坐标2的 RPY 旋转角度（例如：roll = 30°, pitch = 15°, yaw = 45°）
    roll = np.radians(roll_before)
    pitch = np.radians(pitch_before)
    yaw = np.radians(yaw_before)

    # 计算基于坐标2的旋转矩阵
    R_C2 = rotation_matrix_z(yaw) @ rotation_matrix_y(pitch) @ rotation_matrix_x(roll)

    # 将坐标2的旋转应用于坐标1
    # 将坐标2的旋转矩阵转换到坐标1
    R_C1 = rotation_C2 @ R_C2

    yaw_C1, pitch_C1, roll_C1 = rotation_matrix_to_rpy(R_C1)
    return yaw_C1,pitch_C1,roll_C1

###########################
def signal_handler(signal,frame):
    print('You pressed Ctrl + C!')
    #stop the timer update thread
    #rospy.signal_shutdown('Shutting down...')
    sys.exit(0)

def transform(teach_arm_tcp,qua):
    follow_arm_tcp=[]
    quaternion = [qua[3], qua[0], qua[1], qua[2]]
    r = Rotation.from_quat(quaternion)
    euler_angles_deg = r.as_euler('xyz', degrees=True)
    # 欧拉角进行变换补偿
    if euler_angles_deg[0] < -100:
        euler_angles_deg[0] += 180
    if euler_angles_deg[0] > 100:
        euler_angles_deg[0] -= 180
    
    #euler_angles_deg[1] += 135
    print("欧拉角 (角度):", euler_angles_deg)

    x = x_base - (teach_arm_tcp[0] * 1000)*squrt_2 + (teach_arm_tcp[2]*1000)*squrt_2
    y = y_base + teach_arm_tcp[1] * 1000
    z = z_base + (teach_arm_tcp[2] * 1000)*squrt_2 + (teach_arm_tcp[0]*1000)*squrt_2
    
    yaw_after,pitch_after,roll_after = transfrom_rotation(euler_angles_deg[0],euler_angles_deg[1],euler_angles_deg[2])
    rx = rx_base #- euler_angles_deg[2]  #roll
    ry = ry_base #+ euler_angles_deg[1]  #pitch
    rz = rz_base #- euler_angles_deg[0] #yaw
    # rx = rx_base - yaw_after  #roll
    # ry = ry_base -45 + pitch_after  #pitch
    # rz = rz_base - roll_after #yaw

    follow_arm_tcp=[x,y,z,rx,ry,rz]
    return follow_arm_tcp
   
def main():
    
    # 初始化ros节点
    rospy.init_node("remote_encoder")
    #cmd_pub = rospy.Publisher("/frrobot_cmd",frrobot,queue_size=5)
    cmd_pub = rospy.Publisher("/frrobot_cmd",String,queue_size=5)
    last_cmd=[0,0,0,0,0,0]
    cmd_msg=[]
    
    
    try:
        while not rospy.is_shutdown():
            arm_joint_pos = bot.get_current_joint_q()
            gripper_joint_pos = bot.get_current_end()
            xyz, xyzw = bot.get_current_pose()
            eef_pose = xyz + xyzw
            print("xyz",xyz)
            print("End-effector pose:", eef_pose)
            
            pos_ctl = transform(xyz,xyzw)
            
            pos_ctl[1] = -80 -pos_ctl[1]
            # 发布机械臂控制消息
            #cmd_msg = frrobot()
            cmd_msg = [pos_ctl[0],pos_ctl[1],pos_ctl[2],pos_ctl[3],pos_ctl[4],pos_ctl[5]]
            print("pos_ctl",pos_ctl)
            if not last_cmd:
                last_cmd = cmd_msg.copy()  # 初始化 last_cmd
            elif abs_compare(cmd_msg, last_cmd):
                json_data = json.dumps(cmd_msg)
                cmd_pub.publish(json_data)

            last_cmd = cmd_msg.copy()
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print('Exiting due to Ctrl + C')


    
if __name__ == '__main__':
    main()
