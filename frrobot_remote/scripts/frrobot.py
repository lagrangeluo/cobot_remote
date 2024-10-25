#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String

import signal
import sys
import math
from scipy.spatial.transform import Rotation

import time
import Robot



P1=[-135,-121,322,-147,28,68]
p_limit = [170.0,80.0,150.0,80.0,170.0,160.0]
n_limit = [-170.0,-260.0,-150.0,-260.0,-170.0,-160.0]

squrt_2 = math.sqrt(2) / 2
x_base = -300
y_base = -100
z_base = 100

rx_base = 180
ry_base = 45
rz_base = 0

def init_frrobot():
    robot = Robot.RPC('192.168.58.2')
    return robot
    
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
    print("欧拉角 (角度):", euler_angles_deg)
    
    x = x_base - (teach_arm_tcp[0] * 1000)*squrt_2 + (teach_arm_tcp[2]*1000)*squrt_2
    y = y_base + teach_arm_tcp[1] * 1000
    z = z_base + (teach_arm_tcp[2] * 1000)*squrt_2 + (teach_arm_tcp[0]*1000)*squrt_2
    rx = rx_base #+ euler_angles_deg[2]
    ry = ry_base #+ euler_angles_deg[1]
    rz = rz_base #+ euler_angles_deg[2]
    follow_arm_tcp=[x,y,z,rx,ry,rz]
    return follow_arm_tcp


# 初始化fr机械臂通信节点
robot = init_frrobot()
tool = robot.GetActualTCPNum()[1]
user = robot.GetActualWObjNum()[1]

robot.SetLimitPositive(p_limit)
robot.SetLimitNegative(n_limit)
    
def fr_callback(msg):
    json_data = msg.data
    pos_ctl = json.loads(json_data)
    #rospy.info("GET cmd msg: %s",pos_ctl)
    #robot.MoveL(J1,P1,0,0,100.0,180.0,100.0,-1.0,eP1,0,1 ,dP1)   #笛卡尔空间直线运动
    robot.ResetAllError()
    robot.MoveCart(pos_ctl,tool,user,vel=100,acc=70,blendT=100)       #笛卡尔空间点到点运动
    actual_pose = robot.GetActualTCPPose(flag = 1)
    print("actual_pose:",actual_pose)

    #robot.ServoCart(0,pos_ctl,acc=50,vel=20,filterT=0.1)
    #robot.MoveL(pos_ctl,tool,user,vel=20,acc=50,blendR=50)
    
if __name__ == '__main__':
    
    # 初始化ros节点
    rospy.init_node("remote_encoder")
    cmd_sub = rospy.Subscriber("/frrobot_cmd", String, fr_callback)
    rospy.spin()