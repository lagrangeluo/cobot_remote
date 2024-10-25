#!/usr/bin/env python3
import signal
import sys
import math
import argparse
from scipy.spatial.transform import Rotation

import airbot
import time
import Robot



P1=[-135,-121,322,-147,28,68]
squrt_2 = math.sqrt(2) / 2
x_base = -300
y_base = -100
z_base = 100

rx_base = 180
ry_base = 45
rz_base = 0

bot = airbot.create_agent(
    end_mode="encoder", bigarm_type="encoder", forearm_type="encoder"
)

def init_frrobot():
    robot = Robot.RPC('192.168.33.232')
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
   
def main(run_mode):

    if run_mode == 'run':
        robot = init_frrobot()
        tool = robot.GetActualTCPNum()[1]
        user = robot.GetActualWObjNum()[1]
        try:
            while True:
                arm_joint_pos = bot.get_current_joint_q()
                gripper_joint_pos = bot.get_current_end()
                xyz, xyzw = bot.get_current_pose()
                eef_pose = xyz + xyzw
                print("xyz",xyz)
                print("End-effector pose:", eef_pose)
                
                #robot.MoveL(J1,P1,0,0,100.0,180.0,100.0,-1.0,eP1,0,1 ,dP1)   #笛卡尔空间直线运动
                pos_ctl = transform(xyz,xyzw)
                print("pos_ctl",pos_ctl)
                robot.ResetAllError()
                robot.MoveCart(pos_ctl,tool,user,vel=100,acc=70,blendT=500)       #笛卡尔空间点到点运动
                #robot.ServoCart(0,pos_ctl,acc=50,vel=20,filterT=0.1)
                #robot.MoveL(pos_ctl,tool,user,vel=20,acc=50,blendR=50)
                time.sleep(0.05)
        except KeyboardInterrupt:
            print('Exiting due to Ctrl + C')
    elif run_mode == 'encoder':
        try:
            while True:
                arm_joint_pos = bot.get_current_joint_q()
                gripper_joint_pos = bot.get_current_end()
                xyz, xyzw = bot.get_current_pose()
                eef_pose = xyz + xyzw
                print("xyz",xyz)
                print("End-effector pose:", eef_pose)
                
                pos_ctl = transform(xyz,xyzw)
                print("pos_ctl",pos_ctl)
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print('Exiting due to Ctrl + C')
    else:
        print("无效的模式，请选择 'run' 或 'test'.")


    
if __name__ == '__main__':
    # 创建解析器
    parser = argparse.ArgumentParser(description='程序启动参数示例')

    # 添加参数
    parser.add_argument('-r', '--run_mode', type=str, required=True, 
                        choices=['run', 'encoder'], 
                        help="启动模式：'run' 运行模式，'encoder' 测试模式")
    args = parser.parse_args()
    
    # 等待通信正常
    time.sleep(0.5)
    
    main(args.run_mode)
