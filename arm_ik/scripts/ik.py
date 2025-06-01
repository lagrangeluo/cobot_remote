#!/bin/python3
import os
from ikpy import chain
import numpy as np
import math
import rospy
import time
from pytransform3d import rotations
from sensor_msgs.msg import JointState
import tf2_ros
import tf
import tf.transformations as tf_trans
from std_msgs.msg import Header


class ik_caculator():
    def __init__(self):
        # 初始化ros 相关组件
        self.pub_right = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.pub_left = rospy.Publisher('/target_pose/left', JointState, queue_size=10)
        rospy.loginfo("ik caculator init")
        self.joint_state = JointState()
        self.joint_state.name = ['j1', 'j2', 'j3', 'j4','j5' ,'j6']
        self.pre_result = [0,0,0,0,0,0,0]

        # 创建 TF2 监听器和缓冲区
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # 定义坐标系
        self.parent_frame = 'tracker_base_right'
        self.grab_frame = 'right_hand'

        ###### Left Arm ######
        left_arm_links = [
                            "base_link",
                            "shoulder_Link",
                            "upperarm_Link",
                            "forearm_Link",
                            "wrist1_Link",
                            "wrist2_Link",
                            ]

        left_arm_joints = [
                            "j1",
                            "j2",
                            "j3",
                            "j4",
                            "j5",
                            "j6",
                            ]

        left_arm_elements = [x for pair in zip(left_arm_links, left_arm_joints) for x in pair] + ["wrist3_Link"]
        #rospy.loginfo(f'{left_arm_elements}')
        # Note: 'Links' in IKPY correspond to 'Joints' in URDF terminology. 'Links' in URDF are stripped by IKPY. 
        parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        urdf_file_path = os.path.join(parent_dir, "urdf","fr5_robot.urdf")
        
        self.left_arm_chain = chain.Chain.from_urdf_file(
            urdf_file_path,
            base_elements=left_arm_elements,
            #last_link_vector=[0.261, 0, -0.018],   # 如果这里没有腕关节, 该向量为t_elbow_wrist平移向量
            active_links_mask=[False]*0 + 7 * [True],  
            symbolic=False,
            name="left_arm")

        self.left_arm_chain.to_json_file(force=True)
        rospy.loginfo("init chain success")

        qpos = np.zeros(7)
        # 初始化变换矩阵和旋转矩阵
        self.Transform_init = self.left_arm_chain.forward_kinematics(qpos)
        self.Transfrom_Rotation_init = self.Transform_init[:3,:3]

        qua = rotations.quaternion_from_matrix(self.Transform_init[:3,:3])
        #print("init q:", qua)
        #print("rotation: ",self.Transform_init[:3, :3])

    
    def init_arm(self,puppet="all"):

        init_position = [-0.4, 0, 0.3]
        init_orientation = self.Transform_init[:3,:3]
        print("start init arm in 3 seconds")
        q_init = self.left_arm_chain.inverse_kinematics(init_position,init_orientation,'all')
        q_init = [round(a,3) for a in q_init]

        joint_state_init = JointState()
        joint_state_init.position = [q_init[1],q_init[2],q_init[3],q_init[4],q_init[5],q_init[6]]
        joint_state_init.name = self.joint_state.name
        joint_state_init.header = Header()
        joint_state_init.header.stamp = rospy.Time.now()
        if puppet == "right" or puppet=="all":
            self.pub_right.publish(joint_state_init)
        if puppet == "left" or puppet=="all":
            self.pub_left.publish(joint_state_init)
        rospy.loginfo("arm init success!")

    # def calculate_angle_with_tf2(self):

    #     while not rospy.is_shutdown():
    #         try:
    #             # 获取 base_link 到 grab_link 的变换
    #             transform: TransformStamped = self.tf_buffer.lookup_transform(
    #                 "base_link", "grab_link", rospy.Time(0))

    #             # 提取旋转四元数
    #             quat = transform.transform.rotation
    #             quaternion = [quat.x, quat.y, quat.z, quat.w]

    #             # 将四元数转换为旋转矩阵
    #             rot_matrix = tf.transformations.quaternion_matrix(quaternion)

    #             # 提取 grab_link 的 z 轴方向（在 base_link 坐标系中）
    #             z_axis = rot_matrix[:3, 2]  # 旋转矩阵的第三列是 z 轴方向

    #             # 投影到 xz 平面
    #             z_xz = np.array([z_axis[0], 0, z_axis[2]])

    #             # 归一化投影向量
    #             z_xz_normalized = z_xz / np.linalg.norm(z_xz)

    #             # 计算 z 轴与 xz 平面的夹角
    #             angle = np.arctan2(np.linalg.norm(np.cross([0, 0, 1], z_xz_normalized)), np.dot([0, 0, 1], z_xz_normalized))

    #             # 将弧度转为角度
    #             angle_deg = np.degrees(angle)

    #             rospy.loginfo("Angle between grab_link z-axis and base_link xz-plane: {:.2f} degrees".format(angle_deg))
    #             return angle_deg

    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             rospy.logwarn("Transform not available, retrying...")


    def get_target_tcp(self,puppet="right"):
        try:
            
            transform = self.tf_buffer.lookup_transform(self.parent_frame, self.grab_frame, rospy.Time(0))
            # 打印平移和旋转信息
            # rospy.loginfo("Translation: x=%f, y=%f, z=%f", transform.transform.translation.x, 
            #                                             transform.transform.translation.y, 
            #                                             transform.transform.translation.z)
            # rospy.loginfo("Rotation: x=%f, y=%f, z=%f, w=%f", transform.transform.rotation.x,
            #                                             transform.transform.rotation.y,
            #                                             transform.transform.rotation.z,
            #                                             transform.transform.rotation.w)
            
            rotation = transform.transform.rotation
            # 将旋转四元数转换为旋转矩阵
            rotation_matrix = tf_trans.quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])
            # 提取3X3旋转矩阵
            rotation_matrix_3x3 = rotation_matrix[:3, :3]
            # camera_link 坐标系的 z 轴是旋转矩阵的第三列
            z_axis_camera_link_in_base_link = rotation_matrix[:3, 2]
            # z 轴的归一化坐标（如果需要的话）
            norm = np.linalg.norm(z_axis_camera_link_in_base_link)
            normalized_z_axis = z_axis_camera_link_in_base_link / norm
            #rospy.loginfo(f"{grab_link} z 轴在 {parent_link} 坐标系下的归一化坐标: {normalized_z_axis}")
            theta_radians = math.atan2(normalized_z_axis[1], normalized_z_axis[0])
            # 构造旋转矩阵
            R_z = np.array([
                [math.cos(theta_radians), -math.sin(theta_radians), 0],
                [math.sin(theta_radians), math.cos(theta_radians), 0],
                [0, 0, 1]
            ])
            R_rotation = np.dot(R_z, self.Transfrom_Rotation_init)
            
            return {"x":transform.transform.translation.x,"y":transform.transform.translation.y,"z":transform.transform.translation.z,
                    "rotation":rotation_matrix_3x3,"theta":theta_radians}
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #rospy.logerr(f"ik caculator: Got TF from {self.parent_frame} to {self.grab_frame} failed!, escape run process")
            return None


    # 输入目标姿态列表，执行逆解：target_position:(x,y,z),target_orientation:Matrix(3:3)
    # 若不指定position和orientation，则自动默认为初始位置的姿态角
    # step_list:[x,y,z]，该列表代表在原有坐标基础上是否要添加偏移量
    def run(self,target_position=None,target_orientation=None,puppet="right"):
        #rospy.loginfo("Start plan arm")
        # 获取并断言 target_position
        if target_position is not None:
            # 断言 target_position 是一个列表
            assert isinstance(target_position, np.ndarray), "ik caculator: target_position should be a np.ndarray"
            assert target_position.shape[0] == 3, "ik caculator: target_position should have exactly 3 elements"
        else:
            target_pose = self.get_target_tcp(puppet=puppet)
            if target_pose is None:
                #rospy.logerr("ik caculator: target pose is None , break down current task")
                return
            else:
                target_position=[target_pose["x"],target_pose["y"],target_pose["z"]]
        
        #rospy.loginfo(f"target position: {target_position}")

        # 获取并断言 target_orientation 是一个 3x3 的矩阵
        if target_orientation is not None:
            assert isinstance(target_orientation, (list, np.ndarray)), "ik caculator: target_orientation should be a list or numpy array"
            if isinstance(target_orientation, list):
                assert len(target_orientation) == 3, "ik caculator: target_orientation should be a list with 3 rows"
                for row in target_orientation:
                    assert len(row) == 3, "ik caculator: Each row of target_orientation should have exactly 3 elements"
            elif isinstance(target_orientation, np.ndarray):
                assert target_orientation.shape == (3, 3), "ik caculator: target_orientation should be a 3x3 matrix"
        else:
            # 获取初始姿态
            #target_orientation = self.Transfrom_Rotation_init
            # 获取法向量姿态
            target_orientation = target_pose["rotation"]

        # 执行逆解并发布消息
        q = self.left_arm_chain.inverse_kinematics(target_position,target_orientation,'all',initial_position = np.array(self.pre_result))        
        # 保留joint list小数点后三位小数
        q = [round(a,3) for a in q]
        self.pre_result = q
        # 判断joint list里的数值是否为零,q[3]第三个关节角度是否为零，如果为零则逆解计算错误，返回
        if((all(x == 0 for x in q)) or q[3] == 0):
            return
        
        rospy.loginfo(f"ik joint angle: {q}")

        self.joint_state.position = [q[1],q[2],q[3],q[4],q[5],q[6]]
        self.joint_state.header = Header()
        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.header.stamp = rospy.Time.now()
        if puppet=="right":
            #rospy.logwarn("start command right arm")
            self.pub_right.publish(self.joint_state)
        elif puppet=="left":
            rospy.logwarn("start command left arm")
            self.pub_left.publish(self.joint_state)

    
if __name__=='__main__' :
    # rosnode
    rospy.init_node("pyik_node")
    node = ik_caculator()
    node.init_arm()
    while not rospy.is_shutdown():
        node.run()
    #rospy.spin()
