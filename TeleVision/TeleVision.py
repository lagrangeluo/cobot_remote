from vuer import Vuer
from vuer.schemas import ImageBackground, Hands,Movable,Gripper
from multiprocessing import Process, Array, Value, shared_memory
import numpy as np
from scipy.spatial.transform import Rotation as R
import asyncio

import rospy
from inspire_hand.srv import set_angle
from inspire_hand.msg import set_angle_topic

class TeleVision:
    def __init__(self, img_shape, stereo=False):
        self.stereo = stereo

        self.app = Vuer(host='0.0.0.0',port = 8012,cert="./cert.pem", key="./key.pem")

        self.app.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app.add_handler("CAMERA_MOVE")(self.on_cam_move)
        self.app.spawn(start=False)(self.main)

        self.img_shape = (2*img_shape[0], img_shape[1], 3)
        self.img_height, self.img_width = img_shape[:2]
        self.shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
        self.shm_name = self.shm.name
        self.shared_image = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.shm.buf)
        self.shared_image[:] = np.zeros(self.img_shape, dtype=np.uint8)

        self.left_hand_shared = Array('d', 16, lock=True)
        self.right_hand_shared = Array('d', 16, lock=True)
        self.left_landmarks_shared = Array('d', 75, lock=True)
        self.right_landmarks_shared = Array('d', 75, lock=True)
        self.left_hand_flag = False
        self.right_hand_flag = False
        
        self.head_matrix_shared = Array('d', 16, lock=True)
        self.aspect_shared = Value('d', 1.0, lock=True)
        self.head_position_shared = Array('d', 3, lock=True)
        self.head_init_flag = False

        # ros相关
        self.inspire_hand_client = rospy.ServiceProxy('/inspire_hand/set_angle', set_angle)
        self.inspire_hand_pub = rospy.Publisher('/inspire_hand/set_angle_topic',set_angle_topic,queue_size=1)
        self.hand_msg = set_angle_topic()
        self.process = Process(target=self.run)
        self.process.start()

    def run(self):
        self.app.run()

    def update_hand_msg(self, inspire_angle_list):
        self.hand_msg.angle0 = inspire_angle_list['pinky']
        self.hand_msg.angle1 = inspire_angle_list['ring']
        self.hand_msg.angle2 = inspire_angle_list['middle']
        self.hand_msg.angle3 = inspire_angle_list['index']
        self.hand_msg.angle4 = inspire_angle_list['thumb']
        self.hand_msg.angle5 = inspire_angle_list['thumb_axis']
        
        self.inspire_hand_pub.publish(self.hand_msg)

    async def on_cam_move(self, event, session):
        try:
            with self.head_matrix_shared.get_lock():  # Use the lock to ensure thread-safe updates
                self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            with self.aspect_shared.get_lock():
                self.aspect_shared.value = event.value['camera']['aspect']
            with self.head_position_shared.get_lock():
                self.head_position_shared[:] = event.value['camera']['position']
            # print(event.value)
            # print(event.value['camera']['position'])
            # print(event.value['camera']['rotation'])
        except:
            pass

    async def on_hand_move(self, event, session):
        try:
            with self.left_hand_shared.get_lock():  # Use the lock to ensure thread-safe updates
                self.left_hand_shared[:] = event.value["leftHand"]
            with self.right_hand_shared.get_lock():
                self.right_hand_shared[:] = event.value["rightHand"]
            with self.left_landmarks_shared.get_lock():
                self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
            with self.right_landmarks_shared.get_lock():
                self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
        
            # print(np.array(event.value))
        except: 
            pass

    async def main(self, session, fps=60):
        session.upsert @ Hands(fps=fps, stream=True, key="hands")
        session.upsert @ Movable(Gripper(key="not-moving",position=[0, 1.2, -0.2]))
        while True:
            display_image = self.shared_image

            if not self.stereo:
                session.upsert(
                ImageBackground(
                    display_image[:self.img_height],
                    format="jpg",
                    quality=80,
                    key="left-image",
                    interpolate=True,
                    aspect=1.778,
                    distanceToCamera=2,
                    position=[0, 0, -2],
                    rotation=[0, 0, 0],
                ),
                to="bgChildren",
                )
            else:
                session.upsert(
                [ImageBackground(
                    display_image[:self.img_height],
                    format="jpg",
                    quality=40,
                    key="left-image",
                    interpolate=True,
                    aspect=1.778,
                    distanceToCamera=2,
                    layers=1,
                    position=[-4, 0, -2],
                    rotation=[0, 0, 0],
                ),
                ImageBackground(
                    display_image[self.img_height:],
                    format="jpg",
                    quality=40,
                    key="right-image",
                    interpolate=True,
                    aspect=1.778,
                    distanceToCamera=2,
                    layers=2,
                    position=[4, 0, -2],
                    rotation=[0, 0, 0],
                )],
                to="bgChildren",
                )
            await asyncio.sleep(1/fps)

    def modify_shared_image(self, img, random=False):
        assert img.shape == self.img_shape, f"Image shape must be {self.img_shape}, got {img.shape}"
        existing_shm = shared_memory.SharedMemory(name=self.shm_name)
        shared_image = np.ndarray(self.img_shape, dtype=np.uint8, buffer=existing_shm.buf)
        shared_image[:] = img[:] if not random else np.random.randint(0, 256, self.img_shape, dtype=np.uint8)
        existing_shm.close()

    def is_shared_array_empty(self,shared_array):
        # 将共享数组转换为 numpy 数组
        array = np.frombuffer(shared_array.get_obj())
        # 检查是否所有元素都是零
        return np.all(array == 0)
    
    @property
    def left_hand(self):
        with self.left_hand_shared.get_lock():
            return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")
    
    @property
    def right_hand(self):
        if self.right_hand_flag == False:
            if not self.is_shared_array_empty(self.right_hand_shared):
                self.right_hand_flag = True

        with self.right_hand_shared.get_lock():
            if self.right_hand_flag == True:
                # print(np.array(self.right_hand_shared))

                # 将共享矩阵转换为 numpy 数组
                right_matrix = np.array(self.right_hand_shared).reshape(4, 4)
                # 提取平移向量
                translation = right_matrix[:3, 3]
                # 提取旋转矩阵
                rotation_matrix = right_matrix[:3, :3]
                # 将旋转矩阵转换为欧拉角 (RPY)
                rotation = R.from_matrix(rotation_matrix)
                rpy = rotation.as_euler('xyz', degrees=True)
                # 打印平移和旋转
                print("Translation (x, y, z):", translation)
                print("Rotation (roll, pitch, yaw):", rpy)

                return np.array(self.right_hand_shared)
    
    
    @property
    def left_landmarks(self):
        with self.left_landmarks_shared.get_lock():
            # angles = calculate_finger_angles(hand_landmarks)
            # for finger, angle in angles.items():
            #     print(f"{finger} angles: {angle}")

            return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
    
    @property
    def right_landmarks(self):
        if self.right_hand_flag == False:
            if not self.is_shared_array_empty(self.right_hand_shared):
                self.right_hand_flag = True

        with self.right_landmarks_shared.get_lock():
            if self.right_hand_flag == True:
                right_landmarks = np.array(self.right_landmarks_shared[:]).reshape(25, 3)
                # print(right_landmarks)
                angles = self.calculate_finger_angles(right_landmarks)
                # for finger, angle in angles.items():
                #     print(f"{finger} angles: {angle}")

                return angles

    @property
    def head_matrix(self):
        if self.head_init_flag == False:
            if not self.is_shared_array_empty(self.head_matrix_shared):
                self.head_init_flag = True

        with self.head_matrix_shared.get_lock():
            if self.head_init_flag == True:
                # 将共享矩阵转换为 numpy 数组
                head_matrix = np.array(self.head_matrix_shared).reshape(4, 4)
                # 提取旋转矩阵
                rotation_matrix = head_matrix[:3, :3]
                # 将旋转矩阵转换为欧拉角 (RPY)
                rotation = R.from_matrix(rotation_matrix)
                rpy = rotation.as_euler('xyz', degrees=True)
                # 打印平移和旋转
                # print("Translation (x, y, z):", translation)
                print("Rotation (roll, pitch, yaw):", rpy)
        with self.head_position_shared.get_lock():
                if self.head_init_flag == True:
                    position_x = self.head_position_shared[0]
                    position_y = self.head_position_shared[1]
                    position_z = self.head_position_shared[2]
                    print("Translation (x, y, z):",position_x,position_y,position_z)

                # return np.array(self.head_matrix_shared)

    @property
    def aspect(self):
        with self.aspect_shared.get_lock():
            return float(self.aspect_shared.value)

    # 手指角度-->inspire灵巧手关节,1000为手掌平摊的角度，0为关节弯曲90度
    def finger_angle_to_inspire(self,angle):
        scale = 1000/90
        inspire_angle = 1000 - scale * angle
        return int(inspire_angle)
    
    def vector_between_points(self,p1, p2):
        return np.array(p2) - np.array(p1)

    def angle_between_vectors(self,v1, v2):
        unit_vector_1 = v1 / np.linalg.norm(v1)
        unit_vector_2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        angle = np.arccos(dot_product)
        return np.degrees(angle)
    
    def project_vector_on_plane(self,vector, plane_normal):
        """计算向量在给定平面上的投影"""
        plane_normal = plane_normal / np.linalg.norm(plane_normal)
        return vector - np.dot(vector, plane_normal) * plane_normal
    
    def calculate_plane_normal(self,vector1, vector2):
        """
        计算两个向量定义的平面的法向量
        """
        normal_vector = np.cross(vector1, vector2)
        normalized_normal_vector = normal_vector / np.linalg.norm(normal_vector)
        return normalized_normal_vector
    
    def calculate_finger_angles(self, hand_landmarks):
        # 手指关节点索引
        # 行顺序：大拇指，食指，中指，无名指，小拇指
        # 列顺序：掌骨，指骨近端，指骨远端，指尖
        finger_name = ['wrist', 
                        'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip', 
                        'index-finger-metacarpal', 'index-finger-phalanx-proximal', 'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip', 
                        'middle-finger-metacarpal', 'middle-finger-phalanx-proximal', 'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip', 
                        'ring-finger-metacarpal', 'ring-finger-phalanx-proximal', 'ring-finger-phalanx-intermediate', 'ring-finger-phalanx-distal', 'ring-finger-tip', 
                        'pinky-finger-metacarpal', 'pinky-finger-phalanx-proximal', 'pinky-finger-phalanx-intermediate', 'pinky-finger-phalanx-distal', 'pinky-finger-tip']

        fingers = {
            'thumb': [1, 2, 3, 4],
            'index': [5, 6, 7, 8, 9],
            'middle': [10, 11, 12, 13, 14],
            'ring': [15, 16, 17, 18, 19],
            'pinky': [20, 21, 22, 23, 24]
        }
        
        # 最终手指角度
        finger_angles = {}
        
        # 计算5个手指根部角度
        for finger, points in fingers.items():
            # 计算每个关节的角度
            v1 = self.vector_between_points(hand_landmarks[points[0]], hand_landmarks[points[1]])
            v2 = self.vector_between_points(hand_landmarks[points[1]], hand_landmarks[points[2]])
            angle1 = self.angle_between_vectors(v1, v2)
            
            finger_angles[finger] = angle1
        
        #计算大拇指根部另一个转动角度
        index_vector = self.vector_between_points(hand_landmarks[fingers['index'][0]], hand_landmarks[fingers['index'][1]])
        ring_vector = self.vector_between_points(hand_landmarks[fingers['ring'][0]], hand_landmarks[fingers['ring'][1]])
        middle_vector = self.vector_between_points(hand_landmarks[fingers['middle'][0]], hand_landmarks[fingers['middle'][1]])
        thumb_vector = self.vector_between_points(hand_landmarks[fingers['thumb'][0]], hand_landmarks[fingers['thumb'][1]])
        
        # 先计算手掌平面法向量
        palm_normal_vector = self.calculate_plane_normal(index_vector,ring_vector)

        # 计算大拇指根部向量投影到手腕切平面
        thumb_map_vector = self.project_vector_on_plane(thumb_vector,middle_vector)

        # 计算大拇指投影后向量和手掌平面法线的角度
        thumb_angle = self.angle_between_vectors(thumb_map_vector,palm_normal_vector)

        finger_angles['thumb_axis'] = thumb_angle

        return finger_angles