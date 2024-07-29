import numpy as np

def vector_between_points(p1, p2):
    return np.array(p2) - np.array(p1)

def angle_between_vectors(v1, v2):
    unit_vector_1 = v1 / np.linalg.norm(v1)
    unit_vector_2 = v2 / np.linalg.norm(v2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    return np.degrees(angle)

def calculate_finger_angles(hand_landmarks):
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
    
    for finger, points in fingers.items():
        # 计算每个关节的角度
        v1 = vector_between_points(hand_landmarks[points[0]], hand_landmarks[points[1]])
        v2 = vector_between_points(hand_landmarks[points[1]], hand_landmarks[points[2]])
        angle1 = angle_between_vectors(v1, v2)
        
        # v1 = vector_between_points(hand_landmarks[points[1]], hand_landmarks[points[2]])
        # v2 = vector_between_points(hand_landmarks[points[2]], hand_landmarks[points[3]])
        # angle2 = angle_between_vectors(v1, v2)
        
        # finger_angles[finger] = (angle1, angle2)
        finger_angles[finger] = angle1
    
    return finger_angles

# 示例输入：21个关键点的坐标
hand_landmarks = [
    (0, 0, 0), (1, 1, 0), (2, 2, 0), (3, 3, 0), (4, 4, 0),  # 拇指
    (0, 0, 1), (1, 1, 1), (2, 2, 1), (3, 3, 1), (4, 4, 1),  # 食指
    (0, 0, 2), (1, 1, 2), (2, 2, 2), (3, 3, 2), (4, 4, 2),  # 中指
    (0, 0, 3), (1, 1, 3), (2, 2, 3), (3, 3, 3), (4, 4, 3)   # 小指
]

angles = calculate_finger_angles(hand_landmarks)
for finger, angle in angles.items():
    print(f"{finger} angles: {angle}")
