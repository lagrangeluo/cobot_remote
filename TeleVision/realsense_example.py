import numpy as np
np.set_printoptions(precision=2, suppress=True)

import time
import cv2
import pyrealsense2 as rs
import rospy
from TeleVision import TeleVision

resolution = (720, 1280)  # Adjust based on RealSense camera capabilities

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Configure the pipeline to stream both color and depth frames
config.enable_stream(rs.stream.color, resolution[1], resolution[0], rs.format.rgb8, 30)
config.enable_stream(rs.stream.depth, resolution[1], resolution[0], rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# init ros node
rospy.init_node('pointcloud_listener', anonymous=True)
inspire_angle_list = {}
tv = TeleVision(resolution)

try:
    while not rospy.is_shutdown():
        start = time.time()
        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        
        # Simulate left and right images for demonstration
        # In a real application, these would be captured from a stereo camera
        rgb_left = color_image
        rgb_right = color_image.copy()

        tv.modify_shared_image(np.vstack((rgb_left, rgb_right)))

        # tv.head_matrix
        # tv.right_hand
        
        right_hand = tv.right_landmarks

        if right_hand is not None:
            for finger, angle in right_hand.items():
                if finger == 'thumb_axis':
                    inspire_angle_list[finger] = 1000 - tv.finger_angle_to_inspire(angle)
                    continue
                
                inspire_angle_list[finger] = tv.finger_angle_to_inspire(angle)

            # print(inspire_angle_list)
            # tv.inspire_hand_client(inspire_angle_list['pinky'],inspire_angle_list['ring'],inspire_angle_list['middle'],
            #                    inspire_angle_list['index'],inspire_angle_list['thumb'],inspire_angle_list['thumb_axis'])
            tv.update_hand_msg(inspire_angle_list)            


        end = time.time()
        total_time = end - start
        # print("spend time: %f",total_time)
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Interrupted by user, shutting down.")
finally:
    pipeline.stop()
