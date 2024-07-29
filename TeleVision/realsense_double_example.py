import numpy as np
np.set_printoptions(precision=2, suppress=True)

import time
import cv2
import pyrealsense2 as rs
from TeleVision import TeleVision

resolution = (720, 1280)  # Adjust based on RealSense camera capabilities

# Configure the first RealSense camera
pipeline_1 = rs.pipeline()
config_1 = rs.config()

# Find the device serial numbers
context = rs.context()
device_list = context.query_devices()
if len(device_list) < 2:
    print("Need at least two RealSense devices connected")
    exit()

device_1 = device_list[0]
device_2 = device_list[1]

serial_number_1 = device_1.get_info(rs.camera_info.serial_number)
serial_number_2 = device_2.get_info(rs.camera_info.serial_number)

# Configure the first camera to stream
config_1.enable_device(serial_number_1)
config_1.enable_stream(rs.stream.color, resolution[1], resolution[0], rs.format.rgb8, 30)
# config_1.enable_stream(rs.stream.depth, resolution[1], resolution[0], rs.format.z16, 30)

# Configure the second RealSense camera
pipeline_2 = rs.pipeline()
config_2 = rs.config()

# Configure the second camera to stream
config_2.enable_device(serial_number_2)
config_2.enable_stream(rs.stream.color, resolution[1], resolution[0], rs.format.rgb8, 30)
# config_2.enable_stream(rs.stream.depth, resolution[1], resolution[0], rs.format.z16, 30)

# Start streaming from both cameras
pipeline_1.start(config_1)
pipeline_2.start(config_2)

tv = TeleVision(resolution,stereo=True)

try:
    while True:
        start = time.time()
        
        # Wait for a coherent pair of frames: depth and color
        frames_1 = pipeline_1.wait_for_frames()
        color_frame_1 = frames_1.get_color_frame()
        # depth_frame_1 = frames_1.get_depth_frame()

        frames_2 = pipeline_2.wait_for_frames()
        color_frame_2 = frames_2.get_color_frame()
        # depth_frame_2 = frames_2.get_depth_frame()

        if not color_frame_1 or not color_frame_2:
            continue

        # Convert images to numpy arrays
        color_image_1 = np.asanyarray(color_frame_1.get_data())
        color_image_2 = np.asanyarray(color_frame_2.get_data())
        
        # Simulate left and right images for demonstration
        # In a real application, these would be captured from a stereo camera setup
        rgb_left = color_image_1
        rgb_right = color_image_2

        tv.modify_shared_image(np.vstack((rgb_left, rgb_right)))
        
        end = time.time()
        # print(f"Frame time: {end - start:.2f} seconds")

except KeyboardInterrupt:
    print("Interrupted by user, shutting down.")
finally:
    pipeline_1.stop()
    pipeline_2.stop()
