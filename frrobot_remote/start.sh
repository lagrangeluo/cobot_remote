#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/embodyrobotics/remote_ws/devel/setup.bash
cd /home/embodyrobotics/remote_ws/src/cobot_remote/frrobot_remote/scripts
python3 encoder.py
