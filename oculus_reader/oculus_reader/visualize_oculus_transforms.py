#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from oculus_reader import OculusReader
from tf.transformations import quaternion_from_matrix
import rospy
import tf2_ros
import numpy as np
import geometry_msgs.msg


def publish_transform(transform, name):
    r_t = np.array([
        [0,0,-1,0],
        [-1,0,0,0],
        [0,1,0,0],
        [0,0,0,1]
    ])

    # rotate
    transform = np.dot(r_t,transform)

    translation = transform[:3, 3]

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'oculus_head'
    t.child_frame_id = name
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]


    quat = quaternion_from_matrix(transform)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    br.sendTransform(t)


def main():
    oculus_reader = OculusReader(ip_address='10.10.43.3')
    rospy.init_node('oculus_reader')

    rate_param = rospy.get_param('/oculus/rate', 50)
    rate = rospy.Rate(rate_param)

    while not rospy.is_shutdown():
        rate.sleep()
        transformations, buttons = oculus_reader.get_transformations_and_buttons()
        if 'r' not in transformations:
            continue

        right_controller_pose = transformations['r']
        # left_controller_pose = transformations['l']
        publish_transform(right_controller_pose, 'hand_left')
        # publish_transform(left_controller_pose, 'oculus_left')
        print('transformations', transformations)
        print('buttons', buttons)

if __name__ == '__main__':
    main()
