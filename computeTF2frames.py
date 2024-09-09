#!/usr/bin/env python3

import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('look_up_pepper_tf')

    listener = tf.TransformListener()
    trans = []
    rot = []

    rate = rospy.Rate(10.0)
    while not trans:
        try:
            (trans, rot) = listener.lookupTransform('/imu', '/rslidar', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    
    print('Translation: ', trans)
    print('Rotation: ', rot)

