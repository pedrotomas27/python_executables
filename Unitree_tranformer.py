#!/usr/bin/env python3

import rospy
from unitree_legged_msgs.msg import HighState
from sensor_msgs.msg import Imu
import json

def callback(data):
    # Extract IMU data from the HighState message
    imu_data = data.imu
    
    # Create a new IMU message
    imu_msg = Imu()
    
    # Populate header information
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "base"

    # Populate orientation, angular velocity, and linear acceleration data
    imu_msg.orientation.x = imu_data.quaternion[0]
    imu_msg.orientation.y = imu_data.quaternion[1]
    imu_msg.orientation.z = imu_data.quaternion[2]
    imu_msg.orientation.w = imu_data.quaternion[3]
    
    imu_msg.angular_velocity.x = imu_data.gyroscope[0]
    imu_msg.angular_velocity.y = imu_data.gyroscope[1]
    imu_msg.angular_velocity.z = imu_data.gyroscope[2]
    
    imu_msg.linear_acceleration.x = imu_data.accelerometer[0]
    imu_msg.linear_acceleration.y = imu_data.accelerometer[1]
    imu_msg.linear_acceleration.z = imu_data.accelerometer[2]
    imu_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    imu_msg.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    imu_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    # Publish the IMU data
    imu_pub.publish(imu_msg)

def listener():
    rospy.init_node('imu_extractor', anonymous=True)
    rospy.Subscriber("/high_state", HighState, callback)
    rate = rospy.Rate(20)
    global imu_pub
    imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

