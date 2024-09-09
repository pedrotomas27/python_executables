#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import numpy as np

class ImuFilter:
    def __init__(self):
        self.sub = rospy.Subscriber('/imu_data', Imu, self.callback)
        self.pub = rospy.Publisher('/filtered_imu_data', Imu, queue_size=10)

        # Low pass filter parameter for all values
        self.alpha = 0.5
        self.filtered_imu = Imu()

    def low_pass_filter(self, data):
        self.filtered_imu.orientation.x = self.alpha * data.orientation.x + (1 - self.alpha) * self.filtered_imu.orientation.x
        self.filtered_imu.orientation.y = self.alpha * data.orientation.y + (1 - self.alpha) * self.filtered_imu.orientation.y
        self.filtered_imu.orientation.z = self.alpha * data.orientation.z + (1 - self.alpha) * self.filtered_imu.orientation.z
        self.filtered_imu.orientation.w = self.alpha * data.orientation.w + (1 - self.alpha) * self.filtered_imu.orientation.w

        self.filtered_imu.angular_velocity.x = self.alpha * data.angular_velocity.x + (1 - self.alpha) * self.filtered_imu.angular_velocity.x
        self.filtered_imu.angular_velocity.y = self.alpha * data.angular_velocity.y + (1 - self.alpha) * self.filtered_imu.angular_velocity.y
        self.filtered_imu.angular_velocity.z = self.alpha * data.angular_velocity.z + (1 - self.alpha) * self.filtered_imu.angular_velocity.z

        self.filtered_imu.linear_acceleration.x = self.alpha * data.linear_acceleration.x + (1 - self.alpha) * self.filtered_imu.linear_acceleration.x
        self.filtered_imu.linear_acceleration.y = self.alpha * data.linear_acceleration.y + (1 - self.alpha) * self.filtered_imu.linear_acceleration.y
        self.filtered_imu.linear_acceleration.z = self.alpha * data.linear_acceleration.z + (1 - self.alpha) * self.filtered_imu.linear_acceleration.z

    def callback(self, data):
        # Apply low-pass filter to all IMU values
        self.low_pass_filter(data)

        # Set the header of filtered IMU data
        self.filtered_imu.header.stamp = rospy.Time.now()
        self.filtered_imu.header.frame_id = data.header.frame_id

        # Set covariance values (unchanged)
        self.filtered_imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.filtered_imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        self.filtered_imu.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Publish the filtered IMU data
        self.pub.publish(self.filtered_imu)

if __name__ == '__main__':
    rospy.init_node('imu_filter', anonymous=True)
    imu_filter = ImuFilter()
    rospy.spin()

